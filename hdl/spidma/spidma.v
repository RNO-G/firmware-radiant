`timescale 1ns / 1ps
// The SPI DMA module handles mass data handling from/to the SPI path.
//
// DMA works by having a set of descriptors and length.
// But we're not going to bounce around types, so that doesn't make sense.
// So we only need 20 bits for address, 1 bit for a terminator, and 12 bits for length.
// That's 34 bits total. However, our address space doesn't *use* those top 2 bits yet
// (and I'm going to slice up the RadID space for the DMA control, and embed the
//  event path inside the trigger space, so we're not *expanding* anything).
// So our access model will work like:
// address 0x00 - 0x7F : control space
// address 0x80 - 0xFF : standard descriptor space
//
// address 0x00: general control
// bit [0] - enable
// bit [1] - running
// bit [2] - external DMA trigger enabled
// bit [3] - DMA direction (to/from SPI)
// bit [4] - increment address  - NO LONGER USED - it's in the descriptor
// bit [5] - byte mode
// bit [7:6] - byte source/destination
// bit [8] - enable receive FIFO (from SPI path)
// bits[15:9] - cycle delay
// bits[27:16] - SPI flag indicator threshold
// bit [30] - SPI flag indicator value
// bit [31] - enable SPI flag output
// address 0x01:
// bit [0] - reset TX path
// bit [1] - reset RX path
// bit [2] - reset engine
// bit [3] - soft trigger (force initiate DMA)
// address 0x02: descriptor being processed
// address 0x03: DMA transaction counter (write to reset) 
//
// The BB can rip data out pretty fast by setting the indicator threshold to something
// like 256 or something and then just straight dumping the entire event. The SPI
// core is *far* slower than the internal bus speed, so once there's a bit of a buffer
// in the SPI path, it should just stream at speed.
//
// Note that because SPI is constantly bidirectional, to avoid needing to reset
// the SPI RX FIFO all the time, the receive FIFO can be enabled/disabled to just
// ignore MOSI.
// (Obviously the transmit FIFO doesn't need to be).
// 
// Descriptor:
// bit [17:0]   wishbone address
// bit [18]     this descriptor increments addresses
// bit [30:19]  cycle count
// bit [31]     last descriptor
`include "wishbone.vh"
`include "radiant_debug.vh"
`include "dsp_macros.vh"
module spidma( input wb_clk_i,
               input wb_rst_i,
               `WBS_NAMED_PORT(wb, 32, 16, 4),
               `WBM_NAMED_PORT(wbdma, 32, 22, 4),
               // External trigger port. A DMA request is accepted
               // when dma_req_i && dma_rdy_o.
               input dma_req_i,
               output dma_rdy_o,
               
               // programmable full trigger
               output PROG_FULL,
               // SPI path.
               input fast_clk_i,
               input CS_B,
               input SCLK,
               input MOSI,
               output MISO               
        );
    
    parameter DEBUG = `SPIDMA_ENGINE_DEBUG;
    
    localparam FSM_BITS = 4;
    localparam [FSM_BITS-1:0] IDLE = 0;
    localparam [FSM_BITS-1:0] DECODE = 1;
    localparam [FSM_BITS-1:0] FROMSPI_FETCH_0 = 2;
    localparam [FSM_BITS-1:0] FROMSPI_FETCH_1 = 3;
    localparam [FSM_BITS-1:0] FROMSPI_FETCH_2 = 4;
    localparam [FSM_BITS-1:0] FROMSPI_FETCH_3 = 5;
    localparam [FSM_BITS-1:0] FROMSPI_WRITE = 6;
    localparam [FSM_BITS-1:0] FROMSPI_NEXT = 7;
    localparam [FSM_BITS-1:0] TOSPI_READ = 8;
    localparam [FSM_BITS-1:0] TOSPI_STORE_0 = 9;
    localparam [FSM_BITS-1:0] TOSPI_STORE_1 = 10;
    localparam [FSM_BITS-1:0] TOSPI_STORE_2 = 11;
    localparam [FSM_BITS-1:0] TOSPI_STORE_3 = 12;
    localparam [FSM_BITS-1:0] TOSPI_NEXT = 13;
    localparam [FSM_BITS-1:0] DONE = 14;
    reg [FSM_BITS-1:0] state = IDLE;
    
    // resets the engine, aborting anything in progress
    reg dma_engine_reset = 0;
    
    // set when a trigger occurs, cleared in done
    reg dma_run = 0;

    // general enable to allow a trigger to cause DMA
    reg dma_enable = 0;

    // soft DMA request
    reg soft_dma_request = 0;

    // if 1, DMA is transferred as a single-byte write
    // if 0, DMA is a full 32 bit transfer
    reg dma_byte_mode = 0;
    // if 1, DMA is from SPI to WB
    // if 0, DMA is WB to SPI
    reg dma_direction = 0;
    // if 1, external requests are allowed
    reg dma_allow_ext_req = 0;
    // if 1, DMA increments addresses. If 0, addresses
    // stay fixed.
    // NO LONGER USED
    //reg dma_incr_address = 0;
    // In byte mode, this indicates *which* byte is being targeted.
    reg [1:0] dma_byte_target = {2{1'b0}};
    
    // Enable the receive SPI path.
    reg enable_spirx = 0;
    (* ASYNC_REG = "TRUE" *)
    reg [1:0] enable_spirx_fastclk = {2{1'b0}};
    
    // Which descriptor is being targeted.
    reg [4:0] cur_descriptor_num = {5{1'b0}};
    
    reg [3:0] descriptor_write = 0;
    
    reg ack = 0;
    
    reg [11:0] transaction_counter = {12{1'b0}};
    
    // Cycle delay is in units of 16 clocks.
    reg [6:0] cycle_delay = {7{1'b0}};
    reg [10:0] cycle_count = {11{1'b0}};
    wire cycle_delay_reached = ({cycle_delay,4'h0} == cycle_count);
            
    wire [31:0] cur_descriptor;
    wire [17:0] descriptor_addr = cur_descriptor[0 +: 18];
    wire        descriptor_addr_incr = cur_descriptor[18];
    wire [11:0] descriptor_len = cur_descriptor[19 +: 12];
    wire        descriptor_done = cur_descriptor[31];
    // Descriptor RAM.
    generate
        genvar i;
        for (i=0;i<4;i=i+1) begin : DR    
            descriptor_ram u_ram( .a(wb_adr_i[2 +: 5]),
                                  .d(wb_dat_i[8*i +: 8]),
                                  .dpra( (dma_enable) ? cur_descriptor_num : wb_adr_i[2 +: 5] ),
                                  .clk(wb_clk_i),
                                  .we( descriptor_write[i] ),
                                  .dpo( cur_descriptor[8*i +: 8] ) );
        end
    endgenerate
        
    // SPI inbound fifo.
    wire [7:0] rx_axis_tdata;
    wire       rx_axis_tvalid;
    wire       rx_axis_tready;
    // at fast clk side
    wire [7:0] spirx_tdata;
    wire       spirx_tvalid;
    wire       spirx_tready;    
    
    // SPI outbound fifo
    reg [7:0]  tx_axis_tdata;
    wire       tx_axis_tvalid;
    wire       tx_axis_tready;
    // at fast clk side
    wire [7:0] spitx_tdata;
    wire       spitx_tvalid;
    wire       spitx_tready;
    
    // these should autoclear
    reg        tx_reset = 1;
    wire       tx_reset_complete;
    reg [1:0]  tx_reset_fastclk = {2{1'b0}};
    SRLC32E    tx_reset_delay(.D(tx_reset),.A(0),.CE(1'b1),.CLK(wb_clk_i),.Q31(tx_reset_complete));
    
    // these should autoclear
    reg        rx_reset = 1;        
    (* ASYNC_REG = "TRUE" *)
    reg [1:0]  rx_reset_fastclk = {2{1'b0}};
    wire       rx_reset_complete;
    SRLC32E    rx_reset_delay(.D(rx_reset),.A(0),.CE(1'b1),.CLK(wb_clk_i),.Q31(rx_reset_complete));

    reg [10:0] spitx_full_threshold = {11{1'b0}};
    reg        enable_spitx_full = 0;
    reg [3:0]  wbdma_sel_reg = {4{1'b0}};
    reg [18:0] wbdma_adr_reg = {19{1'b0}};
    reg [31:0] wbdma_data_reg = {32{1'b0}};

    reg         dma_transaction_counter_reset = 0;
    wire [31:0] dma_transaction_counter;
    spidma_transaction_counter u_counter(.CLK(wb_clk_i),.CE(state == DONE),.SCLR(dma_transaction_counter_reset),
                                         .Q(dma_transaction_counter));
    
//module dsp_counter_terminal_count(
//        input           clk_i,
//        input           rst_i,
//        input           count_i,
//        input [47:0]    tcount_i,
//        input           update_tcount_i,
//        output          tcount_reached_o,
//        output [47:0]   count_o
//    );    
    
    wire prog_full_out;
    
    wire [31:0] dma_control_regs[3:0];    
    assign dma_control_regs[0] = { enable_spitx_full, prog_full_out && enable_spitx_full, {3{1'b0}}, spitx_full_threshold, cycle_delay, enable_spirx,       // 31:8
                                   dma_byte_target, dma_byte_mode, 1'b0, dma_direction, dma_allow_ext_req, dma_run, dma_enable };
    assign dma_control_regs[1] = {32{1'b0}};
    assign dma_control_regs[2] = cur_descriptor_num;
    assign dma_control_regs[3] = dma_transaction_counter;                                   
    
    
    // multiplex the outgoing data
    always @(*) begin
        if (state == TOSPI_STORE_0) tx_axis_tdata <= wbdma_data_reg[7:0];
        else if (state == TOSPI_STORE_1) tx_axis_tdata <= wbdma_data_reg[15:8];
        else if (state == TOSPI_STORE_2) tx_axis_tdata <= wbdma_data_reg[23:16];
        else tx_axis_tdata <= wbdma_data_reg[31:24];
    end
    
    always @(posedge wb_clk_i) begin
        ack <= wb_cyc_i && wb_stb_i;
        if (wb_cyc_i && wb_stb_i && wb_we_i && wb_adr_i[7]) descriptor_write <= wb_sel_i;
        else descriptor_write <= {4{1'b0}};
    
        if (dma_engine_reset) state <= IDLE;
        else begin
            case (state)
                IDLE: if (dma_run) state <= DECODE;
                // Decode figures out where we go next. It doesn't depend on the descriptor at all.
                // Decode is where the length counter increments (when we hit the cycle delay).
                DECODE: if (cycle_delay_reached) begin
                    if (dma_direction) begin
                        // FROMSPI. If we're byte mode, we need to figure out where we hop to.
                        if (dma_byte_mode) begin
                            if (dma_byte_target == 2'b00) state <= FROMSPI_FETCH_0;
                            else if (dma_byte_target == 2'b01) state <= FROMSPI_FETCH_1;
                            else if (dma_byte_target == 2'b10) state <= FROMSPI_FETCH_2;
                            else if (dma_byte_target == 2'b11) state <= FROMSPI_FETCH_3;
                        end else state <= FROMSPI_FETCH_0;
                    end else begin
                        // TOSPI. Here we always go to READ.
                        state <= TOSPI_READ;
                    end
                end
                FROMSPI_FETCH_0: if (rx_axis_tvalid && rx_axis_tready) begin
                    if (dma_byte_mode) state <= FROMSPI_WRITE;
                    else state <= FROMSPI_FETCH_1;
                end
                FROMSPI_FETCH_1: if (rx_axis_tvalid && rx_axis_tready) begin
                    if (dma_byte_mode) state <= FROMSPI_WRITE;
                    else state <= FROMSPI_FETCH_2;
                end
                FROMSPI_FETCH_2: if (rx_axis_tvalid && rx_axis_tready) begin
                    if (dma_byte_mode) state <= FROMSPI_WRITE;
                    else state <= FROMSPI_FETCH_3;
                end
                FROMSPI_FETCH_3: if (rx_axis_tvalid && rx_axis_tready) state <= FROMSPI_WRITE;
                FROMSPI_WRITE: if (wbdma_ack_i || wbdma_err_i || wbdma_rty_i) state <= FROMSPI_NEXT;
                // At either of the NEXT states, the transaction counter increments, and if
                // it hits the final, the descriptor counter increments.
                // The descriptor output then updates in DECODE and is ready and registered
                // the clock after DECODE. 
                FROMSPI_NEXT: if (transaction_counter == descriptor_len && descriptor_done) state <= DONE;
                              else state <= DECODE;
                TOSPI_READ: if (wbdma_ack_i || wbdma_err_i || wbdma_rty_i) begin
                    if (dma_byte_mode) begin
                        if (dma_byte_target == 2'b00) state <= TOSPI_STORE_0;
                        else if (dma_byte_target == 2'b01) state <= TOSPI_STORE_1;
                        else if (dma_byte_target == 2'b10) state <= TOSPI_STORE_2;
                        else if (dma_byte_target == 2'b11) state <= TOSPI_STORE_3;
                    end else state <= TOSPI_STORE_0;
                end
                TOSPI_STORE_0: if (tx_axis_tvalid && tx_axis_tready) begin
                    if (dma_byte_mode) state <= TOSPI_NEXT;
                    else state <= TOSPI_STORE_1;
                end
                TOSPI_STORE_1: if (tx_axis_tvalid && tx_axis_tready) begin
                    if (dma_byte_mode) state <= TOSPI_NEXT;
                    else state <= TOSPI_STORE_2;
                end
                TOSPI_STORE_2: if (tx_axis_tvalid && tx_axis_tready) begin
                    if (dma_byte_mode) state <= TOSPI_NEXT;
                    else state <= TOSPI_STORE_3;
                end
                TOSPI_STORE_3: if (tx_axis_tvalid && tx_axis_tready) state <= TOSPI_NEXT;
                TOSPI_NEXT: if (transaction_counter == descriptor_len && descriptor_done) state <= DONE;
                            else state <= DECODE;
                DONE: state <= IDLE;
            endcase
        end
        // Data register capture.
        if (state == TOSPI_READ && wbdma_ack_i) begin
            wbdma_data_reg <= wbdma_dat_i;
        end else if (rx_axis_tvalid && rx_axis_tready) begin
            if (state == FROMSPI_FETCH_0) wbdma_data_reg[0 +: 8] <= rx_axis_tdata;
            if (state == FROMSPI_FETCH_1) wbdma_data_reg[8 +: 8] <= rx_axis_tdata;
            if (state == FROMSPI_FETCH_2) wbdma_data_reg[16 +: 8] <= rx_axis_tdata;
            if (state == FROMSPI_FETCH_3) wbdma_data_reg[24 +: 8] <= rx_axis_tdata;
        end               
        if (state == DECODE && cycle_delay_reached) begin
            if (descriptor_addr_incr) wbdma_adr_reg <= descriptor_addr + transaction_counter;
            else wbdma_adr_reg <= descriptor_addr;
        end
        if (state == IDLE) transaction_counter <= {12{1'b0}};
        else if (state == FROMSPI_NEXT || state == TOSPI_NEXT) begin
            if (transaction_counter == descriptor_len) transaction_counter <= {12{1'b0}};
            else transaction_counter <= transaction_counter + 1;
        end
        if (state == IDLE) cur_descriptor_num <= {5{1'b0}};
        else if ((state == FROMSPI_NEXT || state == TOSPI_NEXT) && (transaction_counter==descriptor_len)) 
            cur_descriptor_num <= cur_descriptor_num + 1;

        if (dma_engine_reset) dma_run <= 0;
        else if (dma_enable) begin
            if (state == DONE) dma_run <= 0;
            else if (soft_dma_request || (dma_allow_ext_req && dma_req_i)) dma_run <= 1;
        end

        // Super, super simple.
        if (state != DECODE) cycle_count <= {11{1'b0}};
        else if (state == DECODE) cycle_count <= cycle_count + 1;
    
        // Our controllable bits are:
        // dma_direction
        // dma_byte_mode
        // dma_incr_address
        // dma_enable
        // dma_byte_target
        // dma_allow_ext_req
        // then our flags are
        // soft_dma_request
        // reset TX path
        // reset RX path
        // reset DMA engine
        if (dma_engine_reset) begin
            // Engine reset needs to shut off enable to work as a terminate.
            dma_enable <= 0;
        end if (wb_cyc_i && wb_stb_i && wb_we_i && !wb_adr_i[7] && (wb_adr_i[3:2] == 2'b00) && wb_sel_i[0]) begin
            dma_enable <= wb_dat_i[0];
            dma_allow_ext_req <= wb_dat_i[2];
            dma_direction <= wb_dat_i[3];
            //dma_incr_address <= wb_dat_i[4];
            dma_byte_mode <= wb_dat_i[5];
            dma_byte_target <= wb_dat_i[7:6];
            // just... simplify this
        end
        // the 1 cycle delay here's no big deal
        wbdma_sel_reg[0] <= !dma_byte_mode || (dma_byte_target == 2'b00);
        wbdma_sel_reg[1] <= !dma_byte_mode || (dma_byte_target == 2'b01);
        wbdma_sel_reg[2] <= !dma_byte_mode || (dma_byte_target == 2'b10);
        wbdma_sel_reg[3] <= !dma_byte_mode || (dma_byte_target == 2'b11);
        
        if (wb_cyc_i && wb_stb_i && wb_we_i && !wb_adr_i[7] && (wb_adr_i[3:2] == 2'b00)) begin
            if (wb_sel_i[1]) begin
                enable_spirx <= wb_dat_i[8];
                cycle_delay <= wb_dat_i[9 +: 7];
            end
            if (wb_sel_i[2]) spitx_full_threshold[7:0] <= wb_dat_i[16 +: 8];
            if (wb_sel_i[3]) begin
                spitx_full_threshold[10:8] <= wb_dat_i[24 +: 3];
                enable_spitx_full <= wb_dat_i[31];
            end
        end
        
        if (wb_cyc_i && wb_stb_i && wb_we_i && !wb_adr_i[7] && (wb_adr_i[3:2] == 2'b01) && wb_sel_i[0] && wb_dat_i[0])
            tx_reset <= 1;
        else if (tx_reset_complete)
            tx_reset <= 0;
            
        if (wb_cyc_i && wb_stb_i && wb_we_i && !wb_adr_i[7] && (wb_adr_i[3:2] == 2'b01) && wb_sel_i[0] && wb_dat_i[1])
            rx_reset <= 1;
        else if (rx_reset_complete)
            rx_reset <= 0;
            
        if (wb_cyc_i && wb_stb_i && wb_we_i && !wb_adr_i[7] && (wb_adr_i[3:2] == 2'b01) && wb_sel_i[0] && wb_dat_i[3])
            soft_dma_request <= 1;
        else
            soft_dma_request <= 0;
 
        if (wb_cyc_i && wb_stb_i && wb_we_i && !wb_adr_i[7] && (wb_adr_i[3:2] == 2'b01) && wb_sel_i[0] && wb_dat_i[2])
            dma_engine_reset <= 1;
        else
            dma_engine_reset <= 0;       
            
        dma_transaction_counter_reset <= wb_cyc_i && wb_stb_i && wb_we_i && !wb_adr_i[7] && (wb_adr_i[3:2] == 2'b11);                    
    end    

    always @(posedge fast_clk_i) begin
        enable_spirx_fastclk <= { enable_spirx_fastclk[0], enable_spirx };
        rx_reset_fastclk <= { rx_reset_fastclk[0], rx_reset };
        tx_reset_fastclk <= { tx_reset_fastclk[0], tx_reset };
    end
    
    assign tx_axis_tvalid = (state == TOSPI_STORE_0) || (state == TOSPI_STORE_1) || (state == TOSPI_STORE_2) || (state == TOSPI_STORE_3);
    assign rx_axis_tready = (state == FROMSPI_FETCH_0) || (state == FROMSPI_FETCH_1) || (state == FROMSPI_FETCH_2) || (state == FROMSPI_FETCH_3);
    
    // now we need to cross into the fast clock domains
    // add spidma_tx_fifo, spidma_rx_fifo, gate the programmable full, and add the fast_spi_fifo interface
    
    // TX fifo goes from wb_clk_i -> fast_clk_i
    spidma_tx_fifo u_tx_fifo( .s_aclk(wb_clk_i),
                              .s_aresetn(!tx_reset),
                              .s_axis_tdata(tx_axis_tdata),
                              .s_axis_tvalid(tx_axis_tvalid),
                              .s_axis_tready(tx_axis_tready),
                              .m_aclk(fast_clk_i),
                              .m_axis_tdata(spitx_tdata),
                              .m_axis_tready(spitx_tready),
                              .m_axis_tvalid(spitx_tvalid),
                              .axis_prog_full_thresh(spitx_full_threshold[10:0]),
                              .axis_prog_full(prog_full_out));
    
    OBUFT u_progfull(.I(prog_full_out),.O(PROG_FULL),.T(!enable_spitx_full));
                              
    // RX fifo goes from fast_clk_i -> wb_clk_i
    // reset lives in the fastclk domain, but it's really just reclocked over there.
    spidma_rx_fifo u_rx_fifo( .s_aclk(fast_clk_i),
                              .s_aresetn(!rx_reset_fastclk[1]),
                              .s_axis_tdata(spirx_tdata),
                              .s_axis_tvalid(spirx_tvalid),
                              .s_axis_tready(spirx_tready), // not used, data just goes poof when FIFO full
                              .m_aclk(wb_clk_i),
                              .m_axis_tdata(rx_axis_tdata),
                              .m_axis_tvalid(rx_axis_tvalid),
                              .m_axis_tready(rx_axis_tready));
//module fast_spi_fifo(
//        input       aclk,
//        input       aresetn,
//        input [7:0] s_axis_tdata,
//        input       s_axis_tvalid,
//        // tready gets double-used here. Interrupt goes when
//        // the FIFO has 254 bytes (at least) *and* tready has been
//        // low for at least 2 clock cycles. This indicates that
//        // 256 bytes can be freely read by SPI.
//        output      s_axis_tready,        
        
//        // RX path. No tready.
//        input        enable_rx,
//        output [7:0] m_axis_tdata,
//        output       m_axis_tvalid,
        
//        input CS_B,
//        input SCLK,
//        input MOSI,
//        output MISO
//    );

    // fast SPI fifo
    fast_spi_fifo u_spi( .aclk(fast_clk_i),
                         .aresetn( !tx_reset_fastclk[1] ),
                         .s_axis_tdata( spitx_tdata ),
                         .s_axis_tvalid(spitx_tvalid),
                         .s_axis_tready(spitx_tready),
                         
                         .enable_rx( enable_spirx_fastclk[1] ),
                         .m_axis_tdata( spirx_tdata ),
                         .m_axis_tvalid(spirx_tvalid),
                         
                         .CS_B(CS_B),
                         .SCLK(SCLK),
                         .MOSI(MOSI),
                         .MISO(MISO));                                                            

    generate
        if (DEBUG == "TRUE") begin : ILA
            spidma_ila u_ila(.clk(wb_clk_i),
                             .probe0(state),
                             .probe1(descriptor_addr),
                             .probe2(descriptor_addr_incr),
                             .probe3(descriptor_len),
                             .probe4(descriptor_done),
                             .probe5(cur_descriptor_num),
                             .probe6(transaction_counter));
         end
     endgenerate

    assign wbdma_dat_o = wbdma_data_reg;
    assign wbdma_adr_o = {1'b0,wbdma_adr_reg,2'b00};
    assign wbdma_sel_o = wbdma_sel_reg;
    assign wbdma_cyc_o = (state == FROMSPI_WRITE || state == TOSPI_READ);
    assign wbdma_stb_o = wbdma_cyc_o;
    assign wbdma_we_o = (state == FROMSPI_WRITE);

    assign wb_ack_o = ack;
    assign wb_dat_o = (wb_adr_i[7]) ? cur_descriptor : dma_control_regs[wb_adr_i[3:2]];
    assign wb_err_o = 1'b0;
    assign wb_rty_o = 1'b0;
        
    assign dma_rdy_o = (dma_enable && dma_allow_ext_req && !dma_run);
endmodule
