`timescale 1ns / 1ps
module boardman_interface(
        input clk,
        input rst,
        // we lose 23 to write address
        // we lose 22 to board manager
        // leaves 21:0, or 22 bits.
        // This is a byte address, so it gives us an address space of 4 MiB.
        // We output a 32-bit address so we also lose 1:0, leaving us
        // with 19:0.
        output [19:0] adr_o,
        output [31:0] dat_o,
        input  [31:0] dat_i,
        output        en_o,
        output        wr_o,
        output [3:0]  wstrb_o,
        input         ack_i,
        
        input BM_RX,
        output BM_TX        
    );
    
    parameter CLOCK_RATE = 100000000;
    parameter BAUD_RATE = 1000000;    
    
    reg [23:0] address = {24{1'b0}};
    reg [23:0] capture_address = {24{1'b0}};
    reg en = 0;
    reg wr = 0;
    reg [3:0] wstrb = {4{1'b0}};
    reg [31:0] data = {32{1'b0}};
    
    reg [7:0] len = {8{1'b0}};
    reg write_last = 0;
    
    localparam ACC_BITS = 10;
    localparam real CLOCK_REAL = CLOCK_RATE;
    localparam real BAUD_REAL_X16 = BAUD_RATE*16;
    localparam real BAUD_DIVIDE = CLOCK_REAL/BAUD_REAL_X16;
    localparam real ACC_MAX = (1 << ACC_BITS);
    localparam real ACC_VAL = ACC_MAX/BAUD_DIVIDE;
    localparam ACC_VAL_X2 = ACC_VAL*2;
    // get a fixed bit value here
    localparam [10:0] BRG_ACCUMULATOR_VALUE_X2 = ACC_VAL_X2;
    // this rounds the above. For 1 MBaud this should be 164.
    localparam [9:0] BRG_ACCUMULATOR_VALUE = BRG_ACCUMULATOR_VALUE_X2[10:1] + BRG_ACCUMULATOR_VALUE_X2[0];
    reg [10:0] acc = {11{1'b0}};
    always @(posedge clk) begin
        acc <= acc[9:0] + BRG_ACCUMULATOR_VALUE;
    end
    wire en_16x_baud = acc[10];
    
    wire [7:0] cobs_rx_tdata;
    wire cobs_rx_tvalid;
    wire cobs_rx_tready;
    wire cobs_rx_tlast = (cobs_rx_tdata == 8'h00);
    
    wire [7:0] axis_rx_tdata;
    wire axis_rx_tvalid;
    wire axis_rx_tready;
    wire axis_rx_tlast;
    wire axis_rx_tuser;
    
    reg [7:0] axis_tx_tdata = {8{1'b0}};
    wire axis_tx_tvalid;
    wire axis_tx_tready;
    reg axis_tx_tlast = 0;
    
    wire [7:0] cobs_tx_tdata;
    wire cobs_tx_full;
    wire cobs_tx_tready = !cobs_tx_full;
    wire cobs_tx_tvalid;
        
    localparam FSM_BITS=5;
    localparam [FSM_BITS-1:0] IDLE=0;
    // address comes in big-endian, just freaking because
    localparam [FSM_BITS-1:0] ADDR2=1;
    localparam [FSM_BITS-1:0] ADDR1=2;
    localparam [FSM_BITS-1:0] ADDR0=3;
    localparam [FSM_BITS-1:0] READLEN=4;
    localparam [FSM_BITS-1:0] READADDR2=5;
    localparam [FSM_BITS-1:0] READADDR1=6;
    localparam [FSM_BITS-1:0] READADDR0=7;
    localparam [FSM_BITS-1:0] READCAPTURE=8;
    localparam [FSM_BITS-1:0] READDATA0=9;
    localparam [FSM_BITS-1:0] READDATA1=10;
    localparam [FSM_BITS-1:0] READDATA2=11;
    localparam [FSM_BITS-1:0] READDATA3=12;    
    // data comes in little-endian, as if it's byte-addressed
    localparam [FSM_BITS-1:0] WRITE0=13;
    localparam [FSM_BITS-1:0] WRITE1=14;
    localparam [FSM_BITS-1:0] WRITE2=15;
    localparam [FSM_BITS-1:0] WRITE3=16;
    localparam [FSM_BITS-1:0] WRITEEN=17;
    localparam [FSM_BITS-1:0] WRITEADDR2=18;
    localparam [FSM_BITS-1:0] WRITEADDR1=19;
    localparam [FSM_BITS-1:0] WRITEADDR0=20;
    localparam [FSM_BITS-1:0] WRITELEN=21;
    localparam [FSM_BITS-1:0] RESET0 = 22;
    localparam [FSM_BITS-1:0] RESET1 = 23;
    // yikes this is big
    reg [FSM_BITS-1:0] state = RESET0;

    always @(posedge clk) begin
        case (state)
            IDLE: if (axis_rx_tvalid && !axis_rx_tlast && !axis_rx_tuser) state <= ADDR2;
            ADDR2: if (axis_rx_tvalid) begin
                if (axis_rx_tlast || axis_rx_tuser) state <= IDLE;
                else state <= ADDR1;
            end
            ADDR1: if (axis_rx_tvalid) begin
                if (axis_rx_tlast || axis_rx_tuser) state <= IDLE;
                else state <= ADDR0;
            end
            ADDR0: if (axis_rx_tvalid) begin
                if (axis_rx_tlast || axis_rx_tuser) state <= IDLE;
                else begin
                    if (capture_address[23]) begin
                        // figure out where we start
                        if (capture_address[1:0] == 2'b00) state <= WRITE0;
                        else if (capture_address[1:0] == 2'b01) state <= WRITE1;
                        else if (capture_address[1:0] == 2'b10) state <= WRITE2;
                        else if (capture_address[1:0] == 2'b11) state <= WRITE3;
                    end else state <= READLEN;
                end
            end
            READLEN: if (axis_rx_tvalid) begin
                // TLAST *should* be asserted here.
                if (axis_rx_tuser || !axis_rx_tlast) state <= IDLE;
                else state <= READADDR2;
            end
            // This writes the address back, byte by byte
            READADDR2: if (axis_tx_tready) state <= READADDR1;
            READADDR1: if (axis_tx_tready) state <= READADDR0;
            READADDR0: if (axis_tx_tready) state <= READCAPTURE;
            READCAPTURE: if (ack_i) begin
                // figure out where the read starts
                if (address[1:0] == 2'b00) state <= READDATA0;
                else if (address[1:0] == 2'b01) state <= READDATA1;
                else if (address[1:0] == 2'b10) state <= READDATA2;
                else if (address[1:0] == 2'b11) state <= READDATA3;
            end
            READDATA0: if (axis_tx_tready) begin
                if (!len) state <= IDLE;
                else state <= READDATA1;
            end
            READDATA1: if (axis_tx_tready) begin
                if (!len) state <= IDLE;
                else state <= READDATA2;
            end
            READDATA2: if (axis_tx_tready) begin
                if (!len) state <= IDLE;
                else state <= READDATA3;
            end
            READDATA3: if (axis_tx_tready) begin
                if (!len) state <= IDLE;
                else state <= READCAPTURE;
            end
            // soooo... this will do wackadoodle things if an
            // error comes in the middle. Maybe buffer the writes.
            // Check that later.
            WRITE0: if (axis_rx_tvalid) begin
                if (axis_rx_tuser) state <= IDLE;
                else if (axis_rx_tlast) state <= WRITEEN;
                else state <= WRITE1;
            end
            WRITE1: if (axis_rx_tvalid) begin
                if (axis_rx_tuser) state <= IDLE;
                else if (axis_rx_tlast) state <= WRITEEN;
                else state <= WRITE2;
            end
            WRITE2: if (axis_rx_tvalid) begin
                if (axis_rx_tuser) state <= IDLE;
                else if (axis_rx_tlast) state <= WRITEEN;
                else state <= WRITE3;
            end
            WRITE3: if (axis_rx_tvalid) begin
                if (axis_rx_tuser) state <= IDLE;
                else state <= WRITEEN;
            end
            WRITEEN: if (ack_i) begin
                if (write_last) state <= WRITEADDR2;
                else state <= WRITE0;
            end
            WRITEADDR2: if (axis_tx_tready) state <= WRITEADDR1;
            WRITEADDR1: if (axis_tx_tready) state <= WRITEADDR0;
            WRITEADDR0: if (axis_tx_tready) state <= WRITELEN;
            WRITELEN: if (axis_tx_tready) state <= IDLE;
            RESET0: state <= RESET1;
            RESET1: state <= IDLE;
        endcase
        
        // deal with the address increments
        if ((state == WRITEEN && ack_i) || (state == READDATA3 && axis_tx_tready)) 
            address <= { 2'b00, address[21:2], 2'b00 } + 4;
        else if (state == ADDR0)
            address <= {capture_address[23:8],axis_rx_tdata};
        
        if (state == ADDR2) capture_address[23:16] <= axis_rx_tdata;
        if (state == ADDR1) capture_address[15:8] <= axis_rx_tdata;
        if (state == ADDR0) capture_address[7:0] <= axis_rx_tdata;
    
        if (state == WRITE0 && axis_rx_tvalid) data[7:0] <= axis_rx_tdata;
        else if (state == READCAPTURE && ack_i) data[7:0] <= dat_i[7:0];
        
        if (state == WRITE1 && axis_rx_tvalid) data[15:8] <= axis_rx_tdata;
        else if (state == READCAPTURE && ack_i) data[15:8] <= dat_i[15:8];
        
        if (state == WRITE2 && axis_rx_tvalid) data[23:16] <= axis_rx_tdata;
        else if (state == READCAPTURE && ack_i) data[23:16] <= dat_i[23:16];
        
        if (state == WRITE3 && axis_rx_tvalid) data[31:24] <= axis_rx_tdata;
        else if (state == READCAPTURE && ack_i) data[31:24] <= dat_i[31:24];
        
        if (state == IDLE || (state == WRITEEN && ack_i)) wstrb <= {4{1'b0}};
        else begin
            if (state == WRITE0) wstrb[0] <= 1;
            if (state == WRITE1) wstrb[1] <= 1;
            if (state == WRITE2) wstrb[2] <= 1;
            if (state == WRITE3) wstrb[3] <= 1;
        end
        
        if (state == IDLE) write_last <= 0;
        else if (state == WRITE0 || state == WRITE1 || 
                 state == WRITE2 || state == WRITE3) begin
            if (axis_rx_tvalid && axis_rx_tlast && !axis_rx_tuser) 
                write_last <= 1;
            else 
                write_last <= 0;
        end
        
        // length handling
        if (state == IDLE) len <= {8{1'b0}};
        else if ((state == WRITE0 || state == WRITE1 || state == WRITE2 || state == WRITE3) && axis_rx_tvalid) len <= len + 1;
        else if ((state == READDATA0 || state == READDATA1 || state == READDATA2 || state == READDATA3) && axis_tx_tready) len <= len - 1;
        else if ((state == READLEN) && axis_rx_tvalid) len <= axis_rx_tdata;
        
        // outgoing data determination. Here we need to prep things the cycle before.
        if (state == READLEN || state == WRITEEN) 
            axis_tx_tdata <= capture_address[23:16];
        else if ((state == READADDR2 || state == WRITEADDR2) && axis_tx_tready)
            axis_tx_tdata <= capture_address[15:8];
        else if ((state == READADDR1 || state == WRITEADDR1) && axis_tx_tready)
            axis_tx_tdata <= capture_address[7:0];
        else if (state == READCAPTURE && ack_i) begin
            if (address[1:0] == 2'b00) axis_tx_tdata <= dat_i[7:0];
            else if (address[1:0] == 2'b01) axis_tx_tdata <= dat_i[15:8];
            else if (address[1:0] == 2'b10) axis_tx_tdata <= dat_i[23:16];
            else if (address[1:0] == 2'b11) axis_tx_tdata <= dat_i[31:24];
        end 
        else if (state == READDATA0 && axis_tx_tready) axis_tx_tdata <= dat_i[15:8];
        else if (state == READDATA1 && axis_tx_tready) axis_tx_tdata <= dat_i[23:16];
        else if (state == READDATA2 && axis_tx_tready) axis_tx_tdata <= dat_i[31:24];
        else if (state == WRITEADDR0 && axis_tx_tready) axis_tx_tdata <= len;

        // tlast generation
        if (state == WRITEADDR0 && axis_tx_tready) axis_tx_tlast <= 1;
        else if (((state == READCAPTURE && ack_i) || 
                 ((state == READDATA0 || state == READDATA1 || state == READDATA2) && axis_tx_tready)) && 
                 (len == 1)) axis_tx_tlast <= 1;
        else axis_tx_tlast <= 0;
    end

    uart_rx6 rx(.clk(clk),.en_16_x_baud(en_16x_baud),.buffer_read(cobs_rx_tready && cobs_rx_tvalid),
                .buffer_reset(rst),
                .buffer_data_present(cobs_rx_tvalid),.data_out(cobs_rx_tdata),
                .serial_in(BM_RX));

    uart_tx6 tx(.clk(clk),.en_16_x_baud(en_16x_baud),.buffer_write(cobs_tx_tready && cobs_tx_tvalid),
                .buffer_reset(rst),
                .buffer_full(cobs_tx_full),.data_in(cobs_tx_tdata),
                .serial_out(BM_TX));

    axis_cobs_decode u_decoder(.clk(clk),.rst(rst),
                                .s_axis_tdata(cobs_rx_tdata),
                                .s_axis_tvalid(cobs_rx_tvalid),
                                .s_axis_tready(cobs_rx_tready),
                                .s_axis_tuser(1'b0),
                                .s_axis_tlast(cobs_rx_tlast),
                                .m_axis_tdata(axis_rx_tdata),
                                .m_axis_tuser(axis_rx_tuser),
                                .m_axis_tvalid(axis_rx_tvalid),
                                .m_axis_tready(axis_rx_tready),
                                .m_axis_tlast(axis_rx_tlast));
    axis_cobs_encode u_encoder( .clk(clk), .rst(rst),
                                .s_axis_tdata(axis_tx_tdata),
                                .s_axis_tvalid(axis_tx_tvalid),
                                .s_axis_tready(axis_tx_tready),
                                .s_axis_tuser(1'b0),
                                .s_axis_tlast(axis_tx_tlast),
                                .m_axis_tdata(cobs_tx_tdata),
                                .m_axis_tready(cobs_tx_tready),
                                .m_axis_tvalid(cobs_tx_tvalid));
                                                                                                                                
    cobs_ila u_ila(.clk(clk),.probe0(axis_rx_tdata),
                             .probe1(axis_rx_tuser),
                             .probe2(axis_rx_tvalid),
                             .probe3(axis_rx_tlast),
                             .probe4(axis_rx_tready),
                             .probe5(state),
                             .probe6(adr_o),
                             .probe7(en_o),
                             .probe8(wr_o),
                             .probe9(wstrb_o),
                             .probe10(dat_o),
                             .probe11(dat_i),
                             .probe12(axis_tx_tdata),
                             .probe13(axis_tx_tvalid),
                             .probe14(axis_tx_tready),
                             .probe15(axis_tx_tlast),
                             .probe16(cobs_tx_tdata),
                             .probe17(cobs_tx_tready),
                             .probe18(cobs_tx_tvalid));

    // tready generation. Does NOT happen in IDLE.
    // The state names describe what's currently on the TDATA busses.
    assign axis_rx_tready = (state == ADDR2 || state == ADDR1 || 
                             state == ADDR0 || state == READLEN ||
                             state == WRITE0 || state == WRITE1 || state == WRITE2 ||
                             state == WRITE3);

    // tvalid generation
    assign axis_tx_tvalid = (state == READDATA0 || state == READDATA1 || state == READDATA2 ||
                             state == READDATA3 || state == READADDR0 || state == READADDR1 ||
                             state == READADDR2 || state == WRITEADDR0 || state == WRITEADDR1 ||
                             state == WRITEADDR2 || state == WRITELEN);

    assign adr_o = address[21:2];
    assign dat_o = data;
    assign en_o = (state == WRITEEN) || (state == READCAPTURE);
    assign wr_o = (state == WRITEEN);
    assign wstrb_o = wstrb;
    
endmodule
