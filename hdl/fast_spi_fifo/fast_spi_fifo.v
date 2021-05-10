`timescale 1ns / 1ps
`include "radiant_debug.vh"
// Fast SPI FIFO. This works as a pure data output FIFO, and can operate at ~200 MHz,
// responding to SPI speeds up to 50 MHz without actually using the SPI to clock
// data. Not super-positive about the reliability of doing it this way.
// We've essentially got probably 7 ns setup or so. Maaaybe it's good enough?
//
// We avoid actually clocking the output data using SCLK because it's not a free-running
// clock, which gives me headaches. That being said, we **might** be able to just do
// this with CPHA=1, where a rising edge on SCLK actually would clock MISO, which would
// then be captured by the falling edge.
//
// Timing-wise that's still iffy because we still need to *detect* the SCLK edge
// to do something internally, which might not be fast enough at 200 MHz.
// we'll see...
//
// REALLY need to rethink this thing to *try* to do it off of SCLK.
// Maybe just run a counter off of SCLK directly and have that mux the
// data.
module fast_spi_fifo(
        input       aclk,
        input       aresetn,
        input [7:0] s_axis_tdata,
        input       s_axis_tvalid,
        // tready gets double-used here. Interrupt goes when
        // the FIFO has 254 bytes (at least) *and* tready has been
        // low for at least 2 clock cycles. This indicates that
        // 256 bytes can be freely read by SPI.
        output      s_axis_tready,        
        
        // RX path. No tready.
        input        enable_rx,
        output [7:0] m_axis_tdata,
        output       m_axis_tvalid,
        
        // for debugging.
        output [31:0] spi_clocks_seen,
        output        spi_clocks_seen_valid,
                
        input CS_B,
        input SCLK,
        input MOSI,
        output MISO
    );
    
    localparam DEBUG = `FAST_SPI_FIFO_DEBUG;
    
    // These just use the same sequencing the tx path uses.
    // when load_sequence[7] falls, we transfer over the shift reg.
    reg [7:0] rx_shift_reg = {8{1'b0}};
    reg [7:0] rx_capture_reg = {8{1'b0}};
    reg       rx_load = 0;
    reg       rx_valid = 0;
    
    reg [7:0] data_in = {8{1'b0}};
    reg       di_full = 0;
    
    reg [7:0] data_next = {8{1'b0}};
    reg       dn_full = 0;
    
    reg       clear_data = 0;
    
    reg [6:0] shift_reg = {7{1'b0}};
    
    (* IOB = "TRUE" *)
    reg       sclk_iob = 0;
    reg       sclk_reg = 0;
    
    reg       sclk_rise = 0;

    // tell the debug counter to count    
    reg       sclk_count = 0;
    // periodically capture counter
    wire      sclk_count_capture;
    // and indicate that it's valid.
    reg       sclk_count_valid = 0;
    
    (* IOB = "TRUE" *)
    reg       cs_iob = 0;
    reg       cs_reg = 0;
    
    reg       cs_fall = 0;
    reg       miso_load = 0;
    
    (* IOB = "TRUE" *)
    reg       mosi_iob = 0;
    
    (* IOB = "TRUE" *)
    reg       miso_iob = 0;
    
    reg [7:0] load_sequence = 8'h1;
    reg [7:0] load_sequence_reg = 8'h1;
    reg       last_load_sequence = 0;

    generate
        if (DEBUG == "TRUE") begin : DBG
            // ILAs are tough here, we don't want to overload stuff.
            // So we reregister everything
            reg sclk_dbg = 0;
            reg cs_dbg = 0;
            reg [7:0] load_sequence_dbg = {8{1'b0}};
            reg di_full_dbg = 0;
            reg dn_full_dbg = 0;
            reg [6:0] shift_reg_dbg = {7{1'b0}};    
            reg [7:0] data_in_dbg = {8{1'b0}};
            reg [7:0] data_next_dbg = {8{1'b0}};
            reg miso_iob_copy = 0;
            reg miso_iob_debug = 0;
            always @(posedge aclk) begin : DBGREG
                sclk_dbg <= sclk_iob;
                cs_dbg <= cs_iob;
                load_sequence_dbg <= load_sequence;
                di_full_dbg <= di_full;
                dn_full_dbg <= dn_full;
                shift_reg_dbg <= shift_reg;
                data_in_dbg <= data_in;
                data_next_dbg <= data_next;
                if ((sclk_iob && !sclk_reg) || miso_load)
                    miso_iob_copy <= shift_reg[6];
                miso_iob_debug <= miso_iob_copy;
            end
            
            fast_spi_ila_debug u_ila(.clk(aclk),
                                    .probe0(sclk_dbg),
                                    .probe1(cs_dbg),
                                    .probe2(load_sequence_dbg),
                                    .probe3(di_full_dbg),
                                    .probe4(dn_full_dbg),
                                    .probe5(shift_reg_dbg),
                                    .probe6(data_in_dbg),
                                    .probe7(data_next_dbg),
                                    .probe8(mosi_iob),
                                    .probe9(rx_capture_reg),
                                    .probe10(rx_valid),
                                    .probe11(miso_iob_copy));
        end
    endgenerate
    
    always @(posedge aclk) begin

        // these always operate
        sclk_iob <= SCLK;
        cs_iob <= CS_B;
        mosi_iob <= MOSI;
        
        sclk_reg <= sclk_iob;
        cs_reg <= cs_iob;
        cs_fall <= !cs_iob && cs_reg;
        miso_load <= cs_fall;
        
        // this is *only* used for RX path, which is non-critical
        sclk_rise <= (sclk_iob && !sclk_reg);
        
        // decouple counter timing
        sclk_count <= sclk_rise;
        // valid after capture
        sclk_count_valid <= sclk_count_capture;
        
        if ((sclk_iob && !sclk_reg) || miso_load)
            miso_iob <= shift_reg[6];
            
        // We load at "cs_fall", and then we also shift over one at each clock AND at miso_load.
        // The miso_load grab is because when that happens, MISO already has the MSB. So we
        // need to shift up.
        if ((sclk_iob && !sclk_reg) || miso_load)
            shift_reg <= {shift_reg[6:0], data_in[0]};
        else if (cs_fall || load_sequence[7])
            shift_reg <= data_in[7:1];
        
        if (cs_iob) load_sequence <= 8'h1;
        else if (sclk_iob && !sclk_reg) load_sequence <= {load_sequence[6:0],load_sequence[7]};

        if (cs_iob) load_sequence_reg <= 8'h1;
        else load_sequence_reg <= load_sequence;
        
        last_load_sequence <= load_sequence_reg[7];
        
        rx_load <= (!load_sequence_reg[7] && last_load_sequence && enable_rx);
        rx_valid <= rx_load;
        
        if (rx_load) rx_capture_reg <= rx_shift_reg;
        if (sclk_rise) rx_shift_reg <= {rx_shift_reg[6:0],mosi_iob};                
        
        clear_data <= (load_sequence[1] && !load_sequence_reg[1]);
        
        if (!aresetn || clear_data) di_full <= 0;
        else if (dn_full && !di_full) di_full <= 1;
        
        if (!aresetn || (!di_full && dn_full)) dn_full <= 0;
        else if (s_axis_tvalid) dn_full <= 1;
        
        // always capture: di_full covers whether the data's valid or not
        if (dn_full && !di_full) data_in <= data_next;
        
        // always capture: dn_full covers whether the data's valid or not
        if (s_axis_tvalid && s_axis_tready) data_next <= s_axis_tdata;
    end
    wire [47:0] spi_debug_counter_cascade;
    spi_debug_counter u_counter(.CLK(aclk),.CARRYIN(1'b1),.CEP(sclk_count),.PCOUT(spi_debug_counter_cascade));
    spi_debug_capture u_capture(.CLK(aclk),.PCIN(spi_debug_counter_cascade),.CEP(sclk_count_capture),
                                .P(spi_clocks_seen) );
    // this generates a divide by 64, which is way slow enough for a clock domain cross to 50 MHz.
    clk_div_ce #(.CLK_DIVIDE(31),.EXTRA_DIV2("TRUE"))
        u_capture_clock(.clk(aclk),.ce(sclk_count_capture));
    
    assign m_axis_tdata = rx_capture_reg;
    assign m_axis_tvalid = rx_valid;
    
    assign MISO = miso_iob;
    assign s_axis_tready = !dn_full;
    assign spi_clocks_seen_valid = sclk_count_valid;
endmodule
