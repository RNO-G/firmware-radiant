`timescale 1ns / 1ps
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
        
        input CS_B,
        input SCLK,
        input MOSI,
        output MISO
    );
    
    reg [7:0] data_in = {8{1'b0}};
    reg       di_full = 0;
    
    reg [7:0] data_next = {8{1'b0}};
    reg       dn_full = 0;
    
    reg       clear_data = 0;
    
    reg [6:0] shift_reg = {7{1'b0}};
    
    (* IOBUF = "TRUE" *)
    reg       sclk_iob = 0;
    reg       sclk_reg = 0;
    
    (* IOBUF = "TRUE" *)
    reg       cs_iob = 0;
    reg       cs_reg = 0;
    
    reg       cs_fall = 0;
    reg       miso_load = 0;
    
    (* IOBUF = "TRUE" *)
    reg       mosi_iob = 0;
    
    (* IOBUF = "TRUE" *)
    reg       miso_iob = 0;
    
    reg [7:0] load_sequence = 8'h1;
    reg [7:0] load_sequence_reg = 8'h1;

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
    fast_spi_ila_debug u_ila(.clk(aclk),
                            .probe0(sclk_dbg),
                            .probe1(cs_dbg),
                            .probe2(load_sequence_dbg),
                            .probe3(di_full_dbg),
                            .probe4(dn_full_dbg),
                            .probe5(shift_reg_dbg),
                            .probe6(data_in_dbg),
                            .probe7(data_next_dbg));
    always @(posedge aclk) begin
        sclk_dbg <= sclk_iob;
        cs_dbg <= cs_iob;
        load_sequence_dbg <= load_sequence;
        di_full_dbg <= di_full;
        dn_full_dbg <= dn_full;
        shift_reg_dbg <= shift_reg;
        data_in_dbg <= data_in;
        data_next_dbg <= data_next;

        // these always operate
        sclk_iob <= SCLK;
        cs_iob <= CS_B;
        mosi_iob <= MOSI;
        
        sclk_reg <= sclk_iob;
        cs_reg <= cs_iob;
        cs_fall <= !cs_iob && cs_reg;
        miso_load <= cs_fall;
        
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
    
    
    assign MISO = miso_iob;
    assign s_axis_tready = !dn_full;
endmodule
