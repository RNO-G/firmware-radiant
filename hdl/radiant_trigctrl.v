`timescale 1ns / 1ps
// Trigger control core.
// contains: PPS counter, clock counter, event generation, sync enable, etc.
//
// I'll add triggering stuff soon here as well (... hopefully).
//
// Address Space: Total is 0x0000 - 0x01FF. We split that up into
// the event FIFO (0x100-0x1FF) and the control registers (0x000-0x0FF).
// This corresponds to 64 total control registers here, which dear lord I hope
// is enough for here.
`include "wishbone.vh"
module radiant_trigctrl(
        input clk_i,
        input rst_i,
        `WBS_NAMED_PORT(wb, 32, 9, 4),
        
        input sys_clk_i,
        
        input event_i,
        input [31:0] event_info_i,
        input event_done_i,
        output event_ready_o,
        
        input pps_i        
    );
        
    
    localparam [31:0] EVENT_IDENTIFIER = "RDE0";
    localparam NUM_EVENT_DWORDS = 6;
    localparam NUM_EVENT_DYNAMIC_DWORDS = NUM_EVENT_DWORDS-1;
    // capture the PPS input
    reg [2:0] pps_reg = {3{1'b0}};
    // rising edge flag it
    reg       pps_flag = 0;

    // sync if enabled
    reg       sync = 0;

    // sync enable
    reg [1:0] sync_enable = {2{1'b0}};
    
    (* USE_DSP48 = "TRUE" *)
    reg [47:0] cur_sec_count = {48{1'b0}};
    (* USE_DSP48 = "TRUE" *)
    reg [47:0] cur_event_count = {48{1'b0}};
    (* USE_DSP48 = "TRUE" *)
    reg [47:0] cur_sysclk_count = {48{1'b0}};
    
    reg last_high_sec = 0;
    reg last_high_evcount = 0;
    reg last_high_syscount = 0;    
    wire roll_sec = (cur_sysclk_count[32] ^ last_high_sec);
    wire roll_evcount = (cur_event_count[32] ^ last_high_evcount);
    wire roll_syscount = (cur_sysclk_count[32] ^ last_high_syscount);
    
    // captured at event_i
    wire [31:0] status_flags = { {29{1'b0}}, roll_syscount, roll_sec, roll_evcount };

    // Initial reset of the FIFOs. Works in sys_clk domain because that's likely to startup second.
    reg fifo_reset = 1;
    wire fifo_reset_done;
    SRL16E u_startup_delay(.D(fifo_reset),.CE(1'b1),.CLK(sys_clk_i),.A0(1'b1),.A1(1'b1),.A2(1'b1),.A3(1'b1),.Q(fifo_reset_done));

    reg fifo_reset_clk = 0;
    wire fifo_reset_sysclk;    
    flag_sync u_reset_sync(.in_clkA(fifo_reset_clk),.clkA(clk_i),.out_clkB(fifo_reset_sysclk),.clkB(sys_clk_i));
    
    // clk-side registers
    wire [31:0] event_dwords_read[NUM_EVENT_DWORDS-1:0];
    // read enable
    wire [NUM_EVENT_DWORDS-1:0] event_dword_rden;

    // constant
    assign event_dwords_read[0] = EVENT_IDENTIFIER;
    // the others are assigned through a FIFO below, but here are the logical definitions
//    wire [31:0] event_second  = event_dwords_read[1];
//    wire [31:0] event_count = event_dwords_read[2];
//    wire [31:0] event_sysclk = event_dwords_read[3];
//    wire [31:0] event_info = event_dwords_read[4];
//    wire [31:0] event_status = event_dwords_read[5];
    
    // sysclk-side inputs. Only dynamic dwords. These are shifted up by 1 b/c of the identifier at 0.
    wire [31:0] event_dwords_write[NUM_EVENT_DYNAMIC_DWORDS-1:0];
    assign event_dwords_write[0] = cur_sec_count[31:0];
    assign event_dwords_write[1] = cur_event_count[31:0];
    assign event_dwords_write[2] = cur_sysclk_count[31:0];
    assign event_dwords_write[3] = event_info_i;
    assign event_dwords_write[4] = status_flags;

            
    always @(posedge sys_clk_i) begin
        if (sync) cur_sec_count <= {48{1'b0}};
        else if (pps_flag) cur_sec_count <= cur_sec_count + 1;
    
        if (sync) cur_event_count <= {48{1'b0}};
        else if (event_i) cur_event_count <= cur_event_count + 1;
        
        if (sync) cur_sysclk_count <= {48{1'b0}};
        else cur_sysclk_count <= cur_sysclk_count + 1;
        
        if (sync) last_high_sec <= 0;
        else if (event_i) last_high_sec <= cur_sec_count[32];
        
        if (sync) last_high_evcount <= 0;
        else if (event_i) last_high_evcount <= cur_event_count[32];
        
        if (sync) last_high_syscount <= 0;
        else if (event_i) last_high_syscount <= cur_sysclk_count[32];
        
        // add something here to turn this into a generic FIFO reset as well
        if (fifo_reset_sysclk) fifo_reset <= 1;
        else if (fifo_reset_done) fifo_reset <= 0;            
    end        
    
    // Event headers consist of 6 32-bit values:
    // 0: Static identifying value (RDE0 = 0x52444530)
    // 1: Event number (since reset, 32-bit value)
    // 2: PPS count (since reset, 32-bit value)
    // 3: Event system clock count (since reset, 32-bit value)
    // 4: Event info (to be determined)
    // 5: Status flags. Bits 2/1/0 are event number/count/PPS rollover indicators.
    //    This is a bit pointless since you should *know* when that happens, but, hey.
    //    Makes most sense for count, but even then not really, since rolls only happen
    //    every 43 seconds. I might need to do something to indicate when the data's
    //    garbage (due to being triggered when dead), but I'll need to improve the DMA
    //    engine for that anyway, probably by adding dynamic request types to the engine.
    
    // They're all FIFOs (well, except the identifier) in successive registers, so you just read x 6.
    // These are dist. ram FIFOs, 16 deep. I don't *think* I need them any bigger than that, but we'll see.
    // I'm ignoring the full for now.
    generate
        genvar i;
        for (i=0;i<NUM_EVENT_DYNAMIC_DWORDS;i=i+1) begin : DYDW
            event_hdr_fifo u_event_sec_fifo(.rst(fifo_reset),.wr_clk(sys_clk_i),.rd_clk(clk_i),
                                            .din(event_dwords_write[i]),
                                            .wr_en(event_i),
                                            // shift up because event_dwords_read[0] is the identifier and doesn't
                                            // need a FIFO.
                                            .dout(event_dwords_read[i+1]),
                                            .rd_en(event_dword_rden[i+1]));
        end
    endgenerate                                
                                        
                    
    
endmodule
