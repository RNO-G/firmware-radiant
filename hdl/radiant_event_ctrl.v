`timescale 1ns / 1ps
// Event control core.
// contains: PPS counter, clock counter, event generation, sync enable, etc.
// Probably should just be called "timing control core" or something. WHATEVER.
//
// Address Space: Total is 0x0000 - 0x01FF. We split that up into
// the event FIFO (0x100-0x1FF) and the control registers (0x000-0x0FF).
// This corresponds to 64 total control registers here, which dear lord I hope
// is enough for here.
//
// 0x0000 : event control. bit[0] = reset all FIFOs, bit[1] = enable sync on next PPS
// 0x0004 : current pps count. Poll on this to change, then set bit 1 to sync everyone on next PPS.
// 0x0008 : sysclk count at last PPS
// 0x000C : sysclk count at lastlast PPS
// 0x0100 : current event identifier (constant, always RDE0)
// 0x0104 : current event second
// 0x0108 : current event count
// 0x010C : current event sysclk count
// 0x0110 : current event info
// 0x0114 : current event status flags
// 0x0118 : current event last sysclk count
// 0x011C : current event lastlast sysclk count
`include "wishbone.vh"
`include "radiant_debug.vh"
module radiant_event_ctrl(
        input clk_i,
        input rst_i,
        `WBS_NAMED_PORT(wb, 32, 9, 4),
        
        input sys_clk_i,
        output event_fifo_reset_o,
        input event_fifo_empty_i,
        // types are unused for now, they might be used to indicate dead events
        input event_i,
        input event_type_i,
        input [31:0] event_info_i,
        input event_done_i,

        // These are all CLK_I registers!
        // type is unused for now
        output event_ready_o,
        output event_ready_type_o,
        input event_readout_ready_i,
        
        input pps_i,
        output sync_o
    );

    localparam DEBUG = `EVENTCTRL_DEBUG;
        
    // just 4 control regs for now
    wire [31:0] control_regs[3:0];
    // and 8 event regs
    wire [31:0] event_regs[7:0];
    
    localparam [31:0] EVENT_IDENTIFIER = "RDE0";
    localparam NUM_EVENT_DWORDS = 8;
    localparam NUM_EVENT_DYNAMIC_DWORDS = NUM_EVENT_DWORDS-1;

    wire      pps_flag = pps_i;
    wire      pps_flag_clk;
    // delay on the pps_flag_clk
    reg       do_pps = 0;
    flag_sync u_pps_sync(.in_clkA(pps_i),.clkA(sys_clk_i),.out_clkB(pps_flag_clk),.clkB(clk_i));

    // sync if enabled
    reg       sync = 0;

    // sync enable sysclk
    reg [1:0] sync_enable_sysclk = {2{1'b0}};
    // sync enable clk
    reg       sync_enable_clk = 0;

        
    (* USE_DSP48 = "TRUE" *)
    reg [47:0] cur_sec_count = {48{1'b0}};
    // might try to use a partnered DSP here, who knows
    reg [31:0] cur_sec_count_clk = {32{1'b0}};

    (* USE_DSP48 = "TRUE" *)
    reg [47:0] cur_event_count = {48{1'b0}};
    (* USE_DSP48 = "TRUE" *)
    reg [47:0] cur_sysclk_count = {48{1'b0}};    

    // I don't like the psycho number of registers here. Might
    // try to bury these in DSPs or something.
    reg [31:0] last_sysclk_count = {32{1'b0}};
    reg [31:0] lastlast_sysclk_count = {32{1'b0}};
        
    reg [31:0] last_sysclk_count_clk = {32{1'b0}};
    reg [31:0] lastlast_sysclk_count_clk = {32{1'b0}};        

    wire [5:0] event_pending_count;
    wire [5:0] event_pending_count_clk;

    wire all_fifos_empty;
    wire dmareq_fifo_empty;
        
    // 0x0000 : event control. bit[0] = reset all FIFOs, bit[1] = enable sync on next PPS. [6] pending empty, [7] FIFO empty. [21:16] = number of pending DMA events
    // 0x0004 : current pps count. Poll on this to change, then set bit 1 to sync everyone on next PPS.
    // 0x0008 : sysclk count at last PPS
    // 0x000C : sysclk count at lastlast PPS
    assign control_regs[0] = { {10{1'b0}}, event_pending_count_clk ,all_fifos_empty, dmareq_fifo_empty, {14{1'b0}} };
    assign control_regs[1] = cur_sec_count_clk;
    assign control_regs[2] = last_sysclk_count_clk;
    assign control_regs[3] = lastlast_sysclk_count_clk;                
        
    reg last_high_sec = 0;
    reg last_high_evcount = 0;
    reg last_high_syscount = 0;    
    wire roll_sec = (cur_sysclk_count[32] ^ last_high_sec);
    wire roll_evcount = (cur_event_count[32] ^ last_high_evcount);
    wire roll_syscount = (cur_sysclk_count[32] ^ last_high_syscount);
    
    // captured at event_i
    wire [31:0] status_flags = { {28{1'b0}}, event_type_i, roll_syscount, roll_sec, roll_evcount };

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
    assign event_regs[0] = event_dwords_read[0];
    assign event_regs[1] = event_dwords_read[1];
    assign event_regs[2] = event_dwords_read[2];
    assign event_regs[3] = event_dwords_read[3];
    assign event_regs[4] = event_dwords_read[4];
    assign event_regs[5] = event_dwords_read[5];
    assign event_regs[6] = event_dwords_read[6];
    assign event_regs[7] = event_dwords_read[7];

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
    assign event_dwords_write[5] = last_sysclk_count;
    assign event_dwords_write[6] = lastlast_sysclk_count;

    reg ack = 0;    

    wire type_fifo_empty;
    wire [NUM_EVENT_DYNAMIC_DWORDS-1:0] dydw_fifo_empty;    

    assign all_fifos_empty = (&dydw_fifo_empty) && event_fifo_empty_i;
    assign dmareq_fifo_empty = type_fifo_empty;

    wire [4:0] event_pending_count_sysclk;

    // Event type FIFO. This is what generates the DMA requests.
    // Types are unused for now, they may be helpful later.
    // This needs to be an ASYNCHRONOUS FIFO!!
    // event_ready_o is in the wb_clk domain!
    event_type_fifo u_type_fifo(.din(event_type_i),
                                .wr_en(event_done_i),
                                .wr_clk(sys_clk_i),
                                .rd_clk(clk_i),
                                .dout(event_ready_type_o),
                                .valid(event_ready_o),
                                .empty(type_fifo_empty),
                                .rd_en(event_readout_ready_i && event_ready_o),
                                .wr_data_count(event_pending_count_sysclk),
                                .rd_data_count(event_pending_count_clk),
                                .rst(fifo_reset));
                                    
    always @(posedge clk_i) begin
        if (pps_flag_clk) sync_enable_clk <= 1'b0;
        else if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[8:2] == 7'h00)) sync_enable_clk <= wb_dat_i[1];        
    
        fifo_reset_clk <= wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[8:2] == 7'h00) && ack && wb_dat_i[2];
    
        do_pps <= pps_flag_clk;
    
        // Capture the current second count for monitoring.
        // I ultra-hate this, it's just a bucket-freaking ton of registers.
        if (do_pps) begin
            cur_sec_count_clk <= cur_sec_count;
            last_sysclk_count_clk <= last_sysclk_count;
            lastlast_sysclk_count_clk <= lastlast_sysclk_count;
        end
        
        ack <= wb_cyc_i && wb_stb_i;
    end
    
    reg capture_event = 0;
            
    always @(posedge sys_clk_i) begin
        capture_event <= event_i;
    
        sync_enable_sysclk <= { sync_enable_sysclk[0], sync_enable_clk };
        
        sync <= pps_flag && sync_enable_sysclk;
    
        if (sync) last_sysclk_count <= {32{1'b0}};
        else if (pps_flag) last_sysclk_count <= cur_sysclk_count;
        
        if (sync) lastlast_sysclk_count <= {32{1'b0}};
        else if (pps_flag) lastlast_sysclk_count <= last_sysclk_count;
    
        if (sync) cur_sec_count <= {48{1'b0}};
        else if (pps_flag) cur_sec_count <= cur_sec_count + 1;
    
        if (sync) cur_event_count <= {48{1'b0}};
        else if (capture_event) cur_event_count <= cur_event_count + 1;
        
        if (sync) cur_sysclk_count <= {48{1'b0}};
        else cur_sysclk_count <= cur_sysclk_count + 1;
        
        if (sync) last_high_sec <= 0;
        else if (capture_event) last_high_sec <= cur_sec_count[32];
        
        if (sync) last_high_evcount <= 0;
        else if (capture_event) last_high_evcount <= cur_event_count[32];
        
        if (sync) last_high_syscount <= 0;
        else if (capture_event) last_high_syscount <= cur_sysclk_count[32];
        
        // add something here to turn this into a generic FIFO reset as well
        if (fifo_reset_sysclk) fifo_reset <= 1;
        else if (fifo_reset_done) fifo_reset <= 0;            
    end        
    
    // Event headers consist of 8 32-bit values:
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
    // 6: system clock count on last PPS
    // 7: system clock count on last-last PPS
    
    // They're all FIFOs (well, except the identifier) in successive registers, so you just read x 6.
    // These are dist. ram FIFOs, 16 deep. I don't *think* I need them any bigger than that, but we'll see.
    // I'm ignoring the full for now.

    generate
        genvar i, d;
        for (d=0;d<NUM_EVENT_DWORDS;d=d+1) begin : DEN
            assign event_dword_rden[d] = wb_cyc_i && wb_stb_i && !wb_we_i && wb_adr_i[8] && (wb_adr_i[2 +: 3] == d);
        end
        for (i=0;i<NUM_EVENT_DYNAMIC_DWORDS;i=i+1) begin : DYDW
            event_hdr_fifo u_event_sec_fifo(.rst(fifo_reset),.wr_clk(sys_clk_i),.rd_clk(clk_i),
                                            .din(event_dwords_write[i]),
                                            .wr_en(capture_event),
                                            // shift up because event_dwords_read[0] is the identifier and doesn't
                                            // need a FIFO.
                                            .empty(dydw_fifo_empty[i]),
                                            .dout(event_dwords_read[i+(NUM_EVENT_DWORDS-NUM_EVENT_DYNAMIC_DWORDS)]),
                                            .rd_en(event_dword_rden[i+(NUM_EVENT_DWORDS-NUM_EVENT_DYNAMIC_DWORDS)]));
        end
        if (DEBUG == "TRUE") begin : ILA
            event_ila u_ila(.clk(sys_clk_i),
                            .probe0(capture_event),
                            .probe1(event_done_i),
                            .probe2(event_pending_count_sysclk),
                            .probe3(sync),
                            .probe4(cur_sec_count),
                            .probe5(cur_event_count));
        end
    endgenerate                                
                                        
                    
    // ack
    assign wb_ack_o = ack && wb_cyc_i;
    assign wb_err_o = 1'b0;
    assign wb_rty_o = 1'b0;
    // dat output
    assign wb_dat_o = (wb_adr_i[8]) ? event_regs[wb_adr_i[2 +: 3]] : control_regs[wb_adr_i[2 +: 2]];    

    assign event_fifo_reset_o = fifo_reset;

    // sync
    assign sync_o = sync;

endmodule
