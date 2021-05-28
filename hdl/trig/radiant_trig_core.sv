`timescale 1ns / 1ps
`include "wishbone.vh"
// genericified-up
// This is probably going to be pretty generic for any design with up to 32 channels. Fun.
// Note:
// *All* changes can only be done with the master enable off. You disable everything,
// configure stuff, reenable, then it's good.
module radiant_trig_core #(parameter NUM_TRIG=1, parameter NUM_CH=24, parameter ONESHOT_WIDTH=20, parameter THRESH_WIDTH=5)(
        input clk_i,
        input rst_i,
        `WBS_NAMED_PORT(wb, 32, 9, 4),
        input trig_clk_i,
        input pwm_clk_i,
        // pulse generator output
        output pulse_o,
        // these are sync to trigclk
        output [NUM_CH-1:0] trigin_en_o,
        // these are treated as async
        output [NUM_TRIG*NUM_CH-1:0] trig_maskb_o,
        output [NUM_TRIG-1:0] trig_en_o,
        output [NUM_TRIG*ONESHOT_WIDTH-1:0] trig_window_o,
        output [NUM_TRIG*THRESH_WIDTH-1:0] trig_thresh_o
    );

    // how many bits do we need to store the registers
    localparam NUM_TRIG_ADDR_BITS = $clog2(NUM_TRIG);
    localparam NUM_TRIG_EXPANDED = (1<<NUM_TRIG_ADDR_BITS);
    // address 0x00: [0] master enable of all internal triggers
    // address 0x04: [23:0] trigger input enables. if this is not 1, a channel's input will be permanently 0
    //                      and its IBUF disabled    
    // address 0x08: [31] output pulse enable [30] sharp output pulse [29:0] output pulse repeat period (in 5 ns intervals), don't do 0.
    //               "sharp" output pulse here shrinks the output pulse's width to 2.5 ns
    // address 0x08-0x2F: reserved
    // address 0x100: [31] trigger 0 enable
    // address 0x104: [23:0] trigger 0 mask-bar (set to 1 which channels are *included*)
    // address 0x108: [19:0] trigger 0 coincidence window (add [4:0], [9:5], [14:10], [19:15] and add 7 for total window in 2.5 ns intervals)
    // address 0x10C: [4:0] threshold 
    // address 0x110-11C: as above for trigger 1
    // address 0x120-12C: as above for trigger 2
    // address 0x130-13C: as above for trigger 3
    reg master_en = 0;
    (* ASYNC_REG = "TRUE" *)
    reg [2:0] master_en_trigclk = {3{1'b0}};    
    reg [NUM_TRIG-1:0] trigout_en_trigclk = {NUM_TRIG{1'b0}};

    reg [NUM_TRIG-1:0] trigout_en_reg = {NUM_TRIG{1'b0}};
    reg [NUM_CH-1:0] trigin_en = {24{1'b0}};
    reg [NUM_TRIG*NUM_CH-1:0] trig_maskb = {NUM_TRIG*NUM_CH{1'b0}};
    reg [NUM_TRIG*ONESHOT_WIDTH-1:0] trig_window = {NUM_TRIG*ONESHOT_WIDTH{1'b0}};    
    reg [NUM_TRIG*THRESH_WIDTH-1:0] trig_thresh = {NUM_TRIG*THRESH_WIDTH{1'b0}};
    reg ack = 0;
    reg pulse_sharp = 0;
    reg pulse_dis = 0;
    (* ASYNC_REG = "TRUE" *)
    reg [1:0] pulse_dis_pwmclk = {2{1'b0}};
    (* ASYNC_REG = "TRUE" *)
    reg [1:0] pulse_sharp_pwmclk = {2{1'b0}};
    reg [1:0] pulse_d_pwmclk = {2{1'b0}};    
    reg [29:0] pulse_rate = {30{1'b0}};
    reg pulse_update = 0;
    reg pulse_update_reg = 0;
    wire pulse_update_pwmclk;
    // only 3 control regs for now
    wire [31:0] control_regs[3:0];
    assign control_regs[0] = { {31{1'b0}}, master_en };
    assign control_regs[1] = { {(32-NUM_TRIG){1'b0}}, trigin_en };
    assign control_regs[2] = { pulse_dis, pulse_sharp, pulse_rate };
    assign control_regs[3] = control_regs[1];
    // expanded to necessary power of 2    
    wire [31:0] trigger_regs_expanded[NUM_TRIG_EXPANDED-1:0][3:0];
    // simplify demuxing
    wire [31:0] trigger_regs_muxed;
    
    always @(posedge trig_clk_i) begin
        master_en_trigclk <= { master_en_trigclk[1:0], master_en };
        // what in the hell was I thinking
//        if (master_en_trigclk[1]) trigout_en_trigclk <= {NUM_TRIG{1'b0}};
//        else if (master_en_trigclk[2]) trigout_en_trigclk <= trigout_en_reg;

        // the trigclock capture happens at a RISING edge of master_en.        
        // So when master_en_trigclk[1] goes to 1, master_en_trigclk[2] will be 0
        // for 1 clock only, where we capture.
        if (!master_en_trigclk[1]) trigout_en_trigclk <= {NUM_TRIG{1'b0}};
        else if (!master_en_trigclk[2]) trigout_en_trigclk <= trigout_en_reg;
    end
    
    integer i;
    always @(posedge clk_i) begin    
        ack <= wb_cyc_i && wb_stb_i;
        if (wb_cyc_i && wb_stb_i && wb_we_i && wb_sel_i[0] && ({wb_adr_i[8:2],2'b00} == 9'h00))
            master_en <= wb_dat_i[0];
        if (wb_cyc_i && wb_stb_i && wb_we_i && ({wb_adr_i[8:2],2'b00} == 9'h04)) begin
            for (i=0;i<(NUM_CH/8);i=i+1) begin
                if (wb_sel_i[i] && !master_en) trigin_en[8*i +: 8] <= wb_dat_i[8*i +: 8];
            end
        end
        if (wb_cyc_i && wb_stb_i && wb_we_i && ({wb_adr_i[8:2],2'b00} == 9'h08)) begin
            pulse_update <= !ack;
            if (wb_sel_i[0]) pulse_rate[0 +: 8] <= wb_dat_i[0 +: 8];
            if (wb_sel_i[1]) pulse_rate[8 +: 8] <= wb_dat_i[8 +: 8];
            if (wb_sel_i[2]) pulse_rate[16 +: 8] <= wb_dat_i[16 +: 8];
            if (wb_sel_i[3]) pulse_rate[24 +: 6] <= wb_dat_i[24 +: 6];
            if (wb_sel_i[3]) pulse_sharp <= wb_dat_i[30];
            if (wb_sel_i[3]) pulse_dis <= wb_dat_i[31];
        end else begin
            pulse_update <= 1'b0;
        end
        pulse_update_reg <= pulse_update;
    end
    
    /// pulse generator stuff
    wire pulse_now;
    always @(posedge pwm_clk_i) begin
        pulse_sharp_pwmclk <= { pulse_sharp_pwmclk[0], pulse_sharp };
        pulse_dis_pwmclk <= { pulse_dis_pwmclk[0], pulse_dis };
        
        pulse_d_pwmclk <= { pulse_now, pulse_now && !pulse_sharp_pwmclk[1] }; 
    end
    
    flag_sync u_update_sync(.in_clkA(pulse_update_reg),.out_clkB(pulse_update_pwmclk),.clkA(clk_i),.clkB(pwm_clk_i));
    dsp_counter_terminal_count #(.FIXED_TCOUNT("FALSE"),.RESET_TCOUNT_AT_RESET("FALSE"))
            u_pulse_counter( .clk_i(pwm_clk_i),
                             .rst_i(1'b0),
                             .tcount_i( pulse_rate ),
                             .update_tcount_i( pulse_update_pwmclk ),
                             .tcount_reached_o(pulse_now));
                             
    ODDR #(.DDR_CLK_EDGE("SAME_EDGE")) 
        u_oddrpulse(.D1(pulse_d_pwmclk[1]),.D2(pulse_d_pwmclk[0]),.CE(1'b1),.C(pwm_clk_i),.S(1'b0),.R(1'b0),.Q(pulse_o));    

    // end pulse stuff
    
    // trigger stuff
    generate
        genvar t,c,w,th;
        for (t=0;t<NUM_TRIG_EXPANDED;t=t+1) begin : TL
           if (t < NUM_TRIG) begin : ACT
                wire this_trig_write = wb_cyc_i && wb_stb_i && wb_we_i && wb_adr_i[8] && (wb_adr_i[7:4] == t);
                always @(posedge clk_i) begin : ENLOG
                    if (this_trig_write && { wb_adr_i[3:2],2'b00 } == 4'h0 && wb_sel_i[3] && !master_en)
                        trigout_en_reg[t] <= wb_dat_i[31];
                end
                for (c=0;c<NUM_CH;c=c+1) begin : CL
                    always @(posedge clk_i) begin : MASKLOG
                        if (this_trig_write && { wb_adr_i[3:2],2'b00 } == 4'h4 && wb_sel_i[c/8] && !master_en)
                            trig_maskb[NUM_CH*t + c] <= wb_dat_i[c];
                    end
                end
                for (w=0;w<ONESHOT_WIDTH;w=w+1) begin : WL
                    always @(posedge clk_i) begin : WINDOWLOG                
                        if (this_trig_write && { wb_adr_i[3:2],2'b00} == 4'h8 && wb_sel_i[w/8] && !master_en)
                            trig_window[ONESHOT_WIDTH*t + w] <= wb_dat_i[w];
                    end
                end
                for (th=0;th<THRESH_WIDTH;th=th+1) begin : THL
                    always @(posedge clk_i) begin : THLOG
                        if (this_trig_write && { wb_adr_i[3:2],2'b00} == 4'hC && wb_sel_i[th/8] && !master_en)
                            trig_thresh[THRESH_WIDTH*t + th] <= wb_dat_i[th];
                    end
                end
                assign trigger_regs_expanded[t][0] = { trigout_en_reg[t], {31{1'b0}} };
                assign trigger_regs_expanded[t][1] = { {(32-NUM_CH){1'b0}}, trig_maskb[NUM_CH*t +: NUM_CH] };
                assign trigger_regs_expanded[t][2] = { {(32-ONESHOT_WIDTH){1'b0}}, trig_window[ONESHOT_WIDTH*t +: ONESHOT_WIDTH] };
                assign trigger_regs_expanded[t][3] = { {(32-THRESH_WIDTH){1'b0}}, trig_thresh[THRESH_WIDTH*t +: THRESH_WIDTH] };
            end else begin : DUM
                // these are shadow copies: like if NUM_TRIG is 3,
                // we shadow 1 up to 3. We do this by moduloing the value with NUM_TRIG_EXPANDED>>1
                // This can ONLY happen if NUM_TRIG_EXPANDED = 4 or more.
                // NUM_TRIG_EXPANDED=1 *only* happens with NUM_TRIG = 1
                // NUM_TRIG_EXPANDED=2 *only* happens with NUM_TRIG = 2
                // so for NUM_TRIG=3, this does t % 2 or 1.
                assign trigger_regs_expanded[t][0] = trigger_regs_expanded[t % (NUM_TRIG_EXPANDED>>1)][0];
                assign trigger_regs_expanded[t][1] = trigger_regs_expanded[t % (NUM_TRIG_EXPANDED>>1)][1];
                assign trigger_regs_expanded[t][2] = trigger_regs_expanded[t % (NUM_TRIG_EXPANDED>>1)][2];
                assign trigger_regs_expanded[t][3] = trigger_regs_expanded[t % (NUM_TRIG_EXPANDED>>1)][3];         
            end
        end
    endgenerate
    
    radtrig_core_debug u_ila(.clk(clk_i),
                            .probe0(wb_cyc_i),
                            .probe1(wb_stb_i),
                            .probe2(wb_we_i),
                            .probe3(wb_sel_i),
                            .probe4(wb_adr_i),
                            .probe5(wb_dat_i),
                            .probe6(wb_dat_o),
                            .probe7(wb_ack_o));
                    

    // trigger_regs_muxed only needs to look at addresses if we have more than 1
    // (see why this is needed - otherwise the part select is 4 +: 0)
    assign trigger_regs_muxed = (NUM_TRIG>1) ? trigger_regs_expanded[wb_adr_i[4 +: NUM_TRIG_ADDR_BITS]][wb_adr_i[3:2]] : trigger_regs_expanded[0][wb_adr_i[3:2]];    
    assign wb_dat_o = wb_adr_i[8] ? trigger_regs_muxed : control_regs[wb_adr_i[3:2]];
    assign wb_ack_o = ack;    
    assign wb_err_o = 1'b0;
    assign wb_rty_o = 1'b0;
    
    assign trig_en_o = trigout_en_trigclk;
    assign trigin_en_o = trigin_en;
    assign trig_window_o = trig_window;
    assign trig_maskb_o = trig_maskb;    
    assign trig_thresh_o = trig_thresh;
endmodule
