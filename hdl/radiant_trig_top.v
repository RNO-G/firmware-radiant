`timescale 1ns/1ps
`include "radiant_debug.vh"
`include "wishbone.vh"

module radiant_trig_top #(  parameter NUM_TRIG = 2,
                            parameter TRIG_POLARITY = {24{1'b0}},
                            parameter DEBUG = `RADIANT_TRIG_TOP_DEBUG                            
                         )
                       (    input clk_i,
                            input rst_i,
                            `WBS_NAMED_PORT(wb, 32, 16, 4),
                            input pwm_clk_i,
                            input sys_clk_i,
                            input trig_clk_i,
                            
                            // trigger event generation
                            input event_i,
                            input [31:0] event_info_i,

                            input readout_running_i,
                            input readout_done_i,
                            input [23:0] readout_full_i,
                            output [11:0] readout_full_thresh_o,

                            // reset event FIFOs
                            output event_fifo_reset_o,
                            // event FIFOs are empty
                            input event_fifo_empty_i,
                            
                            // Actual DMA trigger
                            output event_ready_o,
                            // Response from DMA that readout is beginning.
                            input event_readout_ready_i,
                            
                            
                            // PPS input. This is a flag in sysclk.
                            input pps_i,
                            output sync_o,
                            output pulse_o,
                            // Scaler outputs. Just singles for now.
                            // These are in clk50 domain.
                            output [23:0] scal_o,

                            output full_trig_o,

                            input TRIGIN_P,
                            input TRIGIN_N,
                            output TRIGOUT_P,
                            output TRIGOUT_N,
                                              
                            input [23:0] TRIG,
                            input [23:0] THRESH,
                            output [23:0] THRESH_PWM,
                            output [NUM_TRIG-1:0] trig_o );
    
    wire [23:0] raw_trig_in;
    wire [23:0] trig_in;
    
    // Event control space is 0x0000 - 0x1FF. 
    `WB_DEFINE(ctrl, 32, 9, 4);
    wire ctrl_is_selected = (wb_adr_i[10:9] == 2'b00);
    assign ctrl_cyc_o = wb_cyc_i;
    assign ctrl_stb_o = wb_stb_i && ctrl_is_selected;
    assign ctrl_we_o = wb_we_i;
    assign ctrl_adr_o = wb_adr_i[8:0];
    assign ctrl_dat_o = wb_dat_i;
    assign ctrl_sel_o = wb_sel_i;
    
    // PWM space is 0x0200-0x03FF nominally. Right now shadowed above.
    `WB_DEFINE(pwm, 32, 32, 4);
    wire pwm_is_selected = (wb_adr_i[10:9] == 2'b01);
    assign pwm_cyc_o = wb_cyc_i;
    assign pwm_stb_o = wb_stb_i && pwm_is_selected;
    assign pwm_we_o = wb_we_i;
    assign pwm_adr_o = { {23{1'b0}}, wb_adr_i[8:0] };
    assign pwm_dat_o = wb_dat_i;
    assign pwm_sel_o = wb_sel_i;    
    
    // OVERLORD space is 400-5FF. 
    `WB_DEFINE(overlord, 32, 32, 4);
    wire overlord_is_selected = (wb_adr_i[10:9] == 2'b10);
    assign overlord_cyc_o = wb_cyc_i;
    assign overlord_stb_o = wb_stb_i && overlord_is_selected;
    assign overlord_we_o = wb_we_i;
    assign overlord_adr_o = { {23{1'b0}}, wb_adr_i[8:0] };
    assign overlord_dat_o = wb_dat_i;
    assign overlord_sel_o = wb_sel_i;
    
    // and trigger control space is 600-7FF
    `WB_DEFINE(trig, 32, 32, 4);
    wire trig_is_selected = (wb_adr_i[10:9] == 2'b11);
    assign trig_cyc_o = wb_cyc_i;
    assign trig_stb_o = wb_stb_i && trig_is_selected;
    assign trig_we_o = wb_we_i;
    assign trig_adr_o = { {23{1'b0}}, wb_adr_i[8:0] };
    assign trig_dat_o = wb_dat_i;
    assign trig_sel_o = wb_sel_i;    
    
    wire [3:0] ack_vec = { trig_ack_i , overlord_ack_i, pwm_ack_i, ctrl_ack_i };
    wire [3:0] err_vec = { trig_err_i , overlord_err_i, pwm_err_i, ctrl_err_i };
    wire [3:0] rty_vec = { trig_rty_i , overlord_rty_i, pwm_rty_i, ctrl_rty_i };
    wire [31:0] dat_vec[3:0];
    assign dat_vec[3] = trig_dat_i;
    assign dat_vec[2] = overlord_dat_i;
    assign dat_vec[1] = pwm_dat_i;
    assign dat_vec[0] = ctrl_dat_i;

    assign wb_ack_o = ack_vec[ wb_adr_i[10:9] ];
    assign wb_err_o = err_vec[ wb_adr_i[10:9] ];
    assign wb_rty_o = rty_vec[ wb_adr_i[10:9] ];
    assign wb_dat_o = dat_vec[ wb_adr_i[10:9] ];    
            
    // Event control core.
    wire trig_done;
    // Contains PPS counter, sync, event counter, event generation, etc.
    radiant_event_ctrl u_evctrl(.clk_i(clk_i),
                                .rst_i(rst_i),
                                `WBS_CONNECT( ctrl, wb),
                                .sys_clk_i(sys_clk_i),
                                
                                .event_fifo_reset_o(event_fifo_reset_o),
                                .event_fifo_empty_i(event_fifo_empty_i),
                                    
                                .event_i(full_trig_o),
                                .event_type_i(1'b0),
                                .event_info_i(event_info_i),
                                .event_done_i(trig_done),
                                
                                .event_ready_o(event_ready_o),
                                .event_ready_type_o(),
                                .event_readout_ready_i(event_readout_ready_i),
                                .sync_o(sync_o),
                                .pps_i(pps_i));
                                
    
    // PWM core
    pwm_wrap u_wrap(.clk_i(clk_i),
                    .rst_i(rst_i),
                    `WBS_CONNECT( pwm , wb ),
                    .pwm_clk_i(pwm_clk_i),
                    .THRESH_PWM(THRESH_PWM));

    // Trigger core
    localparam NUM_CH = 24;
    localparam ONESHOT_WIDTH = 20;
    localparam THRESH_WIDTH = 5;    
    // Which trigger inputs are enabled
    wire [NUM_CH-1:0] trigin_en;
    // Which inputs are included in each trigger ("mask-bar")
    wire [NUM_TRIG*NUM_CH-1:0] trig_maskb;
    // Which triggers are enabled
    wire [NUM_TRIG-1:0] trig_en;
    // Window for each trigger
    wire [NUM_TRIG*ONESHOT_WIDTH-1:0] trig_window;
    // Threshold for each trigger
    wire [NUM_TRIG*THRESH_WIDTH-1:0] trig_thresh;

    radiant_trig_core #(.NUM_TRIG(NUM_TRIG),
                        .NUM_CH(NUM_CH),
                        .ONESHOT_WIDTH(ONESHOT_WIDTH),
                        .THRESH_WIDTH(THRESH_WIDTH))
            u_trig_core( .clk_i(clk_i),
                         .rst_i(rst_i),
                         `WBS_CONNECT( trig , wb ),
                         .trig_clk_i(trig_clk_i),
                         .pwm_clk_i(pwm_clk_i),
                         .pulse_o(pulse_o),
                         
                         .trigin_en_o(trigin_en),
                         .trig_maskb_o(trig_maskb),
                         .trig_en_o(trig_en),
                         .trig_window_o(trig_window),
                         .trig_thresh_o(trig_thresh));

    wire stuck_period;
    wire [23:0] trigger_inputs;
    wire [NUM_TRIG-1:0] trigger_flag;
    wire [NUM_TRIG-1:0] trigger_flag_clk;
    // I dunno, check every 160 ns I guess.
    clk_div_ce #(.CLK_DIVIDE(8)) u_stuckgen(.clk(clk_i),.ce(stuck_period));
    generate
        genvar i, t;
        for (i=0;i<24;i=i+1) begin : INPUTS
            if (TRIG_POLARITY[i]==1'b0) begin : NORM
                IBUFDS_IBUFDISABLE u_ibuf(.IBUFDISABLE(~trigin_en[i]),.I(THRESH[i]),.IB(TRIG[i]),.O(raw_trig_in[i]));
                assign trig_in[i] = raw_trig_in[i];
            end else begin : INVERT
                IBUFDS_IBUFDISABLE u_ibuf(.IBUFDISABLE(~trigin_en[i]),.I(TRIG[i]),.IB(THRESH[i]),.O(raw_trig_in[i]));
                assign trig_in[i] = ~raw_trig_in[i];
            end
            // OK, try the trig oneshot. This time pass to slow clock
            // and out of here.
            radiant_trig_oneshot u_oneshot( .fast_clk_i( trig_clk_i ),
                                            .slow_clk_i( clk_i ),
                                            .stuck_ce_i( stuck_period ),
                                            .rst_i( ~trigin_en[i] ),
                                            .trigger_i(trig_in[i]),
                                            .trig_o(trigger_inputs[i]),
                                            .scal_o(scal_o[i]));
        end
        for (t=0;t<NUM_TRIG;t=t+1) begin : TRIGGERS
            reg trigger_rereg = 0;
            // trigger takes trig_i, en_i, oneshot_i, threshold_i
            radiant_trigger u_trigger(.trig_i(trigger_inputs),
                                      .en_i(trig_maskb[ NUM_CH*t +: NUM_CH ] ),
                                      .oneshot_i( trig_window[ ONESHOT_WIDTH*t +: ONESHOT_WIDTH ] ),
                                      .threshold_i( trig_thresh[ THRESH_WIDTH*t +: THRESH_WIDTH ] ),
                                      .rst_i( ~trig_en[t] ),
                                      .trig_clk_i(trig_clk_i),
                                      .trigger_o( trig_o[t]));                                      
            always @(posedge trig_clk_i) begin : RR
                trigger_rereg <= trig_o[t];
            end
            assign trigger_flag[t] = (trig_o[t] && !trigger_rereg);
            flag_sync u_tsync(.in_clkA(trigger_flag[t]),.out_clkB(trigger_flag_clk[t]),.clkA(trig_clk_i),.clkB(clk_i));
        end
    endgenerate        
    reg [1:0] trigger_any = 0;
    wire trigger_any_flag = trigger_any[1] && !trigger_any[0];
    wire int_trigger_flag;    
    reg ext_flag = 0;
    
    always @(posedge trig_clk_i) begin
        trigger_any <= {trigger_any[0], |trig_o};
    end
    flag_sync u_tanysync(.in_clkA(trigger_any_flag),.clkA(trig_clk_i),.out_clkB(int_trigger_flag),.clkB(sys_clk_i));
    
    // testing testing testing
    trig_debug_ila u_ila(.clk(clk_i),
                         .probe0(trigger_flag_clk),
                         .probe1(scal_o));

    // these are inverted
    wire ext_in_b;
    wire ext_in;
    wire ext_out_b;
    IBUFDS_DIFF_OUT u_extin_ibuf(.I(TRIGIN_N),.IB(TRIGIN_P),.O(ext_in_b),.OB(ext_in));
    OBUFDS u_extout_obuf(.I(ext_out_b),.O(TRIGOUT_N),.OB(TRIGOUT_P));

    reg [1:0] last_ext_in = {2{1'b0}};
    wire [1:0] this_ext_in;
    reg ext_flag = 0;    
    wire ext_flag_sysclk;
    IDDR #(.DDR_CLK_EDGE("SAME_EDGE_PIPELINED")) u_extin_reg(.D(ext_in),
                                                             .CE(1'b1),
                                                             .C(trig_clk_i),
                                                             .Q1(this_ext_in[0]),
                                                             .Q2(this_ext_in[1]),
                                                             .R(1'b0),
                                                             .S(1'b0));
    always @(posedge trig_clk_i) begin
        last_ext_in <= this_ext_in;
        ext_flag <= (last_ext_in[1]==1'b0 && this_ext_in[0]) || (this_ext_in[1] && !this_ext_in[0]);
    end           
    flag_sync u_tinsync(.in_clkA(ext_flag),.clkA(trig_clk_i),.out_clkB(ext_flag_sysclk),.clkB(sys_clk_i));
    // Trigger overlord
    radiant_overlord_core u_overlord(   .clk_i(clk_i),
                                        .rst_i(rst_i),
                                        `WBS_CONNECT( overlord, wb ),
                                        .sys_clk_i(sys_clk_i),
                                         .pps_i(pps_i),
                                         .trig_o(full_trig_o),
                                         .deadtrig_o(),
                                         .ext_trig_o(ext_out_b),
                                         .ext_trig_i(ext_flag_sysclk),
                                         .trig_done_o(trig_done),
                                         
                                         .int_trig_i(int_trigger_flag),
                                         .readout_running_i(readout_running_i),
                                         .readout_done_i(readout_done_i),
                                         .readout_full_i(readout_full_i),
                                         .readout_full_thresh_o(readout_full_thresh_o));

endmodule
                            