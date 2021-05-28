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
                            // trigger event push (DMA trigger)
                            input event_done_i,

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
                                              
                            input [23:0] TRIG,
                            input [23:0] THRESH,
                            output [23:0] THRESH_PWM,
                            output [NUM_TRIG-1:0] trig_o );
    
    wire [23:0] raw_trig_in;
    wire [23:0] trig_in;
    
    // Event control space is 0x0000 - 0x1FF. 
    `WB_DEFINE(ctrl, 32, 9, 4);
    wire ctrl_is_selected = (wb_adr_i[10:9] == 2'b00);
    assign ctrl_cyc_o = wb_cyc_i && ctrl_is_selected;
    assign ctrl_stb_o = wb_stb_i;
    assign ctrl_we_o = wb_we_i;
    assign ctrl_adr_o = wb_adr_i[8:0];
    assign ctrl_dat_o = wb_dat_i;
    assign ctrl_sel_o = wb_sel_i;
    
    // PWM space is 0x0200-0x03FF nominally. Right now shadowed above.
    `WB_DEFINE(pwm, 32, 32, 4);
    wire pwm_is_selected = (wb_adr_i[10:9] == 2'b01);
    assign pwm_cyc_o = wb_cyc_i && pwm_is_selected;
    assign pwm_stb_o = wb_stb_i;
    assign pwm_we_o = wb_we_i;
    assign pwm_adr_o = { {23{1'b0}}, wb_adr_i[8:0] };
    assign pwm_dat_o = wb_dat_i;
    assign pwm_sel_o = wb_sel_i;    
    
    // Scaler space is 400-5FF. I honestly shouldn't even have these here,
    // I'm totally violating my original address space design. OH EFFING WELL.
    // Maybe these are internal trigger scalers or some wacknuts thing.
    `WB_DEFINE(scal, 32, 32, 4);
    wire scal_is_selected = (wb_adr_i[10:9] == 2'b10);
    assign scal_cyc_o = wb_cyc_i && scal_is_selected;
    assign scal_stb_o = wb_stb_i;
    assign scal_we_o = wb_we_i;
    assign scal_adr_o = { {23{1'b0}}, wb_adr_i[8:0] };
    assign scal_dat_o = wb_dat_i;
    assign scal_sel_o = wb_sel_i;
    `WBM_KILL( scal , 32 );
    
    // and trigger control space is 600-7FF
    `WB_DEFINE(trig, 32, 32, 4);
    wire trig_is_selected = (wb_adr_i[10:9] == 2'b11);
    assign trig_cyc_o = wb_cyc_i && trig_is_selected;
    assign trig_stb_o = wb_stb_i;
    assign trig_we_o = wb_we_i;
    assign trig_adr_o = { {23{1'b0}}, wb_adr_i[8:0] };
    assign trig_dat_o = wb_dat_i;
    assign trig_sel_o = wb_sel_i;    
    
    wire [3:0] ack_vec = { trig_ack_i , scal_ack_i, pwm_ack_i, ctrl_ack_i };
    wire [3:0] err_vec = { trig_err_i , scal_err_i, pwm_err_i, ctrl_err_i };
    wire [3:0] rty_vec = { trig_rty_i , scal_rty_i, pwm_rty_i, ctrl_rty_i };
    wire [31:0] dat_vec[3:0];
    assign dat_vec[3] = trig_dat_i;
    assign dat_vec[2] = scal_dat_i;
    assign dat_vec[1] = pwm_dat_i;
    assign dat_vec[0] = ctrl_dat_i;

    assign wb_ack_o = ack_vec[ wb_adr_i[10:9] ];
    assign wb_err_o = err_vec[ wb_adr_i[10:9] ];
    assign wb_rty_o = rty_vec[ wb_adr_i[10:9] ];
    assign wb_dat_o = dat_vec[ wb_adr_i[10:9] ];    
            
    // Event control core.
    // Contains PPS counter, sync, event counter, event generation, etc.
    radiant_event_ctrl u_evctrl(.clk_i(clk_i),
                                .rst_i(rst_i),
                                `WBS_CONNECT( ctrl, wb),
                                .sys_clk_i(sys_clk_i),
                                
                                .event_fifo_reset_o(event_fifo_reset_o),
                                .event_fifo_empty_i(event_fifo_empty_i),
                                    
                                .event_i(event_i),
                                .event_type_i(1'b0),
                                .event_info_i(event_info_i),
                                .event_done_i(event_done_i),
                                
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
    // testing testing testing
    trig_debug_ila u_ila(.clk(clk_i),
                         .probe0(trigger_flag_clk),
                         .probe1(scal_o));
                    
//    wire [23:0] scal_flag;    
//    radiant_trig_core u_core( .clk_i(clk_i),
//                              .rst_i(rst_i),
//                              `WBS_CONNECT( trig, wb ),
//                              .trig_clk_i(trig_clk_i),
//                              .trig_i(raw_trig_i),
//                              .scal_o(scal_flag),
//                              .trig_o(trig_o));
endmodule
                            