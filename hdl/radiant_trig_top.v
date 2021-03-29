`timescale 1ns/1ps
`include "radiant_debug.vh"
`include "wishbone.vh"

module radiant_trig_top(    input clk_i,
                            input rst_i,
                            `WBS_NAMED_PORT(wb, 32, 16, 4),
                            input pwm_clk_i,
                            
                            // trigger event generation
                            input event_i,
                            input [31:0] event_info_i,
                            // trigger event push (DMA trigger)
                            input event_done_i,
                            
                            // Actual DMA trigger
                            output event_ready_o,
                            // Response from DMA that readout is beginning.
                            input event_readout_ready_i,
                            
                            
                            // PPS input. This is a flag in sysclk.
                            input pps_i,
                                                                                                                                            
                            input [23:0] TRIG,
                            input [23:0] THRESH,
                            output [23:0] THRESH_PWM,
                            output trig_o );

    parameter TRIG_POLARITY = {24{1'b0}};
    parameter DEBUG = `RADIANT_TRIG_TOP_DEBUG;
    
    wire [23:0] raw_trig_in;
    generate
        genvar i;
        for (i=0;i<24;i=i+1) begin : TRIGIN
            if (TRIG_POLARITY[i]==1'b0) begin : NORM
                IBUFDS u_ibuf(.I(THRESH[i]),.IB(TRIG[i]),.O(raw_trig_in[i]));
            end else begin : INVERT
                IBUFDS u_ibuf(.I(TRIG[i]),.IB(THRESH[i]),.O(raw_trig_in[i]));
            end
        end
    endgenerate
    
    // Event control space is 0x0000 - 0x1FF. Right now shadowed above.
    `WB_DEFINE(ctrl, 32, 9, 4);
    wire ctrl_is_selected = (wb_adr_i[9] == 0);
    assign ctrl_cyc_o = wb_cyc_i && ctrl_is_selected;
    assign ctrl_stb_o = wb_stb_i;
    assign ctrl_we_o = wb_we_i;
    assign ctrl_adr_o = wb_adr_i[8:0];
    assign ctrl_dat_o = wb_dat_i;
    
    // PWM space is 0x0200-0x03FF nominally. Right now shadowed above.
    `WB_DEFINE(pwm, 32, 32, 4);
    wire pwm_is_selected = (wb_adr_i[9] == 1);
    assign pwm_cyc_o = wb_cyc_i && pwm_is_selected;
    assign pwm_stb_o = wb_stb_i;
    assign pwm_we_o = wb_we_i;
    assign pwm_adr_o = { {23{1'b0}}, wb_adr_i[8:0] };
    assign pwm_dat_o = wb_dat_i;
    assign pwm_sel_o = wb_sel_i;    
    
    // this needs to be a proper mux
    assign wb_ack_o = (ctrl_is_selected) ? ctrl_ack_i : pwm_ack_i;
    assign wb_dat_o = (ctrl_is_selected) ? ctrl_dat_i : pwm_dat_i;
    assign wb_err_o = (ctrl_is_selected) ? ctrl_err_i : pwm_err_i;
    assign wb_rty_o = (ctrl_is_selected) ? ctrl_rty_i : pwm_rty_i;
    
    // Event control core.
    // Contains PPS counter, sync, event counter, event generation, etc.
    radiant_event_ctrl u_evctrl(.clk_i(clk_i),
                                .rst_i(rst_i),
                                `WBS_CONNECT( ctrl, wb),
                                .sys_clk_i(sys_clk_i),
                                .event_i(event_i),
                                .event_info_i(event_info_i),
                                .event_done_i(event_done_i),
                                
                                .event_ready_o(event_ready_o),
                                .event_readout_ready_i(event_readout_ready_i),
                                
                                .pps_i(pps_i));
                                
    
    // PWM core
    pwm_wrap u_wrap(.clk_i(clk_i),
                    .rst_i(rst_i),
                    `WBS_CONNECT( pwm , wb ),
                    .pwm_clk_i(pwm_clk_i),
                    .THRESH_PWM(THRESH_PWM));

    // these need to get handled differently, but whatever
    (* IOB = "TRUE" *)
    reg [23:0] trig_in_debug = {24{1'b0}};
    reg [23:0] trig_in_debug_rereg = {24{1'b0}};
    always @(posedge pwm_clk_i) begin
        trig_in_debug <= raw_trig_in;
        trig_in_debug_rereg <= trig_in_debug;
    end
    generate
        if (DEBUG == "TRUE") begin : DBG
            trig_debug_ila u_ila(.clk(pwm_clk_i),.probe0(trig_in_debug_rereg));
        end
    endgenerate
endmodule
                            