`timescale 1ns/1ps
`include "radiant_debug.vh"
`include "wishbone.vh"

module radiant_trig_top(    input clk_i,
                            input rst_i,
                            `WBS_NAMED_PORT(wb, 32, 16, 4),
                            input pwm_clk_i,
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
    
    `WB_DEFINE(pwm, 32, 32, 4);
    wire pwm_is_selected = 1'b1; // this should be |wb_adr_i[15:9], make it 1 for now
    assign pwm_cyc_o = wb_cyc_i && pwm_is_selected;
    assign pwm_stb_o = wb_stb_i;
    assign pwm_we_o = wb_we_i;
    assign pwm_adr_o = { {23{1'b0}}, wb_adr_i[8:0] };
    assign pwm_dat_o = wb_dat_i;
    assign pwm_sel_o = wb_sel_i;
    
    // this needs to be a proper mux
    assign wb_ack_o = pwm_ack_i;
    assign wb_dat_o = pwm_dat_i;
    assign wb_err_o = pwm_ack_i;
    assign wb_rty_o = pwm_rty_i;
    
    // the address here is 
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
                            