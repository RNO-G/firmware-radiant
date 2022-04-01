`timescale 1ns / 1ps
module trigger_tb;

    wire tclk;
    tb_rclk #(.PERIOD(2.5)) u_tclk(.clk(tclk));
    wire clk50;
    tb_rclk #(.PERIOD(20.0)) u_sysclk(.clk(clk50));
    
    wire stuckce;
    clk_div_ce #(.CLK_DIVIDE(4)) u_stuckce(.clk(clk50),.ce(stuckce));
    
    reg [23:0] trig_en = {24{1'b0}};
    reg [23:0] trig_rst = {24{1'b0}};
    
    reg [23:0] trig_in = {24{1'b0}};
    reg [4:0] delay[3:0];
    wire [19:0] tot_delay = { delay[3], delay[2], delay[1], delay[0] };

    reg rst = 1;
    reg [5:0] threshold = {6{1'b0}};
    
    wire [23:0] os_to_trig;
    wire [23:0] os_scal;
    generate
        genvar i;
        for (i=0;i<24;i=i+1) begin : OS
            radiant_trig_oneshot u_os(.fast_clk_i(tclk),
                                      .slow_clk_i(clk50),
                                      .stuck_ce_i(stuckce), 
                                      .rst_i(trig_rst[i]),
                                      .trigger_i(trig_in[i]),
                                      .trig_o(os_to_trig[i]),
                                      .scal_o(os_scal[i]));
        end
    endgenerate
    wire trigger;
    radiant_trigger u_trig( .trig_i( os_to_trig ),
                            .en_i( trig_en ),
                            .oneshot_i( tot_delay ),
                            .rst_i( rst ),
                            .trig_clk_i(tclk),
                            .threshold_i( threshold ),
                            .trigger_o( trigger ));
//    module radiant_trigger(
//            input [23:0] trig_i,
//            input [23:0] en_i,
//            input [19:0] oneshot_i,
//            input rst_i,
//            input trig_clk_i,
//            input [4:0] threshold_i,
//            output trigger_o
//    );
    // try a oneshot of 100 ns = 40. Do 31 in the first, 9 in the second.
    // Total range is really up to 124.
    initial begin
        delay[0] <= 5'd30;
        delay[1] <= 5'd9;
        delay[2] <= 5'd0;
        delay[3] <= 5'd0;
        threshold <= 6'd1;
        trig_en <= 24'h000003;
        #100;
        @(posedge tclk); #1 rst = 0;
        #100;
        trig_in[0] <= 1;
        #5;
        trig_in[0] <= 0;
        #90;
        trig_in[1] <= 1;
        #5;
        trig_in[1] <= 0;
    end
        

endmodule
