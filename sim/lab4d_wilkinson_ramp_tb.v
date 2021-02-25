`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/12/2021 06:46:49 PM
// Design Name: 
// Module Name: lab4d_wilkinson_ramp_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module lab4d_wilkinson_ramp_tb();

    wire wclk;
    wire sysclk;
    wire clk;
    tb_rclk #(.PERIOD(5.0)) u_wclk(.clk(wclk));
    tb_rclk #(.PERIOD(10.0)) u_sysclk(.clk(sysclk));
    tb_rclk #(.PERIOD(20.0)) u_clk(.clk(clk));
    
    reg rst = 0;
    reg update = 0;
    reg do_ramp = 0;
    wire ramp_done;
    wire [1:0] ramp;
    wire [23:0] wclk_p;
    wire [23:0] wclk_n;
    wire dbg_ramp;
    
   lab4d_wilkinson_ramp_v2 #(.NUM_LABS(24),.NUM_RAMP(2),.TRISTATE_WCLK("TRUE"),.TRISTATE_RAMP("TRUE")) 
        u_ramp(.clk_i(clk),
               .sys_clk_i(sysclk),
               .wclk_i(wclk),
               .rst_i(rst),
               .update_i(update),
               .ramp_to_wclk_i(0),
               .wclk_stop_count_i(1024),
               .do_ramp_i(do_ramp),
               .ramp_done_o(ramp_done),
               .RAMP(ramp),
               .WCLK_P(wclk_p),
               .WCLK_N(wclk_n),
               .dbg_ramp_o(dbg_ramp));
               
    initial begin
        #500;
        @(posedge clk); #1 do_ramp = 1; @(posedge clk); #1 do_ramp = 0;
    end           
               

endmodule
