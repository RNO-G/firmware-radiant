`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/27/2021 03:30:28 PM
// Design Name: 
// Module Name: radiant_tb
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


module radiant_tb;

    wire clk50;
    tb_rclk #(.PERIOD(20.0)) u_clk50(.clk(clk50));
    wire clkin;
    wire clkin_b = !clkin;
    tb_rclk #(.PERIOD(40.0)) u_sysclk(.clk(clkin));    
    
    wire [1:0] ssincr;
    wire [1:0] ramp;
    pulldown(ssincr[0]);
    pulldown(ssincr[1]);
    pulldown(ramp[0]);
    pulldown(ramp[1]);
    // ignore soooo much
    wire [23:0] doe = {24{1'b0}};
    wire [23:0] doe_b = ~doe;
    wire [1:0] montiming = {2{1'b0}};
    wire [1:0] montiming_b = ~montiming;
    wire syncmon;
    pulldown(syncmon);
    
    reg [23:0] trig = {24{1'b0}};

    reg sclk = 0;
    reg mosi = 0;
    reg cs_b = 1'b1;
    radiant_top #(.SIMULATION("TRUE")) uut( .SYS_CLK_P(clkin),
                 .SYS_CLK_N(clkin_b),
                 .CLK50(clk50),
                 .BM_RX(1'b0),
                 .SSINCR_TDO(ssincr),
                 .RAMP(ramp),
                 .DOE_P(doe),
                 .DOE_N(doe_b),
                 .MONTIMING_P(montiming),
                 .MONTIMING_N(montiming_b),
                 .SYNCMON(syncmon),
                 .TRIG(trig),
                 .THRESH(~trig),
                 .PPS(1'b0),
                 .CB_SCLK_P(sclk),
                 .CB_MOSI_P(mosi),
                 .CB_CS_B_P(cs_b),
                 .MISO(1'b0));
    
    initial begin
        #5000;
        // let's test the PWM core first
        uut.u_bmif.BMWR('h30604, 'h00000000);
        #100;
        uut.u_bmif.BMWR('h30700, 'd6000000);
        #100;
        // set trigger repeat to 1
        uut.u_bmif.BMWR('h10054, 'h81000000);
        #100;
        // set the calram to none
        uut.u_bmif.BMWR('h80000, 0);
        #100;
        uut.u_bmif.BMWR('h80004, 0);
        #100;
        // start the LAB4 controller
        uut.u_bmif.BMWR('h10000, 'h2);
        #5000;
        // force trigger
        uut.u_bmif.BMWR('h10054, 'h2);
    end

endmodule
