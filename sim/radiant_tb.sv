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
    // Run in simulation mode (so we can use task writes)
    // and also set the polarity of the trigger to flat normal
    // (don't want to deal with it, unimportant)
    radiant_top #(.SIMULATION("TRUE"),
                  .TRIG_POLARITY({24{1'b0}})) uut( .SYS_CLK_P(clkin),
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
                 .TRIGIN_P(1'b0),
                 .TRIGIN_N(1'b1),
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

        // reset event FIFOs
        uut.u_bmif.BMWR('h30000, 'h4);
        #100;

        // reset DMA engine
        uut.u_bmif.BMWR('h8004, 'h7);
        #100;
        // write DMA descriptor 0: 0x30100, length 4. increment
        uut.u_bmif.BMWR('h8080, ('h30100 >> 2) | (4 << 19) | (1 << 18));
        #100;
        // write DMA descriptor 1: 0x20000, length 1024, no increment, final
        uut.u_bmif.BMWR('h8080, ('h20000 >> 2) | (1024<<19) | (1<<31));
        #100;
        // enable in event DMA mode
        uut.u_bmif.BMWR('h8000, 'h82000005);
        #100;
        
        // Enable the trigger inputs
        uut.u_bmif.BMWR('h30604, {24{1'b1}});
        #100;
        // Turn on trigger 0
        uut.u_bmif.BMWR('h30700, (1<<31));
        #100;
        // include everyone, I guess
        uut.u_bmif.BMWR('h30704, {24{1'b1}});
        #100;
        // Set coincidence window to I dunno, 200 ns. This is 73 extra clocks,
        // which is 31+31+11. This is (11<10)|(31<<5)|31.
        uut.u_bmif.BMWR('h30708, (11<<10)|(31<<5)|31);
        #100;
        // Set threshold to 2 (3 channels needed.
        uut.u_bmif.BMWR('h3070C, 2);
        #100;
        // enable
        uut.u_bmif.BMWR('h30600, 1);
        #100;
                
        // start the LAB4 controller
        uut.u_bmif.BMWR('h10000, 'h2);
        #5000;

        // WAIT A FREAKING WHILE
        
        // TEST CPU INHIBIT
        
        // Overlord setup
        // bits[2:0] = 1
        // bits[10:8] = 1
        // bits[18:16] = 011
        // bits[24:28] = 0        
        uut.u_bmif.BMWR('h30400, 'h00030101);
        
        #1000;
        // I DUNNO TRY TRIGGERING
        trig = 24'h1;
        #10;
        trig = 24'h0;
        #10;
        trig = 24'h2;
        #10;
        trig = 24'h0;
        #10;
        trig = 24'h4;
        #10;
        trig = 24'h0;
        
        #2000000;
        // CPU clear
        uut.u_bmif.BMWR('h30404, 'h2);
        
    end

endmodule
