`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/02/2021 03:44:44 PM
// Design Name: 
// Module Name: calram_pedestal_tb
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


module calram_pedestal_tb;

    wire sysclk;
    tb_rclk #(.PERIOD(10.0)) u_sysclk(.clk(sysclk));
    wire clk;
    tb_rclk #(.PERIOD(20.0)) u_clk(.clk(clk));
    
    reg [11:0] lab_adr = {12{1'b0}};
    reg [11:0] lab_dat = {12{1'b0}};
    reg lab_wr = 0;
    
    reg en_i = 0;
    reg config_wr = 0;
    reg zc_mode = 0;
    wire zc_full;
    
    reg bram_en = 0;
    reg bram_wr = 0;
    wire ack;
    reg [11:0] adr = {12{1'b0}};
    reg [26:0] dat_in = {27{1'b0}};
    wire [26:0] dat_out;
    
    calram_pedestal u_calram(.sys_clk_i(sysclk),
                             .lab_dat_i(lab_dat),
                             .lab_adr_i(lab_adr),
                             .lab_wr_i(lab_wr),
                             .en_i(en_i),
                             .config_wr_i(config_wr),
                             .zc_mode_i(zc_mode),
                             .zc_full_o(zc_full),
                             .clk_i(clk),
                             .bram_en_i(bram_en),
                             .bram_wr_i(bram_wr),
                             .ack_o(ack),
                             .adr_i(adr),
                             .dat_i(dat_in),
                             .dat_o(dat_out));

    `define wait12 @(posedge sysclk); @(posedge sysclk); @(posedge sysclk); @(posedge sysclk); @(posedge sysclk); @(posedge sysclk); @(posedge sysclk); @(posedge sysclk); @(posedge sysclk); @(posedge sysclk); @(posedge sysclk); @(posedge sysclk)

    integer i;

    task ram_read;
        input [11:0] address;
        output [26:0] data;
        begin
            @(posedge clk); #1 bram_en = 1; bram_wr = 0; adr = address; @(posedge clk); #1 data = dat_out;
            while (!ack) begin
                @(posedge clk);
                #1 data = dat_out;
            end
            #1 bram_en = 0;
            @(posedge clk);
        end
    endtask

    task ram_write;
        input [11:0] address;
        input [26:0] data;
        begin
            @(posedge clk); #1 bram_en = 1; bram_wr = 1; adr = address; dat_in = data; @(posedge clk); #1 bram_en = 0; bram_wr = 0;
            while (!ack) @(posedge clk);
        end
    endtask    

    task lab_write;
        input [11:0] address;
        input [11:0] data;
        begin
            @(posedge sysclk); #1 lab_adr = address; lab_dat = data; lab_wr = 1; @(posedge sysclk); #1 lab_wr = 0;
            `wait12;
        end
    endtask       

    reg [26:0] my_data = {27{1'b0}};



    initial begin
        #100;
        // First thing we need to do is erase the BRAMs.
        for (i=0;i<4096;i=i+1) begin
            ram_write(i, 0);
        end
        #100;
        @(posedge sysclk); #1 config_wr = 1; zc_mode = 0; @(posedge sysclk); #1 config_wr = 0;
        @(posedge sysclk); #1 en_i = 1; @(posedge clk);        
        #100;
        // For our first test we just cycle through 2 addresses, which is enough to test things.
        lab_write(0, 1000);
        lab_write(1, 1200);
        lab_write(0, 998);
        lab_write(1, 1202);
        #100;
        ram_read(0, my_data);
        if (my_data != 1998) begin
            $display("pedestal %0d = %0d does not match expected %0d", 0, my_data, 1998);
            $finish;
        end else begin
            $display("pedestal data %0d matches expected", 0);
        end
        ram_read(1, my_data);
        if (my_data != 2402) begin
            $display("pedestal %0d = %0d does not match expected %0d", 1, my_data, 2402);
            $finish;
        end else begin
            $display("pedestal data %0d matches expected", 1);
        end
        
        #100;
        @(posedge sysclk); #1 en_i = 0; @(posedge sysclk);
        // Now we test ZC mode. For this, we initialize 0 and 1 to 512*1000 and 512*1200.
        // (everyone else is still zero)
        ram_write(0, 512*1000);
        ram_write(1, 512*1200);
        #100;
        // and switch to ZC mode.
        @(posedge sysclk); #1 config_wr = 1; zc_mode = 1; @(posedge sysclk); #1 config_wr = 0;
        @(posedge sysclk); #1 en_i = 1; @(posedge clk);
        #100;
        // Now we need to feed in all 3 examples. First, both positive.
        lab_write(0, 1001);
        lab_write(1, 1201);
        
        #100;
        // let's check if it stayed zero. Check only addr 1! Addr 0 is garbage.
        ram_read(1, my_data);
        my_data = my_data & 9'h1FF;
        $display("after p->p count is %0d", my_data);
        #100;               

        // Next positive -> nonpositive
        lab_write(0, 1001);
        lab_write(1, 1200);

        #100;
        // let's check if it stayed zero. Check only addr 1! Addr 0 is garbage.
        ram_read(1, my_data);
        my_data = my_data & 9'h1FF;
        $display("after p->n count is %0d", my_data);
        #100;               

        // Next nonpositive->nonpositive
        lab_write(0, 1000);
        lab_write(1, 1200);
        
        #100;
        // let's check if it stayed zero. Check only addr 1! Addr 0 is garbage.
        ram_read(1, my_data);
        my_data = my_data & 9'h1FF;
        $display("after n->n count is %0d", my_data);
        #100;                       
        
        // nonpositive->positive
        lab_write(0, 999);
        lab_write(1, 1201);
        
        #100;
        // let's check if it's 1. Check only addr 1! Addr 0 is garbage.
        ram_read(1, my_data);
        my_data = my_data & 9'h1FF;
        $display("after n->p count is %0d", my_data);
        #100;                       
        
        // now try another 'wrong' case
        lab_write(0, 1500); // +
        lab_write(1, 1000); // -
        
        #100;
        ram_read(1, my_data);
        my_data = my_data & 9'h1FF;
        $display("after p->n count is %0d", my_data);
        #100;
        
        // and now try a 'right' one and see if it goes to 2
        lab_write(0, 900);  // -
        lab_write(1, 1500); // +
        
        #100;
        ram_read(1, my_data);
        my_data = my_data & 9'h1FF;
        $display("after n->p count is %0d", my_data);
        #100;
        
        // now we need to test zc_full
        for (i=0;i<510;i=i+1) begin
            lab_write(0, 900);
            lab_write(1, 1500);
        end
        #100;
        @(posedge sysclk); #1;
        if (!zc_full) $display("zc_full did not go high!");
        else $display("zc_full is high!");
        
    end        

endmodule
