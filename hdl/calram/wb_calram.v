`timescale 1ns / 1ps
// WISHBONE calram interface.
// CALRAM needs 4 bytes * 4096 * 24 = 384 kB of address space (we use 72 BRAMs).
// We have 512 kB of address space, so we split it up into 32 16 kB chunks.
// Chunks 0-23 are the individual LAB spaces.
// Chunk 24 is the control space.
// Chunks 25-30 are shadows for 1->6.
// Chunk 31 is magic - on a write, it writes to chunks 0-23, on a read it echoes chunk 7.
//
// chunk 24 addr 0 controls the enable, address counter reset, ZC full indicator.
// chunk 24 addr 1 controls the config.
// chunk 24 addr 2 controls the roll counter.
//
// Right now we keep the address counter internal because I'm lazy: the addresses coming
// out of the LAB4 always run in sequence anyway. The ZC full indicator is because since the
// ZC RAM only goes up to 511, once we hit 511, we *can't* loop anymore otherwise we might
// overflow. And we don't know how many events that'll take. Once the ZC full indicator hits,
// at the roll end, we disable the calram running.
// The roll counter indicates how many 4096 chunks we've processed while active, which means
// you can run bursts of events and then check the ZC full indicator periodically.
//
// addr 0: bit 0: enable
//         bit 1: address counter reset (one-shot, no need to reset)
//         bit 2: ZC full
// addr 1: bit 0: ZC mode (if 0, pedestal mode). MUST WRITE THIS REGISTER FIRST, either 0 or 1!
// addr 2: bits [31:0] = roll counter. This counts the number of 4096 chunks we've processed.
`include "wishbone.vh"
module wb_calram #(parameter NUM_LABS=24, parameter LAB4_BITS=12)
                ( input clk_i,
                  input rst_i,
		          `WBS_NAMED_PORT(wb, 32, 19, 4),
                  
                  input sys_clk_i,
                  input [NUM_LABS*LAB4_BITS-1:0] lab_dat_i,
                  input [NUM_LABS-1:0] lab_wr_i                  
    );

    wire [NUM_LABS-1:0] rollover;
    wire [11:0] lab_addr[NUM_LABS-1:0];

    wire [NUM_LABS-1:0] calram_done;    
    wire [NUM_LABS-1:0] calram_rollover;
    
    wire [31:0] calram_ack;
    wire [31:0] calram_mux_out[31:0];

    wire [4:0] chunk_addr = wb_adr_i[14 +: 5];
    wire [11:0] wb_ram_addr = wb_adr_i[2 +: 12];
    wire [1:0] local_addr = wb_adr_i[2 +: 2];
    
    reg en_clk = 0;
    reg wr_reg = 0;
    wire wr = (wb_cyc_i && wb_stb_i && wb_we_i && !wr_reg);
    wire en_wr = (chunk_addr == 24 && local_addr == 0 && wr);    
    wire en_wr_sysclk;
    flag_sync u_enwrsync(.in_clkA(en_wr),.clkA(clk_i),.out_clkB(en_wr_sysclk),.clkB(sys_clk_i));
    
    reg [NUM_LABS-1:0] en_sysclk = {NUM_LABS{1'b0}};

    reg config_wr = 0;
    wire config_wr_sysclk;
    flag_sync u_wrsync(.in_clkA(config_wr),.clkA(clk_i),.out_clkB(config_wr_sysclk),.clkB(sys_clk_i));

    reg zc_mode = 0;
    reg reset_counter = 0;    
    wire reset_counter_sysclk;
    flag_sync u_resetsync(.in_clkA(reset_counter),.clkA(clk_i),.out_clkB(reset_counter_sysclk),.clkB(sys_clk_i));

    wire [NUM_LABS-1:0] zc_full;
    reg zc_full_reg = 0;
    (* ASYNC_REG = "TRUE" *)
    reg [1:0] zc_full_clk = {2{1'b0}};
        
    wire rollover_sysclk = calram_rollover[0];
    wire rollover_clk;
    flag_sync u_rollsync(.in_clkA(rollover_sysclk),.clkA(sys_clk_i),.out_clkB(rollover_clk),.clkB(clk_i));

    wire [47:0] roll_count;
    dsp_counter_terminal_count #(.FIXED_TCOUNT("TRUE"),.FIXED_TCOUNT_VALUE(32'hFFFFFFFF),.HALT_AT_TCOUNT("TRUE"))
        u_roll_counter(.clk_i(clk_i),
                       .rst_i(reset_counter),
                       .count_i(rollover_clk),
                       .tcount_reached_o(),
                       .count_o(roll_count));

    wire [31:0] en_reg = { {30{1'b0}}, zc_full_clk, 1'b0, en_clk };
    wire [31:0] mode_reg = { {31{1'b0}}, zc_mode };
    wire [31:0] count_reg = roll_count[31:0];
    wire [31:0] control_mux[3:0];
    
    reg control_ack = 0;

    assign control_mux[0] = en_reg;
    assign control_mux[1] = mode_reg;
    assign control_mux[2] = count_reg;
    assign control_mux[3] = control_mux[1];
        
    assign calram_mux_out[24] = control_mux[local_addr];
    assign calram_ack[24] = control_ack;
    assign calram_mux_out[25] = calram_mux_out[9];
    assign calram_ack[25] = calram_ack[9];
    assign calram_mux_out[26] = calram_mux_out[10];
    assign calram_ack[26] = calram_ack[10];
    assign calram_mux_out[27] = calram_mux_out[11];
    assign calram_ack[27] = calram_ack[11];
    assign calram_mux_out[28] = calram_mux_out[12];
    assign calram_ack[28] = calram_ack[12];
    assign calram_mux_out[29] = calram_mux_out[13];
    assign calram_ack[29] = calram_ack[13];
    assign calram_mux_out[30] = calram_mux_out[14];
    assign calram_ack[30] = calram_ack[14];
    assign calram_mux_out[31] = calram_mux_out[15];
    assign calram_ack[31] = calram_ack[15];
    
    assign wb_dat_o = calram_mux_out[chunk_addr];
    assign wb_ack_o = calram_ack[chunk_addr];
    assign wb_err_o = 1'b0;
    assign wb_rty_o = 1'b0;
    
    always @(posedge clk_i) begin    
        wr_reg <= wr;        
        if (chunk_addr == 24 && local_addr == 0 && wr) en_clk <= wb_dat_i[0];

        if (chunk_addr == 24 && local_addr == 0 && wr) reset_counter <= wb_dat_i[1];
        else reset_counter <= 0;
        if (chunk_addr == 24 && local_addr == 1 && wr) zc_mode <= wb_dat_i[2];
        if (chunk_addr == 24 && local_addr == 1 && wr) config_wr <= 1;
        else config_wr <= 0;        

        zc_full_clk <= {zc_full_clk[0], zc_full_reg };

        control_ack <= wb_cyc_i && wb_stb_i;
    end        

    always @(posedge sys_clk_i) begin
        if (en_wr_sysclk) en_sysclk <= {NUM_LABS{en_clk}};
        else if (|(zc_full & calram_rollover)) en_sysclk <= {NUM_LABS{1'b0}};
        
        if (config_wr_sysclk) zc_full_reg <= 0;
        else if (|(zc_full & calram_rollover)) zc_full_reg <= 1;
    end
            
    generate
        genvar i;
        for (i=0;i<NUM_LABS;i=i+1) begin : LL
            reg rollover_reg = 0;
            assign calram_rollover[i] = rollover_reg;
            wire max_reached;
            wire [47:0] cur_count;
            always @(posedge sys_clk_i) begin : LOG
                if (rollover_reg) rollover_reg <= 0;
                else rollover_reg <= max_reached && calram_done[i];
            end
            dsp_counter_terminal_count #(.FIXED_TCOUNT("TRUE"),.FIXED_TCOUNT_VALUE(4095),.HALT_AT_TCOUNT("TRUE"))
                u_lab_counter(.clk_i(sys_clk_i),
                              .rst_i(reset_counter_sysclk || rollover_reg),
                              .count_i(lab_wr_i[i]),
                              .tcount_reached_o(max_reached),
                              .count_o(cur_count));
            assign lab_addr[i] = cur_count[11:0];
            calram_pedestal u_ram(.sys_clk_i(sys_clk_i),
                                  .lab_dat_i(lab_dat_i[LAB4_BITS*i +: LAB4_BITS]),
                                  .lab_wr_i(lab_wr_i[i]),
                                  .lab_adr_i(lab_addr[i]),
                                  .en_i(en_sysclk[i]),
                                  .config_wr_i(config_wr_sysclk),
                                  .zc_mode_i(zc_mode),
                                  .zc_full_o(zc_full[i]),
                                  .done_o(calram_done[i]),
                                  .clk_i(clk_i),
                                  .bram_en_i( wb_cyc_i && wb_stb_i ),
                                  .bram_wr_i( wb_cyc_i && wb_stb_i && wb_we_i && (chunk_addr == i) ),
                                  .ack_o(calram_ack[i]),
                                  .adr_i(wb_ram_addr),
                                  .dat_i(wb_dat_i[26:0]),
                                  .dat_o(calram_mux_out[i]));                                  
        end
    endgenerate
    
                              
endmodule
