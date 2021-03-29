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
//         bit 1: unused
//         bit 2: ZC full
//         bit 3: roll complete (set when a roll completes, cleared by any other write)
// addr 1: bit 0: ZC mode (if 0, pedestal mode). MUST WRITE THIS REGISTER FIRST, either 0 or 1!
//         bit 1: zero (if set, the BRAMs will be fed with zero, regardless)
//         bit 2: ZC read mode (if 1, reads from the BRAM space will only read the low 9 bits - the ZC data)
//                This ONLY AFFECTS the WISHBONE side, and DOES NOT affect writes!
//         bit 3: adjust mode. When calram mode (enable) is not enabled, the event path is pedestal-adjusted.
//         bit 4: adjustments are *negative* adjustments, not positive. This is probably the more useful
//                since the low portion of the dynamic range is unusable.
// addr 2: bits [31:0] = roll counter. This counts the number of 4096 chunks we've processed.

// This is v2: it takes the output from the par_lab4d_readout, and also subsequently outputs
// the processed data to go into the LAB4 FIFOs.
`include "wishbone.vh"
module wb_calram_v2 #(parameter NUM_LABS=24, 
                   parameter LAB4_BITS=12, 
                   parameter DATA_BITS=16,
                   parameter LAB4_SAMPLE_BITS=12,
                   parameter HEADER_BITS=4,
                   parameter HEADER_STOP_BIT=1)
                ( input clk_i,
                  input rst_i,
		          `WBS_NAMED_PORT(wb, 32, 19, 4),
                  
                  input sys_clk_i,
                  // data from the output
                  input [NUM_LABS*LAB4_BITS-1:0] lab_dat_i,
                  // write strobes from the output
                  input [NUM_LABS-1:0] lab_wr_i,
                  // sample counter from the output
                  input [NUM_LABS*LAB4_SAMPLE_BITS-1:0] lab_sample_i,
                  // header bits from the output.
                  input [NUM_LABS*HEADER_BITS-1:0] lab_header_i,
                  
                  // outputs to the FIFO
                  output [NUM_LABS*DATA_BITS-1:0] lab_dat_o,
                  // write strobe output
                  output [NUM_LABS-1:0] lab_wr_o
    );
    
    localparam DEBUG = "FALSE";

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
    reg adjust_clk = 0;
    reg adjust_neg_clk = 0;
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
    reg zero_mode = 0;
    reg zc_read_mode = 0;
    // stop Vivado from whining
    (* ASYNC_REG = "TRUE" *)
    reg [1:0] zero_mode_sysclk = {2{1'b0}};

    wire [NUM_LABS-1:0] zc_full;
    reg zc_full_reg = 0;
    (* ASYNC_REG = "TRUE" *)
    reg [1:0] zc_full_clk = {2{1'b0}};
        
    wire rollover_sysclk = calram_rollover[0];
    wire rollover_clk;
    flag_sync u_rollsync(.in_clkA(rollover_sysclk),.clkA(sys_clk_i),.out_clkB(rollover_clk),.clkB(clk_i));

    wire wr_sysclk = lab_wr_i[0];
    reg roll_complete_sysclk = 0;
    (* ASYNC_REG = "TRUE" *)
    reg [1:0] roll_complete_clk = {2{1'b0}};
    

    wire [47:0] roll_count;
    dsp_counter_terminal_count #(.FIXED_TCOUNT("TRUE"),.FIXED_TCOUNT_VALUE(32'hFFFFFFFF),.HALT_AT_TCOUNT("TRUE"))
        u_roll_counter(.clk_i(clk_i),
                       .rst_i(reset_counter),
                       .count_i(rollover_clk),
                       .tcount_reached_o(),
                       .count_o(roll_count));

    wire [31:0] en_reg = { {28{1'b0}}, roll_complete_clk[1], zc_full_clk, 1'b0, en_clk };
    wire [31:0] mode_reg = { {27{1'b0}}, adjust_neg_clk, adjust_clk, zc_read_mode, zero_mode, zc_mode };
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
        roll_complete_clk <= { roll_complete_clk[0], roll_complete_sysclk };
    
        wr_reg <= wr;        
        if (chunk_addr == 24 && local_addr == 0 && wr) en_clk <= wb_dat_i[0];

        // Mode register stuff. This MUST be written EVERY TIME when register 0 is written too.
        if (chunk_addr == 24 && local_addr == 1 && wr) adjust_clk <= wb_dat_i[3];
        if (chunk_addr == 24 && local_addr == 1 && wr) adjust_neg_clk <= wb_dat_i[4]; 
        if (chunk_addr == 24 && local_addr == 1 && wr) zc_mode <= wb_dat_i[0];
        if (chunk_addr == 24 && local_addr == 1 && wr) zero_mode <= wb_dat_i[1];
        if (chunk_addr == 24 && local_addr == 1 && wr) config_wr <= 1;
        else config_wr <= 0;        

        if (chunk_addr == 24 && local_addr == 1 && wr) zc_read_mode <= wb_dat_i[2];

        zc_full_clk <= {zc_full_clk[0], zc_full_reg };

        control_ack <= wb_cyc_i && wb_stb_i;
    end        

    always @(posedge sys_clk_i) begin
        zero_mode_sysclk <= {zero_mode_sysclk[0], zero_mode};
    
        if (wr_sysclk) roll_complete_sysclk <= 0;
        else if (rollover_sysclk) roll_complete_sysclk <= 1;
    
        if (en_wr_sysclk) en_sysclk <= {NUM_LABS{en_clk}};
        else if (|(zc_full & calram_rollover)) en_sysclk <= {NUM_LABS{1'b0}};
        
        if (config_wr_sysclk) zc_full_reg <= 0;
        else if (|(zc_full & calram_rollover)) zc_full_reg <= 1;
    end
    
    reg [11:0] debug_data = {12{1'b0}};
    reg [3:0] debug_header = {4{1'b0}};
    reg       debug_wr = 0;
    wire [4:0] debug_sel;

    integer d;
    always @(posedge sys_clk_i) begin
        for (d=0;d<NUM_LABS;d=d+1) begin
            if (debug_sel == d) begin
                debug_wr <= lab_wr_o[d];
                debug_header <= lab_dat_o[DATA_BITS*d + LAB4_BITS +: HEADER_BITS];
                debug_data <= lab_dat_o[DATA_BITS*d +: LAB4_BITS];
            end
        end
    end
    
    wb_calram_ila u_ila(.clk(sys_clk_i),.probe0(debug_wr),.probe1(debug_header),.probe2(debug_data));
    wb_calram_vio u_vio(.clk(sys_clk_i),.probe_out0(debug_sel));            
    generate
        genvar i;
        for (i=0;i<NUM_LABS;i=i+1) begin : LL
            localparam DEBUG = (i == 6 || i==0) ? "TRUE" : "FALSE";
            reg rollover_reg = 0;
            assign calram_rollover[i] = rollover_reg;
            reg max_reached = 0;
            wire lab_stop;
            reg [3:0] lab_header = {4{1'b0}};
            always @(posedge sys_clk_i) begin : LOG
                if (rollover_reg) rollover_reg <= 0;
                else rollover_reg <= max_reached && calram_done[i];                
                
                if (lab_wr_i[i]) max_reached <= (lab_sample_i == {LAB4_SAMPLE_BITS{1'b1}});
                if (lab_wr_i[i]) lab_header <= lab_header_i[HEADER_BITS*i +: HEADER_BITS];
            end
            // We don't use our own counter anymore.
//            dsp_counter_terminal_count #(.FIXED_TCOUNT("TRUE"),.FIXED_TCOUNT_VALUE(4095),.HALT_AT_TCOUNT("TRUE"))
//                u_lab_counter(.clk_i(sys_clk_i),
//                              .rst_i(reset_counter_sysclk || rollover_reg),
//                              .count_i(calram_done[i]),
//                              .tcount_reached_o(max_reached),
//                              .count_o(cur_count));
            assign lab_addr[i] = lab_sample_i[LAB4_SAMPLE_BITS*i +: LAB4_SAMPLE_BITS];
            assign lab_stop = lab_header_i[HEADER_BITS*i + HEADER_STOP_BIT];
            assign lab_dat_o[DATA_BITS*i + LAB4_BITS +: HEADER_BITS] = lab_header;
            calram_pedestal #(.DEBUG(DEBUG)) u_ram(.sys_clk_i(sys_clk_i),
                                  .lab_dat_i(lab_dat_i[LAB4_BITS*i +: LAB4_BITS]),
                                  .lab_wr_i(lab_wr_i[i]),
                                  .lab_adr_i(lab_addr[i]),
                                  .lab_stop_i(lab_stop),
                                  
                                  // data outputs
                                  .lab_dat_o(lab_dat_o[DATA_BITS*i +: LAB4_BITS]),
                                  .lab_wr_o(lab_wr_o[i]),
                                  
                                  .en_i(en_sysclk[i]),
                                  .adjust_i(adjust_clk),
                                  .adj_neg_i(adjust_neg_clk),
                                  .config_wr_i(config_wr_sysclk),
                                  .zc_mode_i(zc_mode),
                                  .zero_i(zero_mode_sysclk[1]),
                                  .zc_full_o(zc_full[i]),
                                  .done_o(calram_done[i]),
                                  .clk_i(clk_i),
                                  .bram_en_i( wb_cyc_i && wb_stb_i ),
                                  .bram_wr_i( wb_cyc_i && wb_stb_i && wb_we_i && ((chunk_addr == i) || &chunk_addr) ),
                                  .zc_read_i(zc_read_mode),
                                  .ack_o(calram_ack[i]),
                                  .adr_i(wb_ram_addr),
                                  .dat_i(wb_dat_i[26:0]),
                                  .dat_o(calram_mux_out[i]));                                  
        end
        
        
    endgenerate
    
                              
endmodule
