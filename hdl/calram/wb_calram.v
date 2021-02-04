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
endmodule
