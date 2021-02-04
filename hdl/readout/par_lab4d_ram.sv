`include "wishbone.vh"
`timescale 1ns / 1ps
// This stores all the data from a LAB4 into a FIFO.
// The address space is 16 bits, and so the LAB4 selection
// picks off the needed number of top bits to split off.
// For instance, for 24, the top 5 bits are picked off
// (lab0 = 0000-07FF)
// (lab1 = 0800-0FFF)
// etc.
module par_lab4d_ram #(
        parameter NUM_SS_INCR = 2,
        parameter NUM_SRCLK = 2,
        parameter SRCLK_DIFFERENTIAL = "TRUE",
        parameter [NUM_SRCLK-1:0] SRCLK_POLARITY = {NUM_SRCLK{1'b0}},
        parameter NUM_LAB4 = 24,
        parameter LAB4_BITS = 12,
        parameter [NUM_LAB4-1:0] DOE_POLARITY = {NUM_LAB4{1'b0}},
        parameter SURF5_COMPAT="FALSE"
)(
		input clk_i,
		input rst_i,
		`WBS_NAMED_PORT(wb, 32, 16, 4),
		input dma_lock_i,
		output dma_locked_o,
		`WBS_NAMED_PORT(wbdma, 32, 16, 4),
		input sys_clk_i,
		input wclk_i,
		// Readout test pattern control
		input readout_test_pattern_i,
		// Readout controls, from the LAB4 controller.
		input readout_i,
		input [3:0] readout_header_i,
		input readout_rst_i,
		input readout_fifo_rst_i,
		input [9:0] readout_empty_size_i,
		output [NUM_LAB4-1:0] readout_fifo_empty_o,
		input [15:0] prescale_i,
		
		// Redirected readout data for CALRAM (or whatever...)
		output [NUM_LAB4*LAB4_BITS-1:0] lab_dat_o,
		output [NUM_LAB4-1:0] lab_wr_o,
		
		output complete_o,
		output [31:0] readout_debug_o,
		input [NUM_LAB4-1:0] DOE_LVDS_P,
		input [NUM_LAB4-1:0] DOE_LVDS_N,
		output [NUM_SS_INCR-1:0] SS_INCR,
		output [NUM_SRCLK-1:0] SRCLK_P,
		output [NUM_SRCLK-1:0] SRCLK_N,
		
		output [15:0] sample_debug
    );
    localparam L4W=$clog2(NUM_LAB4);
    localparam L4_BITS = LAB4_BITS;
    
	wire [NUM_SRCLK-1:0] SRCLK;
	wire [NUM_LAB4-1:0] DOE;
	wire [NUM_LAB4-1:0] DOE_B;
	generate
		genvar kk;
		for (kk=0;kk<NUM_LAB4;kk=kk+1) begin : DIFFLOOP
			// Polarity swap means we just swap the Ns and Ps. The actual polarity of the SRCLK[kk] signal is swapped in the shift register.
			if (SRCLK_DIFFERENTIAL == "TRUE") begin : DIFF
                if (SRCLK_POLARITY[kk] == 0) begin : POS
                    OBUFDS u_srclk_p(.I(SRCLK[kk]),.O(SRCLK_P[kk]),.OB(SRCLK_N[kk]));
                end else begin : NEG
                    OBUFDS u_srclk_n(.I(SRCLK[kk]),.O(SRCLK_N[kk]),.OB(SRCLK_P[kk]));
                end
            end else begin : SE
                // Just... no bufs.
                assign SRCLK_P[kk] = SRCLK[kk];
                assign SRCLK_N[kk] = ~SRCLK[kk];
            end
            if (DOE_POLARITY[kk] == 0) begin : DOEPOS
				IBUFDS_DIFF_OUT u_doe_p(.I(DOE_LVDS_P[kk]),.IB(DOE_LVDS_N[kk]),.O(DOE[kk]),.OB(DOE_B[kk]));
			end else begin : DOENEG
				IBUFDS_DIFF_OUT u_doe_n(.I(DOE_LVDS_N[kk]),.IB(DOE_LVDS_P[kk]),.O(DOE_B[kk]),.OB(DOE[kk]));
			end
		end
	endgenerate
		
	reg use_dma_input = 0;
	wire [31:0] local_dat_o;
	wire local_cyc_i;
	wire local_stb_i;
	wire local_we_i;
	wire local_ack_o;
	wire [15:0] local_adr_i;
	reg illegal_access_ack = 0;
	
	// Guard against changing DMA access during a control cycle.
	// The reverse (changing DMA access during a DMA cycle) is guarded elsewhere
	// because the DMA module handles lock/unlock.
	always @(posedge clk_i) begin
		if (dma_lock_i && !wb_cyc_i) use_dma_input <= 1;
		else if (!dma_lock_i && !wb_cyc_i) use_dma_input <= 0;
	end

	assign local_cyc_i = (use_dma_input) ? wbdma_cyc_i : wb_cyc_i;
	assign local_stb_i = (use_dma_input) ? wbdma_stb_i : wb_stb_i;
	assign local_we_i = (use_dma_input) ? wbdma_we_i : wb_we_i;
	assign local_adr_i = (use_dma_input) ? wbdma_adr_i : wb_adr_i;
	// The only thing we need to override is the ack - the local bus
	// will still get data, it'll just be nonsense.
	assign wb_dat_o = local_dat_o;
	assign wb_ack_o = (use_dma_input) ? illegal_access_ack : local_ack_o;

    // If SURF5_COMPAT is true, we use 14:11 no matter what.
    // Otherwise start as far back as needed, and no more.
    wire [L4W-1:0] l4_sel = (SURF5_COMPAT=="TRUE") ? local_adr_i[14:11] : local_adr_i[16-L4W +: L4W];


	// just straight assign these, they're guarded by firmware
	assign wbdma_dat_o = local_dat_o;
	assign wbdma_ack_o = local_ack_o;
	
	wire data_wr;
	wire [NUM_LAB4*L4_BITS-1:0] data;
	// This needs to be expanded to a proper power of 2...
	localparam DATA_OUT_SIZE = (1<<L4W);
	wire [31:0] data_out[DATA_OUT_SIZE-1:0];
	wire [11:0] lab_read;

	// We're now using a FWFT FIFO, with no embedded registers, but we register the data ourselves.
	reg [31:0] data_out_registered = {32{1'b0}};

	reg [9:0] readout_empty_threshold = {10{1'b0}};

	// So the state machine now goes IDLE -> ACK immediately, and then back to IDLE.
	// (making it a bit of a pointless state machine).
	// fifo_read gets asserted in IDLE, which means the data changes in ACK.
	// If everything stays asserted, that means data_out_registered is actually valid immediately again.
	// If we make burst reads work, need to figure out how that works
	//          state   new_fifo_read  cti data_out data_out_registered
	// clock 0: IDLE    1               0   D[0]     X
	// clock 1: ACK     1               0   D[1]     D[0]
	// clock 2: ACK     0               7   D[2]     D[1]
	// so new_fifo_read would be (local_cyc_i && local_stb_i && !local_we_i) && ((state==IDLE) || (state==ACK && cti_i != 7))
	// *should* work..... can probably simplify this to just && !(cti_i == 7 && local_ack_o)
	
	localparam FSM_BITS=2;
	localparam [FSM_BITS-1:0] IDLE = 0;
	localparam [FSM_BITS-1:0] READ = 1;
	localparam [FSM_BITS-1:0] WTF = 2;
	localparam [FSM_BITS-1:0] ACK = 3;
	reg [FSM_BITS-1:0] state = IDLE;
	always @(posedge clk_i) begin
		// buy time for the empty threshold
		readout_empty_threshold <= readout_empty_size_i;
	
		if (wb_cyc_i && wb_stb_i && use_dma_input && !illegal_access_ack) illegal_access_ack <= 1;
		else illegal_access_ack <= 0;
		
		case (state)
			IDLE: if (local_cyc_i && local_stb_i && !local_we_i) state <= ACK;	// read is 1 here
			READ: state <= ACK;														// data is at output here
			WTF: state <= ACK;														// why is this here?!?!?
			ACK: state <= IDLE;														// data is at register here
		endcase
	end

	wire fifo_read = (local_cyc_i && local_stb_i && !local_we_i) && (state == IDLE);
	assign local_ack_o = (state == ACK);
	
	always @(posedge clk_i) begin
		data_out_registered <= data_out[l4_sel];
	end
	assign local_dat_o = data_out_registered;

	reg [L4_BITS-1:0] readout_test_pattern = {L4_BITS{1'b0}};
	reg data_wr_reg = 0;
	always @(posedge sys_clk_i) begin
		if (readout_i && readout_test_pattern_i) readout_test_pattern <= {L4_BITS{1'b0}};
		else if (readout_test_pattern_i && data_wr) readout_test_pattern <= readout_test_pattern + 1;

		data_wr_reg <= data_wr;
	end
	
	generate
		genvar i;
		for (i=0;i<DATA_OUT_SIZE;i=i+1) begin : FIFOS
		    if (i < NUM_LAB4) begin : RL
                wire [15:0] data_input;
                wire [31:0] data_output;
                reg [15:0] data_in_reg = {16{1'b0}};
                // Each LAB gets 512 entries of space, or 2048 bytes: so e.g. from 0000-07FF.
                // So the lab selection picks off bits [14:11].
                // It's a FIFO, not actually a RAM, because we don't have enough address space.
                assign lab_read[i] = fifo_read && (local_adr_i[14:11] == i);
                always @(posedge sys_clk_i) begin : DATA_INPUT_REGISTER
                    if (data_wr) begin
                        if (readout_test_pattern_i) data_in_reg <= {readout_header_i, readout_test_pattern};
                        else data_in_reg <= {readout_header_i, data[L4_BITS*i +: L4_BITS]};
                    end
                end
                // Data write and data are delayed to let them come from registers.
                lab4d_fifo u_fifo(.wr_clk(sys_clk_i),.wr_en(data_wr_reg),.din(data_in_reg),
                                        .rst(readout_fifo_rst_i),.prog_empty(readout_fifo_empty_o[i]),
                                        .prog_empty_thresh(readout_empty_threshold),
                                        .rd_clk(clk_i),.rd_en(lab_read[i]),.dout(data_output));                                        
                assign data_out[i] = {data_output[15:0],data_output[31:16]};
                assign lab_dat_o[L4_BITS*i +: L4_BITS] = data_in_reg[0 +: L4_BITS];
                assign lab_wr_o[i] = data_wr_reg;
            end else begin : SHAD
                // We want to shadow things to keep the decode easy, so what we do is just shift
                // DATA_OUT_SIZE down by 1 and use that as a modulo. We don't just do modulo NUM_LAB4,
                // because that wraps to *increase* the coding: i.e. 24 = 0, so 0 matches 0 *and* 24,
                // but not 16, so everyone still needs all the bits.
                // Whereas this way 24 maps to 8, meaning for 8-15 it just
                // ignores the top bit, *reducing* the coding.
                assign data_out[i] = data_out[ i % (DATA_OUT_SIZE>>1) ];
            end
        end
	endgenerate
	
	assign dma_locked_o = use_dma_input;
	
	wire dbg_ss_incr;
	wire dbg_srclk;
	wire [6:0] sample_counter;
	wire [3:0] bit_counter;
	par_lab4d_data_shift_register #(.NUM_LAB4(NUM_LAB4),.NUM_SS_INCR(NUM_SS_INCR),.NUM_SRCLK(NUM_SRCLK))
	                                          u_shreg(.sys_clk_i(sys_clk_i),.wclk_i(wclk_i),
													  .readout_i(readout_i),.readout_rst_i(readout_rst_i),
													  .done_o(complete_o),.dat_o(data),
													  .dat_wr_o(data_wr),.prescale_i(prescale_i),
													  .ss_incr_o(dbg_ss_incr),.srclk_o(dbg_srclk),
													  .bit_counter_o(bit_counter),
													  .sample_counter_o(sample_counter),.DOE(DOE),.SS_INCR(SS_INCR),.SRCLK(SRCLK),
													  .sample_debug(sample_debug));
    // this is stupid fix this for Vivado
	assign readout_debug_o[0 +: L4_BITS] = data[0 +: L4_BITS];
	assign readout_debug_o[12] = DOE[0];
	assign readout_debug_o[13] = data_wr;
	assign readout_debug_o[14] = readout_i;
	assign readout_debug_o[15] = dbg_ss_incr;
	assign readout_debug_o[16] = dbg_srclk;
	assign readout_debug_o[17] = complete_o;
	assign readout_debug_o[18 +: 7] = sample_counter;
	assign readout_debug_o[25 +: 4] = bit_counter;
endmodule
