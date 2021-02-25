`timescale 1ns / 1ps
`include "radiant_debug.vh"
////////////////////////////////////////////////////////////////////////////////
// This file is a part of the Antarctic Impulsive Transient Antenna (ANITA)
// project, a collaborative scientific effort between multiple institutions. For
// more information, contact Peter Gorham (gorham@phys.hawaii.edu).
//
// All rights reserved.
//
// Author: Patrick Allison, Ohio State University (allison.122@osu.edu)
// Author:
// Author:
////////////////////////////////////////////////////////////////////////////////
//
// Note: right now the only way to deal with a long SRCLK -> DOE delay
// is to slow down the whole thing via the prescale register. This is incredibly
// dumb, since I know that the sequence works up to 50 MHz at least.
// So what I want to do next is to add a variable delay to the whole setup
// where I delay the input bit and also delay shreg_ce through SRLs, but
// then I can adjust the relative delay of the capture and the data.
module par_lab4d_data_shift_register #( parameter NUM_LAB4 = 12,
					parameter NUM_SRCLK = 12,
					parameter [NUM_SRCLK-1:0] SRCLK_POLARITY = {NUM_SRCLK{1'b0}},
					parameter NUM_SS_INCR = 12,
					parameter LAB4_BITS = 12,
					parameter DEBUG = `LAB4D_DATA_REGISTER_DEBUG
				       
)(
		input sys_clk_i,
		input wclk_i,
		input readout_i,
		input readout_rst_i,
		output done_o,
		output [LAB4_BITS*NUM_LAB4-1:0] dat_o,
		output dat_wr_o,
		input [3:0] delay_i,
		input [3:0] prescale_i,
		output srclk_o,
		output ss_incr_o,
		output [6:0] sample_counter_o,
		output [3:0] bit_counter_o,
		input [NUM_LAB4-1:0] DOE,
		output [NUM_SRCLK-1:0] SRCLK,
		output [NUM_SS_INCR-1:0] SS_INCR,
		output [15:0] sample_debug
    );

    

	// Readout shift register in the LAB4D:
	// Basically it's just clock out 12 bits and have SS_INCR low in the last clock cycle.
	// However, there's a quirk: REGCLR, and presumedly power-on, both have the SS_INCR
	// negative-edge detection screwed up since it puts them both in the 0 state.
	// So we need to clock in SS_INCR at reset.

	//% Number of bits clocked out each sample.
	localparam [3:0] MAX_BITS = LAB4_BITS;
	//% Number of clocks output at reset.
	localparam [3:0] RESET_CLOCKS = 2;
	
	//% Current number of bits in sample.
	reg [3:0] bit_counter = {4{1'b0}};
	//% Current number of samples.
	reg [6:0] sample_counter = {7{1'b0}};
	//% Next counter value (used for terminal count detection)
	wire [7:0] sample_counter_plus_one = sample_counter + 1;

	//% The prescale counter slows down everything internal to this block.
	reg [3:0] prescale_counter = {4{1'b0}};

	//% Goes high when readout_i goes high, and then goes low when state exits IDLE.
	reg readout_request_seen = 0;
	
	//% Flag to capture the shift register data.
	reg dat_wr = 0;
	
	//% Indicates that the first bit is being loaded, which means we don't increment the bit counter.
	reg loading_first_bit = 0;
	
	//% Debug equal to the SRCLK output
	reg dbg_srclk = 0;
	//% Debug equal to the SS_INCR output.
	reg dbg_ss_incr = 1;
	
	localparam FSM_BITS=3;
	localparam [FSM_BITS-1:0] RESET_LOW = 0;
	localparam [FSM_BITS-1:0] RESET_HIGH = 1;
	localparam [FSM_BITS-1:0] RESET_LOW_EXIT = 2;
	localparam [FSM_BITS-1:0] IDLE = 3;
	localparam [FSM_BITS-1:0] SHIFT_HIGH = 4;
	localparam [FSM_BITS-1:0] SHIFT_LOW = 5;
	localparam [FSM_BITS-1:0] SHIFT_LOW_EXIT = 6;
	localparam [FSM_BITS-1:0] DONE = 7;
	//% FSM state.
	reg [FSM_BITS-1:0] state = RESET_LOW;
		
	//% Overall clock enable for this entire block.
	wire ce = (prescale_counter == prescale_i);

	//% Clock enable for SRCLK.
	wire srclk_ce = ce;
	//% Data input for SRCLK. Goes high entering RESET_HIGH or SHIFT_HIGH.
	wire srclk_d = (state == SHIFT_LOW) || (state == IDLE && readout_request_seen) || (state == RESET_LOW);
	
	//% Clock enable for SS_INCR.
	wire ss_incr_ce = (ce) && ((state == IDLE && readout_request_seen) || (state == SHIFT_LOW) || (state == SHIFT_LOW_EXIT));
	//% Data input for SS_INCR. Goes low at first entry to SHIFT_HIGH, and also at next-to-last bit entry into SHIFT_HIGH, unless last sample.
	wire ss_incr_d = !(state == IDLE && readout_request_seen) && !(state == SHIFT_LOW && bit_counter == MAX_BITS-2 && !sample_counter_plus_one[7]);
	
	//% Clock enable for the shift register.
	wire shreg_ce = ce && (state == SHIFT_LOW);

	always @(posedge sys_clk_i) begin	
		// Prescale counter runs all the time.
		if (readout_rst_i) prescale_counter <= {4{1'b0}};
		else if (prescale_counter == prescale_i) prescale_counter <= {4{1'b0}};
		else prescale_counter <= prescale_counter + 1;		
	
		// Bit counter increments in SHIFT_LOW or RESET_HIGH.
		if (readout_rst_i) bit_counter <= {4{1'b0}};
		else if (state == IDLE || loading_first_bit) bit_counter <= {4{1'b0}};
		else if (ce) begin
			if (state == SHIFT_LOW) begin
				if (bit_counter == MAX_BITS-1) bit_counter <= {4{1'b0}};
				else bit_counter <= bit_counter + 1;
			end else if (state == RESET_HIGH) begin
				bit_counter <= bit_counter + 1;
			end
		end
		
		// Sample counter increments in SHIFT_LOW when bit_counter is at maximum.
		if (readout_rst_i) sample_counter <= {7{1'b0}};
		else if (ce) begin
		  if (state == IDLE || state == DONE) sample_counter <= {7{1'b0}};
		  else if (state == SHIFT_LOW && bit_counter == MAX_BITS-1) sample_counter <= sample_counter_plus_one;
        end

		// Readout request seen goes high when seeing readout_i, and cleared at SHIFT_HIGH.
		if (readout_rst_i) readout_request_seen <= 0;
		else if (readout_i) readout_request_seen <= 1;
		else if (state == SHIFT_HIGH) readout_request_seen <= 0;
		
		if (readout_rst_i) state <= RESET_LOW;
		else if (ce) begin
			case (state)
				// Move to RESET_HIGH
				RESET_LOW: state <= RESET_HIGH;
				// If we've counted the max number of clocks, move to exit. Otherwise keep going.
				RESET_HIGH: if (bit_counter == RESET_CLOCKS-1) state <= RESET_LOW_EXIT;
								else state <= RESET_LOW;
				// At exit, go to IDLE.
				RESET_LOW_EXIT: state <= IDLE;
				// At idle, if we're waiting for a readout_i, move to SHIFT_HIGH.
				// Our problem here is the first bit. 
				IDLE: if (readout_request_seen) state <= SHIFT_HIGH;
				// At SHIFT_HIGH, if we've clocked in 12 bits (bit_counter increments in SHIFT_LOW)
				// and we've clocked in 127 samples (127+1 sets sample_counter_plus_one[7]) move to SHIFT_LOW_EXIT.
				// Otherwise keep going.
				SHIFT_HIGH: if (bit_counter == MAX_BITS-1 && sample_counter_plus_one[7]) state <= SHIFT_LOW_EXIT;
								else state <= SHIFT_LOW;
				// Move to SHIFT_HIGH.
				SHIFT_LOW: state <= SHIFT_HIGH;
				// We're done, so exit.
				SHIFT_LOW_EXIT: state <= DONE;
				// Back to IDLE.
				DONE: state <= IDLE;
			endcase
		end
		
		if (ce) begin
			if (state == IDLE && readout_request_seen) loading_first_bit <= 1;
			else if (state == SHIFT_LOW) loading_first_bit <= 0;
		end
		
		// dat_wr can go upon entry to SHIFT_LOW when we've clocked in 12 bits.
		dat_wr <= ce && (state == SHIFT_HIGH) && (bit_counter == MAX_BITS-1);
		
		if (srclk_ce) dbg_srclk <= srclk_d;
		if (ss_incr_ce) dbg_ss_incr <= ss_incr_d;
	end

    wire [NUM_LAB4-1:0] shreg_msbs;
	generate
		genvar i,j;
		for (i=0;i<NUM_LAB4;i=i+1) begin : LAB
			reg [LAB4_BITS-2:0] data_shreg = {LAB4_BITS-1{1'b0}};
			// Always have DOE clock in constantly. It just only gets added to the shreg occasionally.
			(* IOB = "TRUE" *)
			FDRE #(.INIT(1'b0)) u_shreg(.D(DOE[i]),.CE(1'b1),.C(sys_clk_i),.R(1'b0),.Q(shreg_msbs[i]));
			always @(posedge sys_clk_i) begin : SHREG
				if (shreg_ce) data_shreg <= {shreg_msbs[i],data_shreg[10:1]};
			end
			// I don't think this is entirely right. Need to think about this. Maybe just output the data shreg
			// and delay off the wr or something.
			assign dat_o[12*i +: 12] = {shreg_msbs[i], data_shreg};		   
		end
	        for (i=0;i<NUM_SS_INCR;i=i+1) begin : SS
			(* IOB = "TRUE" *)
			FDSE #(.INIT(1'b1)) u_ss_incr(.D(ss_incr_d),.CE(ss_incr_ce),.C(sys_clk_i),.S(readout_rst_i),.Q(SS_INCR[i]));
		end
	        for (i=0;i<NUM_SRCLK;i=i+1) begin : CL
			if (SRCLK_POLARITY[i] == 0) begin : POS_POL
				(* IOB = "TRUE" *)
				FDRE #(.INIT(1'b0)) u_srclk(.D(srclk_d),.CE(srclk_ce),.C(sys_clk_i),.R(1'b0),.Q(SRCLK[i]));
			end else begin : NEG_POL
				(* IOB = "TRUE" *)
				FDRE #(.INIT(1'b1)) u_srclk(.D(~srclk_d),.CE(srclk_ce),.C(sys_clk_i),.R(1'b0),.Q(SRCLK[i]));
			end	   
		end		
		if (DEBUG == "TRUE") begin : DBG
            // make VIO 8 bits period
            wire [7:0] vio_sel;
            localparam L4NB = $clog2(NUM_LAB4);
            // now find the max this can represent
            localparam L4NBMAX = (1<<L4NB);
            // now create an array that big
            wire [LAB4_BITS-1:0] debug_lab_arr[L4NBMAX-1:0];  
            wire [L4NB-1:0] sel_dbg = vio_sel[0 +: L4NB];            
            reg [LAB4_BITS-1:0] debug_dat = {LAB4_BITS{1'b0}};
            reg debug_ss_incr = 0;
            reg debug_wr = 0;
            reg debug_srclk = 0;
            for (j=0;j<L4NBMAX;j=j+1) begin : DBGARR
                if (j < NUM_LAB4) begin : REAL
                    assign debug_lab_arr[j] = dat_o[12*j +: 12];
                end else begin : DUM
                    // simplify the decode: if L4NB = 4 (so L4NBMAX = 16),
                    // then this is j-8. So if we only have 12,
                    // 13 would map to 5, 14 would map to 6, 15 to 7.
                    assign debug_lab_arr[j] = debug_lab_arr[j - (1<<(L4NB-1))];
                end
            end
            always @(posedge sys_clk_i) begin : MUX
                debug_dat <= debug_lab_arr[sel_dbg];
                // and delay the control signals to match up to the delayed data.
                debug_ss_incr <= ss_incr_o;
                debug_wr <= dat_wr_o;
                debug_srclk <= srclk_o;
            end
            lab4_data_debug_vio u_dbg_vio(.clk(sys_clk_i),.probe_out0(vio_sel));
            // the 'shreg_msbs' are the direct copies of all DOE inputs
            lab4_data_debug_ila u_dbg_ila(.clk(sys_clk_i),.probe0(debug_srclk),.probe1(debug_ss_incr),.probe2(debug_wr),.probe3(debug_dat),.probe4(shreg_msbs));
		end		
	endgenerate
	
    
    
		
	assign sample_counter_o = sample_counter;
	assign bit_counter_o = bit_counter;
	assign srclk_o = dbg_srclk;
	assign ss_incr_o = dbg_ss_incr;
	assign dat_wr_o = dat_wr;
	assign done_o = (state == DONE);
	assign sample_debug = {16{1'b0}};
endmodule
