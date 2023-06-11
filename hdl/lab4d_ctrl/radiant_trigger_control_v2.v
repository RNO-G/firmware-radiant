`timescale 1ns / 1ps
// RADIANT trigger control.
// Author: PSA (allison.122@osu.edu)
`include "lab4.vh"
`include "radiant_debug.vh"
// This is an utter disaster of a hack, because I just have to get things working.
// v1 added lookback, but not nearly enough. 


// The trigger control splits up the LAB4 into 4 banks of 8 windows each.
// Due to LAB4 phase stuff the top address bit toggles each time, so the
// banks are
// 0,16,1,17,2,18,3,19
// 4,20,5,21,6,22,7,23
// etc.
// The number of banks,etc. are hardcoded here, as the logic needs
// to totally change if you change it.
// NUM_WR specifies the number of LAB4 address ports to generate
// (doesn't have to match the number of LAB4s).
//
// WR_DELAY adds clock cycles to the WR *output* (period, no matter what - they're
// just free-running shift registers). This can be done to delay WR to compensate for
// an external WR delay, for instance.
//
// (note that this only one way to deal with it: you could also just adjust the sys_clk_div4_flag_i
//  back an equivalent amount to compensate for it, adding yet more offset to the syncOffset field
//  in the PHAB automatch. Either or. I'm doing it this way for now)
module radiant_trigger_control_v2 #(parameter NUM_WR=4, parameter WR_DELAY=0, parameter WR_MAX_VARIABLE_DELAY = 15)(
		input clk_i,
		input sys_clk_i,
		input sys_clk_div4_flag_i,
		input sync_i,
		
		input [`LAB4_WR_WIDTH-1:0] reset_wr_i,
		
		input start_i,
		input stop_i,
		output ready_o,
		// this is just ready in sysclk
		output running_o,
		// Bit of status.
		output [1:0] current_bank_o,
		// Configuration interface
		input rst_i,
		input [2:0] post_trigger_i,
		input post_trigger_wr_i,
		input [1:0] trigger_repeat_i,
		input trigger_repeat_wr_i,
		
		// Triggering interface
		input trigger_i,
		input force_trigger_i,

        output event_o,

		// Trigger FIFO read interface
		output trigger_empty_o,
		input trigger_rd_i,
		output [`LAB4_WR_WIDTH-1:0] trigger_address_o,
		output trigger_last_o,
		input trigger_clear_i,
        // ditch this, put a fixed ILA in here, we're in Vivado now		
		output [15:0] trigger_debug_o,
		
		//WR interface, controls the window and bank information being sent to the LAB4Ds
		input [4*NUM_WR-1:0] WR_variable_delay,
		output [`LAB4_WR_WIDTH*NUM_WR-1:0] WR
    );
    
    localparam DEBUG = `LAB_TRIGGER_DEBUG;
    
    // do NOT change these
    localparam NUM_BANK_BITS = 2;
    localparam NUM_WINDOW_BITS = 3;
    // promote this to a parameter so it's compilation/simulation visible
    localparam LAB4_WR_WIDTH = `LAB4_WR_WIDTH;
        
	reg pre_sys_clk_div4_flag = 0;
	reg [1:0] sys_clk_counter = {2{1'b0}};

	reg dual_bank_mode = 0;
	reg dual_bank_trigger_complete = 0;
	reg do_repeat = 0;

	reg [NUM_BANK_BITS-1:0] bank = {NUM_BANK_BITS{1'b0}};
	// This is the input to the FDs which handle the bank stuff.
	// This is functionally an adder: in dual bank mode, we increment by two (flop top bit)
	// In single bank mode we only flop bank 1 if bank 0 is 1, or:
	// B1 B0 Q
	// 0  0  0
	// 0  1  1
	// 1  0  1
	// 1  1  0
	// which is an xor
    wire next_bank_wr3 = (dual_bank_mode) ? ~bank[1] : bank[0] ^ bank[1];
    wire next_bank_wr2 = ~bank[0];
    // This is split.
	wire enable_next_bank_wr3;
	wire enable_next_bank_wr2;
	
	reg [NUM_BANK_BITS-1:0] bank_full_counter = {NUM_BANK_BITS{1'b0}};
	
	reg [NUM_WINDOW_BITS-1:0] window = {NUM_WINDOW_BITS{1'b0}};
	wire [NUM_WINDOW_BITS:0] window_plus_one = window + 1;
	wire enable_next_window;
	
	reg [NUM_WINDOW_BITS-1:0] post_trigger_counter = {NUM_WINDOW_BITS{1'b0}};
	reg [NUM_WINDOW_BITS-1:0] post_trigger_limit = {NUM_WINDOW_BITS{1'b0}};
	
	
	reg reset_post_trigger = 0;
	
	reg [LAB4_WR_WIDTH-1:0] trigger_address = {LAB4_WR_WIDTH{1'b0}};	
	reg triggering = 0;
	reg trigger_write = 0;
	
	reg start_seen = 0;
	wire start_sysclk;
	reg stop_seen = 0;
	wire stop_sysclk;

	reg enabled_sysclk = 0;

	wire force_trigger_sysclk;	
	wire trigger_clear_sysclk;
	wire post_trigger_wr_sysclk;
	wire trigger_repeat_wr_sysclk;
	
	flag_sync u_start_sync(.in_clkA(start_i),.clkA(clk_i),.out_clkB(start_sysclk),.clkB(sys_clk_i));
	flag_sync u_stop_sync(.in_clkA(stop_i),.clkA(clk_i),.out_clkB(stop_sysclk),.clkB(sys_clk_i));
	flag_sync u_force_sync(.in_clkA(force_trigger_i),.clkA(clk_i),.out_clkB(force_trigger_sysclk),.clkB(sys_clk_i));
	flag_sync u_clear_sync(.in_clkA(trigger_clear_i),.clkA(clk_i),.out_clkB(trigger_clear_sysclk),.clkB(sys_clk_i));
	signal_sync u_enabled_sync(.in_clkA(enabled_sysclk),.clkA(sys_clk_i),.out_clkB(ready_o),.clkB(clk_i));
	
	flag_sync u_post_trigger_sync(.in_clkA(post_trigger_wr_i),.clkA(clk_i),.out_clkB(post_trigger_wr_sysclk),.clkB(sys_clk_i));
	flag_sync u_trigger_repeat_sync(.in_clkA(trigger_repeat_wr_i),.clkA(clk_i),.out_clkB(trigger_repeat_wr_sysclk),.clkB(sys_clk_i));
	reg enabled_sysclk_reg = 0;
	reg update_bank_sysclk = 0;
	wire update_bank;
	reg [NUM_BANK_BITS-1:0] cur_bank = {NUM_BANK_BITS{1'b0}};
	flag_sync u_update_bank(.in_clkA(update_bank_sysclk),.clkA(sys_clk_i),.out_clkB(update_bank),.clkB(clk_i));
	
	always @(posedge clk_i) begin
		if (update_bank) cur_bank <= bank;
	end
	
	// This is used A TON, so pull it out for readability.
	// This indicates when the trigger completes (i.e. our trigger delay is done).
	wire trigger_is_done = (post_trigger_counter == post_trigger_limit && sys_clk_div4_flag_i);
	// This is also for readability. Indicates the trigger will be done on the next cycle.
	wire trigger_will_be_done = (post_trigger_counter == post_trigger_limit && pre_sys_clk_div4_flag);
	// Hold whether or not we're about to repeat.
	reg trigger_will_repeat = 0;
	
	// okay. so. here's our hack.
	// In dual bank mode, we switch to the new bank *right away*. This gives us the *maximum*
	// lookback time with 2 windows. We still write the trigger in exactly as before, 8 clocks later.
	// But in that time, the bank has *swapped*, so... invert it.
	wire bank1_bit_hack = (dual_bank_mode) ? bank[1] ^ dual_bank_trigger_complete : bank[1];
	
	always @(posedge sys_clk_i) begin
		if (sys_clk_div4_flag_i) sys_clk_counter <= {2{1'b0}};
		else sys_clk_counter <= sys_clk_counter + 1;
	
		// clk div4_flag counter pre_div4_flag
		// 0   1         x       0
		// 1   0         0       0
		// 2   0         1       0
		// 3   0         2       1
		// 4   1         3       0
		// 5   0         0       0
	
		// flag when div4_flag is *about* to go
		pre_sys_clk_div4_flag <= (sys_clk_counter == 2'b01);
	
		enabled_sysclk_reg <= enabled_sysclk;
		// this goes high every time the bank changes or we start up.
		// OR is a rising edge detection on enabled_sysclk
		update_bank_sysclk <= (enable_next_bank_wr3) || (!enabled_sysclk_reg && enabled_sysclk);
	
		if (start_sysclk) start_seen <= 1;
		else if (enabled_sysclk) start_seen <= 0;
		
		if (stop_sysclk) stop_seen <= 1;
		else if (!enabled_sysclk) stop_seen <= 0;
		
		// Start up only when we're exiting sync=1. We then start with write address 0.
		// End at a boundary.
		if (start_seen && sys_clk_div4_flag_i && sync_i) enabled_sysclk <= 1;
		else if (stop_seen && sys_clk_div4_flag_i) enabled_sysclk <= 0;

		// Move forward a window at each sys_clk_div4_flag. Reset if not enabled.
		if (enabled_sysclk) begin
			if (sys_clk_div4_flag_i) window <= window + 1;
		end
			else window <= {reset_wr_i[4],reset_wr_i[1:0]};		    // this needs to be altered to allow for BIST
		
		// If we get a trigger, set to triggering state. Once we hit the post-trigger limit, exit that state.
		if (trigger_i || force_trigger_sysclk) triggering <= 1;
		else if (trigger_is_done && !do_repeat) triggering <= 0;

		// Capture address when we transition. Trigger address indicates LAST window.
		// This crap is hardcoded.
		if (triggering && trigger_is_done) 
			trigger_address <= {window[0],bank1_bit_hack,bank[0],window[2:1]};

		// Move to next buffer when we hit post trigger limit.
		// This actually is just for tracking.
		if (enable_next_bank_wr3) bank[1] <= next_bank_wr3;
		else if (!enabled_sysclk) bank[1] <= reset_wr_i[3]; // this needs to be altered to allow for BIST

		if (enable_next_bank_wr2) bank[0] <= next_bank_wr2;
		else if (!enabled_sysclk) bank[0] <= reset_wr_i[2]; // this needs to be altered to allow for BIST
		
		// Write the trigger into the FIFO when we hit the post trigger limit. trigger_write=1 and trigger_address latch happen at same time.
		if (triggering && trigger_is_done) trigger_write <= 1;
		else trigger_write <= 0;

        if (triggering && trigger_is_done) trigger_will_repeat <= dual_bank_mode && !dual_bank_trigger_complete;
        else trigger_will_repeat <= 0;

		// Count the post trigger counter when triggering, but not when repeating. (do_repeat essentially acts as another trigger)
		if (!triggering || do_repeat) post_trigger_counter <= {NUM_WINDOW_BITS{1'b0}};
		else if (sys_clk_div4_flag_i) post_trigger_counter <= post_trigger_counter + 1;

		// Grab the limit from the control interface.
		if (post_trigger_wr_sysclk || reset_post_trigger) post_trigger_limit <= post_trigger_i;		
		else if (triggering && do_repeat) post_trigger_limit <= 7;

		if (trigger_repeat_wr_sysclk)
		  dual_bank_mode <= trigger_repeat_i[0];

		// this indicates that the trigger should be repeated (continued readout)
		// The logic here ensures that do_repeat is definitely high when triggering goes high.
		if ((triggering || trigger_i || force_trigger_sysclk) && trigger_will_be_done && (dual_bank_mode && !dual_bank_trigger_complete))
			do_repeat <= 1;
		else
			do_repeat <= 0;

		if (triggering && (post_trigger_counter == post_trigger_limit) && pre_sys_clk_div4_flag && (!dual_bank_mode || (dual_bank_mode && dual_bank_trigger_complete)))
			reset_post_trigger <= 1;
		else
			reset_post_trigger <= 0;

		if (triggering && (post_trigger_counter == post_trigger_limit) && sys_clk_div4_flag_i && dual_bank_mode && !dual_bank_trigger_complete)
			dual_bank_trigger_complete <= 1;
		else if (!triggering)
			dual_bank_trigger_complete <= 0;
				
		// DO SOMETHING SMART HERE TO DISABLE TRIGGERS!!
		if (trigger_write) bank_full_counter <= bank_full_counter + 1;
		else if (trigger_clear_sysclk) bank_full_counter <= bank_full_counter - 1;

	end
    // The top bit flags if the trigger's actually complete (done through the repeat).
    // trigger_write goes high the cycle after triggering would go low,
    // so we need to catch do_repeat's state right then.
	trigger_fifo u_fifo(.din({!trigger_will_repeat, trigger_address}),.dout({trigger_last_o, trigger_address_o}),.rd_clk(clk_i),.wr_clk(sys_clk_i),
							  .wr_en(trigger_write),.rd_en(trigger_rd_i),.empty(trigger_empty_o),
							  .rst(rst_i));

    // ULTRA MEGA HACK
    // WE SWITCH TO NEW BANK RIGHT AWAY IN DUAL BANK MODE, BUT WRITE THE OLD BANK
	assign enable_next_bank_wr3 = (triggering && post_trigger_counter == post_trigger_limit && sys_clk_div4_flag_i) && (!dual_bank_mode || (dual_bank_mode && !dual_bank_trigger_complete));
    assign enable_next_bank_wr2 = (!dual_bank_mode && triggering && post_trigger_counter == post_trigger_limit && sys_clk_div4_flag_i) ||
                                  (dual_bank_mode && sys_clk_div4_flag_i && window[2:0] == 3'b111);
	assign enable_next_window = (sys_clk_div4_flag_i);

	// The WR[4:0] map here is to accomodate LAB4 weirdnesss.
	// WR[4] has to toggle every SST, so that makes *it* the LSB.
	// In the PicoBlaze code, what we have to do is mask off
	// 10 011 = 13 (e.g. address = address  0x13)
	// and then do
	// convert address
	// convert address | 0x10
	// convert address+1
	// convert address+1 | 0x10
	// convert address+2
	// convert address+2 | 0x10
	// convert address+3
	// convert address+3 | 0x10
	//
	// WR_DELAY allows for forward-delaying WR to compensate for an external WR delay.
	// This is an utter garbage way to do this, mind you, I should rethink this whole thing:
	// but realistically the window's wide enough that we can adjust the post trigger counter for proper
	// capture pretty easily.
	wire [WR_DELAY+WR_MAX_VARIABLE_DELAY*4:0] enable_next_window_delay;
	wire [WR_DELAY+WR_MAX_VARIABLE_DELAY*4:0] enable_next_bank_delay_wr3;
	wire [WR_DELAY+WR_MAX_VARIABLE_DELAY*4:0] enable_next_bank_delay_wr2;
	wire [WR_DELAY+WR_MAX_VARIABLE_DELAY*4:0] enabled_sysclk_delay;
	wire [NUM_WINDOW_BITS:0] window_plus_one_delay[WR_DELAY+WR_MAX_VARIABLE_DELAY*4:0];

	wire [NUM_BANK_BITS:0] next_bank_wr3_delay[WR_DELAY+WR_MAX_VARIABLE_DELAY*4:0];
    wire [NUM_BANK_BITS:0] next_bank_wr2_delay[WR_DELAY+WR_MAX_VARIABLE_DELAY*4:0];

	// connect the tails
	assign enable_next_window_delay[0] = enable_next_window;
	assign enable_next_bank_delay_wr3[0] = enable_next_bank_wr3;
	assign enable_next_bank_delay_wr2[0] = enable_next_bank_wr2;
	assign enabled_sysclk_delay[0] = enabled_sysclk;
	assign window_plus_one_delay[0] = window_plus_one;
	assign next_bank_wr3_delay[0] = next_bank_wr3;
	assign next_bank_wr2_delay[0] = next_bank_wr2;
	
	generate
		genvar i,j;
        // world's goofiest defined shift register
        for (j=0;j<(WR_DELAY+WR_MAX_VARIABLE_DELAY*4);j=j+1) begin : DL
            reg enable_next_window_reg = 0;
            assign enable_next_window_delay[j+1] = enable_next_window_reg;
            reg enable_next_bank_wr3_reg = 0;
            assign enable_next_bank_delay_wr3[j+1] = enable_next_bank_wr3_reg;
            reg enable_next_bank_wr2_reg = 0;
            assign enable_next_bank_delay_wr2[j+1] = enable_next_bank_wr2_reg;

            reg enabled_sysclk_reg = 0;
            assign enabled_sysclk_delay[j+1] = enabled_sysclk_reg;
            
            reg [NUM_WINDOW_BITS:0] window_plus_one_reg = {NUM_WINDOW_BITS+1{1'b0}};
            assign window_plus_one_delay[j+1] = window_plus_one_reg;
            
            reg next_bank_wr3_reg = 0;
            reg next_bank_wr2_reg = 0;
            assign next_bank_wr3_delay[j+1] = next_bank_wr3_reg;
            assign next_bank_wr2_delay[j+1] = next_bank_wr2_reg;

            always @(posedge sys_clk_i) begin : LOGIC
                enable_next_window_reg <= enable_next_window_delay[j];
                enable_next_bank_wr3_reg <= enable_next_bank_delay_wr3[j];
                enable_next_bank_wr2_reg <= enable_next_bank_delay_wr2[j];
                
                enabled_sysclk_reg <= enabled_sysclk_delay[j];
                
                window_plus_one_reg <= window_plus_one_delay[j];
                next_bank_wr3_reg <= next_bank_wr3_delay[j];
                next_bank_wr2_reg <= next_bank_wr2_delay[j];
            end
        end
                
		for (i=0;i<NUM_WR;i=i+1) begin : LAB
			(* IOB = "TRUE" *)
			wire [3:0] i_variable_delay = i; //WR_variable_delay[i*4+3:i*4];
			
			FDRE u_wr4(.D(window_plus_one_delay[WR_DELAY+4*i_variable_delay][0]),
						  .CE(enable_next_window_delay[WR_DELAY+4*i_variable_delay]),
						  .C(sys_clk_i),
						  .R(!enabled_sysclk_delay[WR_DELAY+4*i_variable_delay]),
						  .Q(WR[5*i+4]));
			FDRE u_wr3(.D(next_bank_wr3_delay[WR_DELAY+4*i_variable_delay]),
						  .CE(enable_next_bank_delay_wr3[WR_DELAY+4*i_variable_delay]),
						  .C(sys_clk_i),
						  .R(!enabled_sysclk_delay[WR_DELAY+4*i_variable_delay]),
						  .Q(WR[5*i+3]));
			FDRE u_wr2(.D(next_bank_wr2_delay[WR_DELAY+4*i_variable_delay]),
						  .CE(enable_next_bank_delay_wr2[WR_DELAY+4*i_variable_delay]),
						  .C(sys_clk_i),
						  .R(!enabled_sysclk_delay[WR_DELAY+4*i_variable_delay]),
						  .Q(WR[5*i+2]));
			FDRE u_wr1(.D(window_plus_one_delay[WR_DELAY+4*i_variable_delay][2]),
						  .CE(enable_next_window_delay[WR_DELAY+4*i_variable_delay]),
						  .C(sys_clk_i),
						  .R(!enabled_sysclk_delay[WR_DELAY+4*i_variable_delay]),
						  .Q(WR[5*i+1]));
			FDRE u_wr0(.D(window_plus_one_delay[WR_DELAY+4*i_variable_delay][1]),
						  .CE(enable_next_window_delay[WR_DELAY+4*i_variable_delay]),
						  .C(sys_clk_i),
						  .R(!enabled_sysclk_delay[WR_DELAY+4*i_variable_delay]),
						  .Q(WR[5*i+0]));
		end
		if (DEBUG == "TRUE") begin : DBG
		  wire [5:0] dbg_window = {window[0],bank,window[2:1]};
		  trigger_control_ila u_ila(.clk(sys_clk_i),
		                            .probe0( dbg_window ),
		                            .probe1(trigger_i),
		                            .probe2(force_trigger_sysclk),
		                            .probe3(trigger_write),
		                            .probe4(triggering),
		                            .probe5(rst_i),
		                            .probe6(enabled_sysclk),
		                            .probe7(start_sysclk),
		                            .probe8(stop_sysclk),
		                            .probe9(enable_next_bank_wr3),
		                            .probe10(trigger_will_repeat));
        end
	endgenerate
	assign trigger_debug_o[5:0] = {window[0],bank,window[2:1]};
	assign trigger_debug_o[6] = trigger_i;
	assign trigger_debug_o[7] = force_trigger_sysclk;
	assign trigger_debug_o[8] = trigger_write;
	assign trigger_debug_o[9] = triggering;
	assign trigger_debug_o[10] = rst_i;
	assign trigger_debug_o[11] = enabled_sysclk;
	assign trigger_debug_o[12] = start_sysclk;
	assign trigger_debug_o[13] = stop_sysclk;
	assign trigger_debug_o[14] = enable_next_bank_wr3;
	assign trigger_debug_o[15] = trigger_will_repeat;

    assign running_o = enabled_sysclk;

	assign current_bank_o = cur_bank;
	assign event_o = trigger_i || force_trigger_sysclk;
endmodule
