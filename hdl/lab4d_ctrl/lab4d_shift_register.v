`timescale 1ns / 1ps
`include "radiant_debug.vh"
`include "lab4.vh"
module lab4d_shift_register (
        clk_i,
        go_i,
        dat_i,
        sel_i,
        prescale_i,
        busy_o,
        dbg_sin_o,
        dbg_sclk_o,
        dbg_pclk_o,
        SIN,
        SCLK,
        PCLK);
    parameter DEBUG = `LAB4D_SHIFT_REGISTER_DEBUG;
    parameter NUM_LABS = 24;
    parameter NUM_SCLK = 2;
    parameter NUM_SEL_BITS = 5;
    
    input clk_i;
    input go_i;
    input [23:0] dat_i;
    input [NUM_SEL_BITS-1:0] sel_i;
    input [7:0] prescale_i;
    output busy_o;
    output dbg_sin_o;
    output dbg_sclk_o;
    output dbg_pclk_o;
    output [NUM_LABS-1:0] SIN;
    output [NUM_SCLK-1:0] SCLK;
    output [NUM_LABS-1:0] PCLK;
                
	localparam NUM_BITS = 24;

	reg [7:0] prescale_counter = {8{1'b0}};
	reg [4:0] bit_counter = {5{1'b0}};
	reg [23:0] shift_reg = {24{1'b0}};
	reg sclk_reg = 0;
	reg pclk_reg = 0;
	(* KEEP = "TRUE" *)
	reg [NUM_SCLK-1:0] sclk_copy = {NUM_SCLK{1'b0}};
	(* KEEP = "TRUE" *)
	reg [NUM_LABS-1:0] pclk_copy = {NUM_LABS{1'b0}};
	(* KEEP = "TRUE" *)
	reg [NUM_LABS-1:0] sin_copy = {NUM_LABS{1'b0}};
	
	wire [NUM_LABS-1:0] lab_is_selected;
	
	localparam FSM_BITS = 4;
	localparam [FSM_BITS-1:0] IDLE = 0;
	localparam [FSM_BITS-1:0] LOAD = 1;
	localparam [FSM_BITS-1:0] SCLK_HIGH = 2;
	localparam [FSM_BITS-1:0] SCLK_LOW = 3;
	localparam [FSM_BITS-1:0] UPDATE_LOW = 4;
	localparam [FSM_BITS-1:0] UPDATE_HIGH = 5;
	localparam [FSM_BITS-1:0] PCLK_LOW = 6;
	localparam [FSM_BITS-1:0] SIN_HIGH = 7;
	localparam [FSM_BITS-1:0] PCLK_HIGH = 8;
	localparam [FSM_BITS-1:0] DONE = 9;
	reg [FSM_BITS-1:0] state = IDLE;
	
	always @(posedge clk_i) begin
		if (state == LOAD) shift_reg <= dat_i;
		else if (state == SCLK_HIGH && prescale_counter == prescale_i) shift_reg <= {shift_reg[22:0],1'b0};
		else if (state == PCLK_LOW && prescale_counter == prescale_i) shift_reg[23] <= 1;
		else if (state == DONE) shift_reg[23] <= 0;
		
		if (state == LOAD && prescale_counter == prescale_i) sclk_reg <= 1;
		else if (state == SCLK_LOW && prescale_counter == prescale_i && bit_counter != NUM_BITS) sclk_reg <= 1;
		else if (state == SCLK_HIGH && prescale_counter == prescale_i) sclk_reg <= 0;
		else if (state == IDLE) sclk_reg <= 0;
		
		if (state == UPDATE_LOW && prescale_counter == prescale_i) pclk_reg <= 1;
		else if (state == UPDATE_HIGH && prescale_counter == prescale_i) pclk_reg <= 0;
		else if (state == SIN_HIGH && prescale_counter == prescale_i) pclk_reg <= 1;
		else if (state == PCLK_HIGH && prescale_counter == prescale_i) pclk_reg <= 0;
		
		if (state == IDLE || prescale_counter == prescale_i) prescale_counter <= {8{1'b0}};
		else prescale_counter <= prescale_counter + 1;
		
		if (state == IDLE) bit_counter <= {5{1'b0}};
		if (state == SCLK_HIGH && prescale_counter == prescale_i) bit_counter <= bit_counter + 1;

		case (state)
			IDLE: if (go_i) state <= LOAD;
			LOAD: if (prescale_counter == prescale_i) state <= SCLK_HIGH;
			SCLK_HIGH: if (prescale_counter == prescale_i) state <= SCLK_LOW;
			SCLK_LOW: if (prescale_counter == prescale_i) begin
								if (bit_counter == NUM_BITS) state <= UPDATE_LOW;
								else state <= SCLK_HIGH;
						 end
			UPDATE_LOW: if (prescale_counter == prescale_i) state <= UPDATE_HIGH;
			UPDATE_HIGH: if (prescale_counter == prescale_i) state <= PCLK_LOW;
			PCLK_LOW: if (prescale_counter == prescale_i) state <= SIN_HIGH;
			SIN_HIGH: if (prescale_counter == prescale_i) state <= PCLK_HIGH;
			PCLK_HIGH: if (prescale_counter == prescale_i) state <= DONE;
			DONE: state <= IDLE;
		endcase
	end
	generate
		genvar i,j;
		for (i=0;i<NUM_LABS;i=i+1) begin : LABS
			assign lab_is_selected[i] = (sel_i == i) || &sel_i;

            always @(posedge clk_i) begin : DBG
                if (!lab_is_selected[i]) pclk_copy[i] <= 1'b0;
                else pclk_copy[i] <= pclk_reg;
                
                if (!lab_is_selected[i]) sin_copy[i] <= 1'b0;
                else sin_copy[i] <= shift_reg[23];
            end						
			(* IOB = "TRUE" *)
			FDRE u_pclk(.D(pclk_reg),.C(clk_i),.R(!lab_is_selected[i]),.CE(1'b1),.Q(PCLK[i]));
			(* IOB = "TRUE" *)
			FDRE u_sin(.D(shift_reg[23]),.C(clk_i),.R(!lab_is_selected[i]),.CE(1'b1),.Q(SIN[i]));
		end
		for (j=0;j<NUM_SCLK;j=j+1) begin : SCLKS
		    always @(posedge clk_i) begin : DBG
		      if (state == IDLE) sclk_copy[j] <= 1'b0;
		      else sclk_copy[j] <= sclk_reg;
            end
		    (* IOB = "TRUE" *)
			FDRE u_sclk(.D(sclk_reg),.C(clk_i),.R((state==IDLE)),.CE(1'b1),.Q(SCLK[j]));
        end
        if (DEBUG == "TRUE") begin : ILA
            lab4d_shift_register_ila u_ila(.clk(clk_i),.probe0(sin_copy),.probe1(pclk_copy),.probe2(sclk_copy),.probe3(go_i));
        end
	endgenerate


	assign busy_o = (state != IDLE);
endmodule
