`timescale 1ns / 1ps
`include "wishbone.vh"
// Simple WISHBONE interconnect.
// We have 4MB of register space.
// The full space of 24 LAB4Ds is 192 kB, or 48k qwords,
// which fits in 64k of space, so we have buckets of space.
module wbc_intercon(
		input clk_i,
		input rst_i,
		// Masters.
		`WBS_NAMED_PORT(bmc, 32, 20, 4),
		`WBS_NAMED_PORT(spic, 32, 20, 4),
		`WBS_NAMED_PORT(pciec, 32, 20, 4),
		// Slaves. 
		`WBM_NAMED_PORT(rad_id_ctrl, 32, 16, 4),
		`WBM_NAMED_PORT(l4_ctrl, 32, 16, 4),
		`WBM_NAMED_PORT(l4_ram, 32, 16, 4),
		`WBM_NAMED_PORT(trig, 32, 16, 4),
		`WBM_NAMED_PORT(scal, 32, 16, 4),
		output [70:0] debug_o
    );

	localparam [19:0] RAD_ID_CTRL_BASE     = 20'h00000;
	localparam [19:0] RAD_ID_CTRL_MASK     = 20'h8FFFF;	// match x000: 0x00000-0x0FFFF, shadowed at 0x80000-0x8FFFF
	localparam [19:0] L4_CTRL_BASE         = 20'h10000;
	localparam [19:0] L4_CTRL_MASK	       = 20'h8FFFF;	// match x001: 0x10000-0x1FFFF, shadowed at 0x90000-0x9FFFF
	localparam [19:0] L4_RAM_BASE		   = 20'h20000;	
	localparam [19:0] L4_RAM_MASK		   = 20'h8FFFF;	// match x010: 0x20000-0x2FFFF, shadowed at 0xA0000-0xAFFFF
	localparam [19:0] TRIG_BASE			   = 20'h30000;
	localparam [19:0] TRIG_MASK			   = 20'h8FFFF;	// match x011: 0x30000-0x3FFFF, shadowed at 0xB0000-0xBFFFF
	localparam [19:0] SCAL_BASE			   = 20'h40000;
	localparam [19:0] SCAL_MASK			   = 20'hBFFFF;	// match x1xx: 0x40000-0x7FFFF, shadowed at 0xC0000-0xFFFFF

	wire bmc_gnt;
	wire spic_gnt;
	wire pciec_gnt;
	// Simple round robin arbiter for right now. Stolen from asic-world.
//	arbiter u_arbiter(.clk(clk_i),.rst(rst_i),
//							.req0(bmc_cyc_i),.gnt0(bmc_gnt),
//							.req1(spic_cyc_i),.gnt1(spic_gnt),
//							.req2(pciec_cyc_i),.gnt2(pciec_gnt),
//							.req3(wbvio_cyc_i),.gnt3(wbvio_gnt));							
	// Switch to expandable round-robin arbiter
	localparam NUM_MASTERS = 3;
	wire [NUM_MASTERS-1:0] requests;
	wire [NUM_MASTERS-1:0] grants;
	wire [NUM_MASTERS-1:0] strobes;
	wire [NUM_MASTERS-1:0] writes;
	wire [NUM_MASTERS-1:0] acks;
	wire [NUM_MASTERS-1:0] errs;
	wire [NUM_MASTERS-1:0] rtys;
	reg muxed_ack;
	reg muxed_err;
	reg muxed_rty;
	reg [31:0] muxed_dat_i;
	`define MASTER(x, y) \
		assign requests[ y ] = x``_cyc_i; \
		assign x``_gnt = grants[ y ];     \
		assign strobes[ y ] = x``_stb_i;  \
		assign writes[ y ] = x``_we_i;	 \
		assign x``_ack_o = acks[ y ];	    \
		assign x``_err_o = errs[ y ];     \
		assign x``_rty_o = rtys[ y ];     \
		assign x``_dat_o = muxed_dat_i
	
	`MASTER(bmc, 0);
	`MASTER(spic, 1);
	`MASTER(pciec, 2);
	// The multiplexer is harder to code automatically because it needs to be done in a define. 
	
	wishbone_arbiter #(.NUM_MASTERS(NUM_MASTERS)) u_arbiter(.rst_i(rst_i),.clk_i(clk_i),.cyc_i(requests),.gnt_o(grants));
	
	// now the controls are easy, they're a reduction or of bitwise ands
	wire cyc = |(requests & grants);
	wire stb = |(strobes & grants);
	wire we = |(writes & grants);
	// or just a bitwise and for the outputs. Since we're a shared interconnect (not a crossbar), we just repeat the muxed acks.
	assign acks = {NUM_MASTERS{muxed_ack}} & grants;
	assign rtys = {NUM_MASTERS{muxed_rty}} & grants;
	assign errs = {NUM_MASTERS{muxed_err}} & grants;
	
	// The multiplexed addresses, data, and selects are harder.
	reg [19:0] adr;
	reg [31:0] dat_o;
	reg [3:0] sel;
	always @(*) begin
		if (spic_gnt) begin 
			adr <= spic_adr_i;
			dat_o <= spic_dat_i;
			sel <= spic_sel_i;
		end else if (pciec_gnt) begin
			adr <= pciec_adr_i;
			dat_o <= pciec_dat_i;
			sel <= pciec_sel_i;
		end else begin
			adr <= bmc_adr_i;
			dat_o <= bmc_dat_i;
			sel <= bmc_sel_i;
		end
	end

	// Match addresses by masking off all mask bits, and comparing to base.
	`define SLAVE_MAP(prefix, mask, base)						\
		wire sel_``prefix = ((adr & ~ mask ) == base );		\
		assign prefix``_cyc_o = cyc && sel_``prefix ;		\
		assign prefix``_stb_o = stb && sel_``prefix ;		\
		assign prefix``_we_o = we && sel_``prefix;			\
		assign prefix``_adr_o = (adr & mask );					\
		assign prefix``_dat_o = dat_o;							\
		assign prefix``_sel_o = sel

	`SLAVE_MAP( rad_id_ctrl, RAD_ID_CTRL_MASK, RAD_ID_CTRL_BASE );
	`SLAVE_MAP( l4_ctrl, L4_CTRL_MASK, L4_CTRL_BASE );
	`SLAVE_MAP( l4_ram,  L4_RAM_MASK, L4_RAM_BASE );
	`SLAVE_MAP( trig, TRIG_MASK, TRIG_BASE );
	`SLAVE_MAP( scal, SCAL_MASK, SCAL_BASE );
	
	always @(*) begin
		if (sel_l4_ram) begin
			muxed_ack <= l4_ram_ack_i;
			muxed_err <= l4_ram_err_i;
			muxed_rty <= l4_ram_rty_i;
			muxed_dat_i <= l4_ram_dat_i;
		end else if (sel_trig) begin
			muxed_ack <= trig_ack_i;
			muxed_err <= trig_err_i;
			muxed_rty <= trig_rty_i;
			muxed_dat_i <= trig_dat_i;
		end else if (sel_l4_ctrl) begin
			muxed_ack <= l4_ctrl_ack_i;
			muxed_err <= l4_ctrl_err_i;
			muxed_rty <= l4_ctrl_rty_i;
			muxed_dat_i <= l4_ctrl_dat_i;
		end else if (sel_scal) begin
			muxed_ack <= scal_ack_i;
			muxed_err <= scal_err_i;
			muxed_rty <= scal_rty_i;
			muxed_dat_i <= scal_dat_i;
		end else begin
			muxed_ack <= rad_id_ctrl_ack_i;
			muxed_err <= rad_id_ctrl_err_i;
			muxed_rty <= rad_id_ctrl_rty_i;
			muxed_dat_i <= rad_id_ctrl_dat_i;
		end
	end
	

	// The 'debug' output is a minimal version of the WISHBONE interface.
	// Because the interconnect is a shared bus at this point, this is the entire bus view.
	// If it becomes a crossbar I'll have to do something more intelligent.
	// Output and input data are muxed because we don't have enough pins.

//	wire cyc = (bmc_cyc_i && bmc_gnt) || (spic_cyc_i && spic_gnt) || (pciec_cyc_i && pciec_gnt);
//	wire stb = (bmc_stb_i && bmc_gnt) || (spic_stb_i && spic_gnt) || (pciec_stb_i && pciec_gnt);
//	wire we = (bmc_we_i && bmc_gnt) || (spic_we_i && spic_gnt) || (pciec_we_i && pciec_gnt);
//	reg [19:0] adr;
//	reg [31:0] dat_o;
//	reg [3:0] sel;

	reg [31:0] wbc_debug_data = {32{1'b0}};
	reg [19:0] wbc_debug_adr = {20{1'b0}};
	reg [3:0] wbc_debug_sel = {4{1'b0}};
	reg wbc_debug_cyc = 0;
	reg wbc_debug_stb = 0;
	reg wbc_debug_ack = 0;
	reg wbc_debug_we = 0;
	reg wbc_debug_err = 0;
	reg wbc_debug_rty = 0;
	reg [NUM_MASTERS-1:0] wbc_debug_gnt = {NUM_MASTERS{1'b0}};
	
	always @(posedge clk_i) begin
		if (we) wbc_debug_data <= dat_o;
		else wbc_debug_data <= muxed_dat_i;
		
		wbc_debug_adr <= adr;
		wbc_debug_cyc <= cyc;
		wbc_debug_sel <= sel;
		wbc_debug_stb <= stb;
		wbc_debug_we <= we;
		wbc_debug_ack <= muxed_ack;
		wbc_debug_err <= muxed_err;
		wbc_debug_rty <= muxed_rty;
		wbc_debug_gnt <= grants;
	end
	assign debug_o[0 +: 32] = wbc_debug_data;
	assign debug_o[32 +: 20] = wbc_debug_adr;
	assign debug_o[52 +: 4] = wbc_debug_sel;
	assign debug_o[56] = wbc_debug_cyc;
	assign debug_o[57] = wbc_debug_stb;
	assign debug_o[58] = wbc_debug_we;
	assign debug_o[59] = wbc_debug_ack;
	assign debug_o[60] = wbc_debug_err;
	assign debug_o[61] = wbc_debug_rty;
	assign debug_o[62 +: NUM_MASTERS] = wbc_debug_gnt;

endmodule
