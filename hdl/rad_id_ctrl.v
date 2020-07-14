`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// Author: Patrick Allison, Ohio State University (allison.122@osu.edu)
// All rights reserved.
////////////////////////////////////////////////////////////////////////////////


// RADIANT ID and Control Block.
//
// This module contains:
//
// *) Device ID register
// *) Firmware ID register
// *) a few general purpose I/O registers (clock selection, internal PPS selection)
// *) LED output charlieplexer
// *) SPI flash WISHBONE module.
//
// Module memory map:
// Internal address range is 0x0000-0xFFFF.
// 0x0000: Device ID   ('RDNT')
// 0x0004: Firmware ID (standard day/month/major/minor/revision packing)
// 0x0008: unused
// 0x000C: unused
// 0x0010: PPS selection register.
// 0x0014: Reset register (global reset)
// 0x0018: General control (pulser, LED, I guess)
// 0x001C: JTAG core, left.
// 0x0020: JTAG core, right.
// 0x0024: SPI core slave select output register.
// NOT YET - 0x0028: Phase select register.
// 0x002C: Device DNA port.
// 0x0030-0x003F: Simple SPI core.
//
// *Note*: In order to use the SPI flash, the internal CCLK signal needs a few dummy
//         cycles to switch over from the internal configuration clock. To do this,
//         execute a dummy SPI transaction with the Slave Select pin high (bit 0 in
//         register 0x0030 = 0) before performing any real SPI transactions.
`include "wishbone.vh"
module rad_id_ctrl(
		input clk_i,
		input rst_i,
		`WBS_NAMED_PORT(wb, 32, 16, 4),
		
        // internal LED signal
        input [0:0] internal_led_i,
        
        // Passthroughs for the shared JTAG signals.
        input [1:0] ss_incr_i,
        input [1:0] sclk_i,
        
        // JTAG enable
        output JTAGENB,
        // JTAG signals. 1=right, 0=left
        inout  [1:0] SSINCR_TDO,
        output [1:0] CDAT_TDI,
        output [1:0] SCLK_TCK,
        output [1:0] CCLK_TMS,

		// SPI
		output MOSI,
		input MISO,
		output CS_B,
		output F_LED
    );

		parameter DEVICE = "RDNT";
		parameter VERSION = 32'h00000000;

		// Device DNA (used for identification)
		wire dna_data;
		reg read_reg = 0;
		reg shift_reg = 0;

		reg wb_ack_mux;
		reg [31:0] wb_data_out_mux;
		// Our internal space is 4 bits wide = 16 registers (5:2 only matter).
		// The last 4 are really the SPI core, and the 2 before that are reserved.
		wire [31:0] wishbone_registers[15:0];

		reg [31:0] reset_reg = {32{1'b0}};
		reg [31:0] pps_sel_reg = {32{1'b0}};
		reg [31:0] led_reg = {32{1'b0}};
		wire [31:0] jtag_left_reg;
		wire [31:0] jtag_right_reg;
		reg [31:0] spiss_reg = {32{1'b0}};
		reg internal_ack = 0;

        wire jtagen = reset_reg[31];
        assign JTAGENB = !jtagen;

        // control signals for the aux CPLDs,
        // these will get handled later.
        wire [1:0] cclk_i = {2{1'b0}};
        wire [1:0] cdat_i = {2{1'b0}};

		wire [31:0] spi_dat_o;
		wire spi_ack_o;
		wire spi_inta_o;
		wire spi_select = (wb_adr_i[7:4] == 4'h3) && (!wb_adr_i[9]);
		wire spi_sck;
		wire spi_cyc = wb_cyc_i && spi_select;
		
		always @(*) begin
			if (spi_select) wb_data_out_mux <= spi_dat_o;
			else wb_data_out_mux <= wishbone_registers[wb_adr_i[5:2]];
		end
		always @(*) begin
			if (spi_select) wb_ack_mux <= spi_ack_o;
			else wb_ack_mux <= internal_ack;
		end
		always @(posedge clk_i) begin
		    internal_ack <= wb_cyc_i && wb_stb_i && ~internal_ack;
        end
		assign wb_ack_o = wb_ack_mux && wb_cyc_i;
		assign wb_dat_o = wb_data_out_mux;
		assign wb_err_o = 0;
		assign wb_rty_o = 0;
		// BASE needs to be defined to convert the base address into an index.
		localparam BASEWIDTH = 4;
		function [BASEWIDTH-1:0] BASE;
				input [15:0] bar_value;
				begin
						BASE = bar_value[5:2];
				end
		endfunction
		`define OUTPUT(addr, x, range, dummy)																				\
					assign wishbone_registers[ addr ] range = x
		`define SELECT(addr, x, dummy, dummy1)																			\
					wire x;																											\
					localparam [BASEWIDTH-1:0] addr_``x = addr;															\
					assign x = (wb_cyc_i && wb_stb_i && wb_we_i && wb_ack_o && (BASE(wb_adr_i) == addr_``x))
		`define OUTPUTSELECT(addr, x, y, dummy)																		\
					wire y;																											\
					localparam [BASEWIDTH-1:0] addr_``y = addr;															\
					assign y = (wb_cyc_i && wb_stb_i && wb_ack_o && (BASE(wb_adr_i) == addr_``y));	\
					assign wishbone_registers[ addr ] = x

		`define SIGNALRESET(addr, x, range, resetval)																	\
					always @(posedge clk_i) begin																			\
						if (rst_i) x <= resetval;																				\
						else if (wb_cyc_i && wb_stb_i && wb_we_i && (BASE(wb_adr_i) == addr))		\
							x <= wb_dat_i range;																						\
					end																												\
					assign wishbone_registers[ addr ] range = x
		`define WISHBONE_ADDRESS( addr, name, TYPE, par1, par2 )														\
					`TYPE(BASE(addr), name, par1, par2)

		// Sleaze at first. Just make all the registers 32 bit.
		`WISHBONE_ADDRESS( 16'h0000, DEVICE, OUTPUT, [31:0], 0);
		`WISHBONE_ADDRESS( 16'h0004, VERSION, OUTPUT, [31:0], 0);
		`WISHBONE_ADDRESS( 16'h0008, {32{1'b0}}, OUTPUT, [31:0], 0);
		`WISHBONE_ADDRESS( 16'h000C, {32{1'b0}}, OUTPUT, [31:0], 0);
		`WISHBONE_ADDRESS( 16'h0010, pps_sel_reg, SIGNALRESET, [31:0], {32{1'b0}});
		`WISHBONE_ADDRESS( 16'h0014, reset_reg, SIGNALRESET, [31:0], {32{1'b0}});
		`WISHBONE_ADDRESS( 16'h0018, led_reg, OUTPUTSELECT, sel_led_reg, 0);
		`WISHBONE_ADDRESS( 16'h001C, jtag_left_reg, OUTPUTSELECT, sel_jtag_left, 0);
		`WISHBONE_ADDRESS( 16'h0020, jtag_right_reg, OUTPUTSELECT, sel_jtag_right, 0);
        // 1C is jtag_left
        // 20 is jtag_right
		`WISHBONE_ADDRESS( 16'h0024, spiss_reg, SIGNALRESET, [31:0], {32{1'b0}});
		`WISHBONE_ADDRESS( 16'h0028, {32{1'b0}}, OUTPUT, [31:0], 0);
//		`WISHBONE_ADDRESS( 16'h0028, {{30{1'b0}},phase_select}, OUTPUTSELECT, sel_phase_select, 0);
		`WISHBONE_ADDRESS( 16'h002C, {{31{1'b0}},dna_data}, OUTPUTSELECT, sel_dna, 0);
		// Shadow registers - never accessed (the SPI core takes over). Just here to make decoding easier.
		`WISHBONE_ADDRESS( 16'h0030, pps_sel_reg, OUTPUT, [31:0], 0);
		`WISHBONE_ADDRESS( 16'h0034, reset_reg, OUTPUT, [31:0], 0);
		`WISHBONE_ADDRESS( 16'h0038, led_reg, OUTPUT, [31:0], 0);
		`WISHBONE_ADDRESS( 16'h003C, {32{1'b0}}, OUTPUT, [31:0], 0);
		
		// the JTAG addresses work like this:
		// [7:0] TDI output values
		// [15:8] TMS output values
		// [23:16] TDO return values
		// [26:24] number of bits to send minus 1
		// [29] JTAG mux enable
		// [30] enable sequence
		// [31] data done capturing
		//
		// Any write with bit 30 set autosets the sequence. That way we run at ~1 Mbit if you do, for instance:
		// first write:
		// 4-byte write to JTAG address: 47 00 00 FF  : shifts out FF with TMS held high
		// 128-byte write to JTAG address with address increment disabled
		// 00 FF 00 FF 00 FF 00 FF ... etc.
		// Because the enable sequence and number of bits are never updated, only the TDI values update
		// and things just stream out.
		reg [1:0] enable_sequence = 0;
		reg [1:0] sequence_running = 0;

        reg [15:0] tdi = {16{1'b0}};
        reg [15:0] tms = {16{1'b0}};
        reg [15:0] tdo = {16{1'b0}};

		reg [5:0] nbit_count_max = {6{1'b0}};
		reg [5:0] nbit_count = {6{1'b0}};

        assign jtag_left_reg = { sequence_running[0], enable_sequence[0], 3'b000, nbit_count_max[0 +: 3],
                                 tdo[0 +: 8], tms[0 +: 8], tdi[0 +: 8] };
        assign jtag_right_reg = {sequence_running[1], enable_sequence[1], 3'b000, 2'b00, nbit_count_max[3 +: 3],
                                 tdo[8 +: 8], tms[8 +: 8], tdi[8 +: 8] };


        reg [5:0] sclk_count = {6{1'b0}};        

        (* IOB = "TRUE" *)
        reg [1:0] ssincr_tdo_reg = {2{1'b0}};
        (* IOB = "TRUE" *)
        reg [1:0] cdat_tdi_reg = {2{1'b0}};
        (* IOB = "TRUE" *)
        reg [1:0] sclk_tck_reg = {2{1'b0}};
        (* IOB = "TRUE" *)
        reg [1:0] cclk_tms_reg = {2{1'b0}};

//        // JTAG signals. 1=right, 0=left
//        inout  [1:0] SSINCR_TDO,
//        output [1:0] CDAT_TDI,
//        output [1:0] SCLK_TCK,
//        output [1:0] CCLK_TMS,
        assign SSINCR_TDO = (jtagen) ? 2'bZZ : ss_incr_i;
        assign CDAT_TDI = cdat_tdi_reg;
        assign SCLK_TCK = sclk_tck_reg;
        assign CCLK_TMS = cclk_tms_reg;
        
        wire [1:0] sel_jtag = { sel_jtag_right, sel_jtag_left };
        generate    
            genvar s;
		  // clk we sr es nc sc  tck tdi/tms tdo_q
		  // 0   0  0  1  0  0   0   X       X
		  // 1   1  0  1  0  0   0   X       X
		  // 2   0  1  1  0  0   0   X       X
		  // 3   0  1  1  0  1   0   X       X
		  // 4   0  1  1  0  2   0   D       X
		  // 5   0  1  1  0  3   1   D       X
		  // 6   0  1  1  0  4   1   D       <D>
		  // 7   0  0  1  1  1   0   D       <D>
		  // 8   0  0  1  0  0
		  
          // so we set output TDI/TMS at sr=1 && sc=1
          // and set TCK at sr=1 && sc=2
          // capture TDO at sr=1 && sc=3
          // clear TCK at sr=1 && sc=4          
            for (s=0;s<2;s=s+1) begin : SIDE
                always @(posedge clk_i) begin : LOGIC
                    if ((nbit_count[3*s +: 3] == nbit_count_max[3*s +: 3]) && sclk_count[3*s+2])
                        sequence_running[s] <= 0;
                    else if (sel_jtag[s] && wb_we_i && ((!wb_sel_i[3] && enable_sequence[s]) || wb_sel_i[3] && wb_dat_i[31]))
                        sequence_running[s] <= 1;
                    
                    // sclk counter
                    if (sequence_running[s]) sclk_count[3*s +: 3] <= sclk_count[3*s +: 2] + 1;
                    else sclk_count[3*s +: 3] <= {3{1'b0}};
                    
                    if (!jtagen) 
                        cdat_tdi_reg[s] <= cdat_i[s];
                    else if (sequence_running[s] && (sclk_count[3*s +: 3]==1)) 
                        cdat_tdi_reg[s] <= tdi[8*s + nbit_count[3*s +: 3]];
                    
                    if (!jtagen) 
                        cclk_tms_reg[s] <= cclk_i[s];
                    else if (sequence_running[s] && (sclk_count[3*s +: 3]==1)) 
                        cclk_tms_reg[s] <= tms[8*s + nbit_count[3*s +: 3]];
                    
                    if (!jtagen) 
                        sclk_tck_reg[s] <= sclk_i[s];
                    else if (sequence_running[s] && (sclk_count[3*s +: 3]==2)) 
                        sclk_tck_reg[s] <= 1'b1;
                    else if (sequence_running[s] && (sclk_count[3*s + 2]))
                        sclk_tck_reg[s] <= 1'b0;
                    
                    if (jtagen && sequence_running[s] && (sclk_count[3*s +: 3]==3))
                        ssincr_tdo_reg[s] <= SSINCR_TDO[s];
                    if (jtagen && sequence_running[s] && (sclk_count[3*s+2]))
                        tdo[8*s + nbit_count[3*s +: 3]] <= ssincr_tdo_reg[s];
        
                    // nbit counter
                    if (sequence_running[s]) begin
                      if (sclk_count[3*s+2]) nbit_count[3*s +: 3] <= nbit_count[3*s +: 3] + 1;
                    end else nbit_count[3*s +: 3] <= {3{1'b0}};
        
                    // nbit max count
                    if (sel_jtag[s] && wb_we_i && wb_sel_i[3]) nbit_count_max[3*s +: 3] <= wb_dat_i[24 +: 3];
                    
                    // tdi/tms
                    if (sel_jtag[s] && wb_we_i && wb_sel_i[0]) tdi[8*s +: 8] <= wb_dat_i[0 +: 8];
                    if (sel_jtag[s] && wb_we_i && wb_sel_i[1]) tms[8*s +: 8] <= wb_dat_i[8 +: 8];
                end
            end
        endgenerate
		

		//% LED register. If the override is set, then use the written value.
		always @(posedge clk_i) begin : LED_REGISTER
			if (rst_i) led_reg <= {32{1'b0}};
			else begin
				if (sel_led_reg && wb_we_i) begin
				    if (wb_sel_i[2]) led_reg[16] <= wb_dat_i[16];
				    if (((wb_sel_i[2] && wb_dat_i[16]) || (!wb_sel_i[2] && led_reg[16])) 
				        && wb_sel_i[0])
				        led_reg[0] <= wb_dat_i[0];
                end else if (!led_reg[16]) led_reg[0] <= internal_led_i[0];
			end
		end

        assign F_LED = led_reg[0];
        
		simple_spi_top u_spi(.clk_i(clk_i),
							  .rst_i(~rst_i),
							  .inta_o(spi_inta_o),
							  .cyc_i(spi_cyc),
							  .stb_i(wb_stb_i),
							  .we_i(wb_we_i),
							  .adr_i(wb_adr_i[3:2]),
							  .dat_i(wb_dat_i[7:0]),
							  .dat_o(spi_dat_o[7:0]),
							  .ack_o(spi_ack_o),
							  .sck_o(spi_sck),
							  .mosi_o(MOSI),
							  .miso_i(MISO));
		STARTUPE2 #(.PROG_USR("FALSE")) u_startupe2(.CLK(1'b0),
									 .GSR(1'b0),
									 .GTS(1'b0),
									 .KEYCLEARB(1'b0),
									 .PACK(1'b0),
									 .USRCCLKO(spi_sck),
									 .USRCCLKTS(1'b0),
									 .USRDONEO(1'b1),
									 .USRDONETS(1'b1));
		// Device DNA.
		DNA_PORT u_dna(.DIN(1'b0),.READ(read_reg),.SHIFT(shift_reg),.CLK(clk_i),.DOUT(dna_data));		
		always @(posedge clk_i) begin
			if (sel_dna && ~wb_we_i && wb_ack_o) shift_reg <= 1;
			else shift_reg <= 0;
			if (sel_dna && wb_we_i && wb_ack_o && wb_sel_i[3]) read_reg <= wb_dat_i[31];
			else read_reg <= 0;
		end			

		assign CS_B = !spiss_reg[0];		
		assign CS_B_alt = !spiss_reg[0];

endmodule
