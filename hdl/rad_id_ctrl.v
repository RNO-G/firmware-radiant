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
// 0x0008: CPLD control register.
// 0x000C: unused
// 0x0010: PPS selection register.
// 0x0014: Reset/mode register
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
`include "lab4.vh"
`include "radiant_debug.vh"

module rad_id_ctrl(
		input clk_i,
		input rst_i,
		`WBS_NAMED_PORT(wb, 32, 16, 4),

        // Sysclk stuff. Input 25 MHz...
        input sys_clk_in,
        // output 100 MHz
        output sys_clk_o,
        // output 25 MHz sync to sys_clk_in
        output sys_clk_div4_o,
        // flag, in 100 MHz domain, indicating that this
        // is the clock where sys_clk_div4 goes high
        output sys_clk_div4_flag_o,
        // phase
        output sync_o,
        input sync_reset_i,
        // 200 MHz clock
        output wclk_o,
        
        // phase shifter 12.5 MHz clock
        output sys_clk_div8_ps_o,
        input ps_clk_i,
        input ps_en_i,
        input ps_incdec_i,
        output ps_done_o,
        
        // static WR value when in reset (not actually needed!)
        output [`LAB4_WR_WIDTH-1:0] reset_wr_o,
		// indicates that we're selecting an inverted MONTIMING
		output [1:0] invert_montiming_o,		
		
        // internal LED signal
        input [0:0] internal_led_i,
        
        // Passthroughs for the shared JTAG signals.
        input [1:0] ss_incr_i,
        input [1:0] sclk_i,
        output [1:0] shout_o,
        
        // Burst size register
        output [1:0] burst_size_o,
        
        // JTAG enable
        output JTAGENB,
        // JTAG signals. 1=right, 0=left
        inout  [1:0] SSINCR_TDO,
        output [1:0] CDAT_TDI,
        output [1:0] SCLK_TCK,
        output [1:0] CCLK_TMS,

        // clock sel
        output [1:0] SST_SEL,

		// SPI
		output MOSI,
		input MISO,
		output CS_B,
		output F_LED
    );

		parameter DEVICE = "RDNT";
		parameter VERSION = 32'h00000000;
		
		parameter JTAG_DEBUG = `RAD_ID_CTRL_JTAG_DEBUG;
		parameter SPI_DEBUG = `RAD_ID_CTRL_SPI_DEBUG;
		
		parameter [23:0] MONTIMING_POLARITY = {24{1'b0}};

		// Device DNA (used for identification)
		wire dna_data;
		reg read_reg = 0;
		reg shift_reg = 0;

		reg wb_ack_mux;
		reg [31:0] wb_data_out_mux;
		// Our internal space is 4 bits wide = 16 registers (5:2 only matter).
		// The last 4 are really the SPI core, and the 2 before that are reserved.
		wire [31:0] wishbone_registers[15:0];
        
        // these determine which phase of the 100 MHz clock is picked off
        // for matching to the 25 MHz clock.
		reg [1:0] phase_select = {2{1'b0}};

        parameter CBIT_COUNT = 8;
        parameter LOG2_CBIT_COUNT = 3;
        parameter BIST_BIT = 7;
        
        reg [2*CBIT_COUNT-1:0] cpld_ctrl = {2*CBIT_COUNT{1'b0}};
        reg [1:0] cpld_ctrl_update = {2{1'b0}};
        reg [1:0] cpld_ctrl_busy = {2{1'b0}};
        // BIST is the top bit, and inverts the direction of SS_INCR to use as an input.
        // *This* type of BIST we can *always* do. We CANNOT always do the analog BIST
        // portion which relies on custom (non-running) CPLD firmware.
        // We need to be a bit careful here, since the CPLD's output driver can do
        // wonky things when we switch.
        // So cpld_bist ALWAYS gets set whenever we're running. 
        wire [1:0] cpld_bist = { cpld_ctrl[CBIT_COUNT+BIST_BIT] || cpld_ctrl_busy[1], cpld_ctrl[BIST_BIT] || cpld_ctrl_busy[0] };
        wire [31:0] cpld_ctrl_reg = { {7{1'b0}}, cpld_ctrl_busy[1],
                                      {(8-CBIT_COUNT){1'b0}}, cpld_ctrl[CBIT_COUNT +: CBIT_COUNT],
                                      {7{1'b0}}, cpld_ctrl_busy[0],
                                      {(8-CBIT_COUNT){1'b0}}, cpld_ctrl[0 +: CBIT_COUNT]};
        reg [5:0] cclk_count = {6{1'b0}};
        reg [2*LOG2_CBIT_COUNT-1:0] cbit_count = {2*LOG2_CBIT_COUNT{1'b0}};
        
        assign invert_montiming_o[0] = (MONTIMING_POLARITY[ cpld_ctrl_reg[2:0] ]);
        assign invert_montiming_o[1] = (MONTIMING_POLARITY[ 12+cpld_ctrl_reg[CBIT_COUNT +: 3] ]);            

        // Reset/mode register
        // bit [0]     = MMCM (all internal clock) reset
        // bit [9:8]   = burst size
        // bit [20:16] = RESETWR (static WR value when in reset)
        // bit [30:29] = SST_SEL outputs
        // bit [31]    = JTAG mode
		reg [31:0] reset_reg = {32{1'b0}};        

		reg [31:0] pps_sel_reg = {32{1'b0}};
		reg [31:0] led_reg = {32{1'b0}};
		wire [31:0] jtag_left_reg;
		wire [31:0] jtag_right_reg;
		reg [31:0] spiss_reg = {32{1'b0}};
		reg internal_ack = 0;

        wire jtagen = reset_reg[31];
        assign JTAGENB = jtagen;

        // control signals for the aux CPLDs,
        // these will get handled later.
        reg [1:0] cclk_i = {2{1'b0}};
        reg [1:0] cdat_i = {2{1'b0}};

        // resynchronized in sysclk
        reg [`LAB4_WR_WIDTH-1:0] reset_wr_sysclk = {`LAB4_WR_WIDTH{1'b0}};
        reg [`LAB4_WR_WIDTH-1:0] reset_wr_sysclk_rereg = {`LAB4_WR_WIDTH{1'b0}};
        always @(posedge sys_clk_o) begin
            reset_wr_sysclk <= reset_reg[16 +: `LAB4_WR_WIDTH];
            reset_wr_sysclk_rereg <= reset_wr_sysclk;
        end
        assign reset_wr_o = reset_wr_sysclk_rereg;

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
		`WISHBONE_ADDRESS( 16'h0008, cpld_ctrl_reg, OUTPUTSELECT, sel_cpld_ctrl, 0);
		`WISHBONE_ADDRESS( 16'h000C, {32{1'b0}}, OUTPUT, [31:0], 0);
		`WISHBONE_ADDRESS( 16'h0010, pps_sel_reg, SIGNALRESET, [31:0], {32{1'b0}});
		`WISHBONE_ADDRESS( 16'h0014, reset_reg, SIGNALRESET, [31:0], {32{1'b0}});
		`WISHBONE_ADDRESS( 16'h0018, led_reg, OUTPUTSELECT, sel_led_reg, 0);
		`WISHBONE_ADDRESS( 16'h001C, jtag_left_reg, OUTPUTSELECT, sel_jtag_left, 0);
		`WISHBONE_ADDRESS( 16'h0020, jtag_right_reg, OUTPUTSELECT, sel_jtag_right, 0);
		`WISHBONE_ADDRESS( 16'h0024, spiss_reg, SIGNALRESET, [31:0], {32{1'b0}});
		`WISHBONE_ADDRESS( 16'h0028, {{30{1'b0}},phase_select}, OUTPUTSELECT, sel_phase_select, 0);
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
		// [29] TDI bit order reverse-on-write
		// [30] enable sequence
		// [31] data not done capturing
		//
		// Any write with bit 30 set autosets the sequence. That way we run at ~1 Mbit if you do, for instance:
		// first write:
		// 4-byte write to JTAG address: 47 00 00 FF  : shifts out FF with TMS held high
		// 128-byte write to JTAG address with address increment disabled
		// 00 FF 00 FF 00 FF 00 FF ... etc.
		// Because the enable sequence and number of bits are never updated, only the TDI values update
		// and things just stream out.
		reg [1:0] enable_sequence = {2{1'b0}};
		reg [1:0] sequence_running = {2{1'b0}};
		reg [1:0] reverse_bitorder = {2{1'b0}};

        reg [15:0] tdi = {16{1'b0}};
        reg [15:0] tms = {16{1'b0}};
        reg [15:0] tdo = {16{1'b0}};

		reg [5:0] nbit_count_max = {6{1'b0}};
		reg [5:0] nbit_count = {6{1'b0}};

        assign jtag_left_reg = { sequence_running[0], enable_sequence[0], reverse_bitorder[0], 2'b00, nbit_count_max[0 +: 3],
                                 tdo[0 +: 8], tms[0 +: 8], tdi[0 +: 8] };
        assign jtag_right_reg = {sequence_running[1], enable_sequence[1], reverse_bitorder[1], 2'b00, nbit_count_max[3 +: 3],
                                 tdo[8 +: 8], tms[8 +: 8], tdi[8 +: 8] };


        reg [5:0] sclk_count = {6{1'b0}};        

        (* IOB = "TRUE" *)
        reg [1:0] ssincr_tdo_reg = {2{1'b0}};
        (* IOB = "TRUE" *)
        reg [1:0] cdat_tdi_reg = {2{1'b0}};
        (* KEEP = "TRUE" *)
        reg [1:0] cdat_tdi_reg_copy = {2{1'b0}};
        (* IOB = "TRUE" *)
        reg [1:0] sclk_tck_reg = {2{1'b0}};
        (* KEEP = "TRUE" *)
        reg [1:0] sclk_tck_reg_copy = {2{1'b0}};
        (* IOB = "TRUE" *)
        reg [1:0] cclk_tms_reg = {2{1'b0}};
        (* KEEP = "TRUE" *)
        reg [1:0] cclk_tms_reg_copy = {2{1'b0}};
        generate
            if (JTAG_DEBUG=="TRUE") begin : JTGILA
                jtag_ila u_jtag_ila(.clk(clk_i),
                                    .probe0(enable_sequence),
                                    .probe1(sequence_running),
                                    .probe2(nbit_count),
                                    .probe3(sclk_count),
                                    .probe4(cdat_tdi_reg_copy),
                                    .probe5(sclk_tck_reg_copy),
                                    .probe6(cclk_tms_reg_copy),
                                    .probe7(ssincr_tdo_reg));        
            end
        endgenerate
    
//        // JTAG signals. 1=right, 0=left
//        inout  [1:0] SSINCR_TDO,
//        output [1:0] CDAT_TDI,
//        output [1:0] SCLK_TCK,
//        output [1:0] CCLK_TMS,
        assign SSINCR_TDO[0] = (jtagen || cpld_bist[0]) ? 1'bZ : ss_incr_i[0];
        assign SSINCR_TDO[1] = (jtagen || cpld_bist[1]) ? 1'bZ : ss_incr_i[1];        
        assign CDAT_TDI = cdat_tdi_reg;
        assign SCLK_TCK = sclk_tck_reg;
        assign CCLK_TMS = cclk_tms_reg;
        
        wire [1:0] sel_jtag = { sel_jtag_right, sel_jtag_left };
        wire sel_cpld_ctrl_qual = (sel_cpld_ctrl && !jtagen);
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
                    ///////////////////////////////////////////////////////////                    
                    // CCLK/CDAT logic
                    ///////////////////////////////////////////////////////////                    
                    cpld_ctrl_update[s] <= sel_cpld_ctrl_qual && wb_we_i 
                                           && wb_sel_i[2*s+1] 
                                           && wb_dat_i[16*s + 8];

                    if (sel_cpld_ctrl_qual && wb_we_i && wb_sel_i[2*s] 
                        && wb_sel_i[2*s+1] && wb_dat_i[16*s+8]) 
                        cpld_ctrl[CBIT_COUNT*s +: CBIT_COUNT] <= wb_dat_i[16*s +: CBIT_COUNT];                    
                    
                    if (cpld_ctrl_update[s]) cpld_ctrl_busy[s] <= 1;
                    else if ((cbit_count[LOG2_CBIT_COUNT*s +: LOG2_CBIT_COUNT] == CBIT_COUNT-1) 
                             && cclk_count[3*s+2]) cpld_ctrl_busy[s] <= 0;
                    
                    if (cpld_ctrl_busy[s]) 
                        cclk_count[3*s +: 3] <= cclk_count[3*s +: 2] + 1;
                    else cclk_count[3*s +: 3] <= {3{1'b0}};
                    
                    if (cpld_ctrl_busy[s] && (cclk_count[3*s +: 3] == 2)) 
                        cclk_i[s] <= 1'b1;
                    else if (cpld_ctrl_busy[s] && (cclk_count[3*s+2])) 
                        cclk_i[s] <= 1'b0;
                    
                    if (cpld_ctrl_busy[s] && (cclk_count[3*s +: 3] == 1)) 
                        cdat_i[s] <= cpld_ctrl[CBIT_COUNT*s + cbit_count[LOG2_CBIT_COUNT*s +: LOG2_CBIT_COUNT]];

                    if (cpld_ctrl_busy[s]) begin
                        if (cclk_count[3*s+2]) 
                            cbit_count[LOG2_CBIT_COUNT*s +: LOG2_CBIT_COUNT] <= 
                                cbit_count[LOG2_CBIT_COUNT*s +: LOG2_CBIT_COUNT] + 1;
                    end else begin
                        cbit_count[LOG2_CBIT_COUNT*s +: LOG2_CBIT_COUNT] <= {LOG2_CBIT_COUNT{1'b0}};
                    end

                    ///////////////////////////////////////////////////////////                    
                    // JTAG logic
                    ///////////////////////////////////////////////////////////                    
                
                    if (sel_jtag[s] && wb_we_i && wb_sel_i[3]) reverse_bitorder[s] <= wb_dat_i[29];
                    if (sel_jtag[s] && wb_we_i && wb_sel_i[3]) enable_sequence[s] <= wb_dat_i[30];
                    if ((nbit_count[3*s +: 3] == nbit_count_max[3*s +: 3]) && sclk_count[3*s+2])
                        sequence_running[s] <= 0;
                    else if (sel_jtag[s] && wb_we_i && ((!wb_sel_i[3] && enable_sequence[s]) || wb_sel_i[3] && wb_dat_i[30]))
                        sequence_running[s] <= 1;
                    
                    // sclk counter
                    if (sequence_running[s]) sclk_count[3*s +: 3] <= sclk_count[3*s +: 2] + 1;
                    else sclk_count[3*s +: 3] <= {3{1'b0}};
                    
                    if (!jtagen) begin
                        cdat_tdi_reg[s] <= cdat_i[s];
                        cdat_tdi_reg_copy[s] <= cdat_i[s];
                    end else if (sequence_running[s] && (sclk_count[3*s +: 3]==1)) begin
                        cdat_tdi_reg[s] <= tdi[8*s + nbit_count[3*s +: 3]];
                        cdat_tdi_reg_copy[s] <= tdi[8*s + nbit_count[3*s +: 3]];
                    end
                    
                    if (!jtagen) begin
                        cclk_tms_reg[s] <= cclk_i[s];
                        cclk_tms_reg_copy[s] <= cclk_i[s];
                    end else if (sequence_running[s] && (sclk_count[3*s +: 3]==1)) begin
                        cclk_tms_reg[s] <= tms[8*s + nbit_count[3*s +: 3]];
                        cclk_tms_reg_copy[s] <= tms[8*s + nbit_count[3*s +: 3]];
                    end
                    
                    if (!jtagen) begin
                        sclk_tck_reg[s] <= sclk_i[s];
                        sclk_tck_reg_copy[s] <= sclk_i[s];
                    end else if (sequence_running[s] && (sclk_count[3*s +: 3]==2)) begin
                        sclk_tck_reg[s] <= 1'b1;
                        sclk_tck_reg_copy[s] <= 1'b1;
                    end else if (sequence_running[s] && (sclk_count[3*s + 2])) begin
                        sclk_tck_reg[s] <= 1'b0;
                        sclk_tck_reg_copy[s] <= 1'b0;
                    end
                    
                    if ((jtagen && sequence_running[s] && (sclk_count[3*s +: 3]==3)) || cpld_bist[s])
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
                    // tdi has magic byte reversal powers, tms/tdo do not.
                    if (sel_jtag[s] && wb_we_i && wb_sel_i[0]) begin
                        if ((wb_sel_i[3] && !wb_dat_i[29]) || (!wb_sel_i[3] && !reverse_bitorder[s]))
                            tdi[8*s +: 8] <= wb_dat_i[0 +: 8];
                        else begin
                            tdi[8*s + 0] <= wb_dat_i[7];
                            tdi[8*s + 1] <= wb_dat_i[6];
                            tdi[8*s + 2] <= wb_dat_i[5];
                            tdi[8*s + 3] <= wb_dat_i[4];
                            tdi[8*s + 4] <= wb_dat_i[3];
                            tdi[8*s + 5] <= wb_dat_i[2];
                            tdi[8*s + 6] <= wb_dat_i[1];
                            tdi[8*s + 7] <= wb_dat_i[0];
                        end
                    end                            
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

        // SYS CLK WIZARDRY		
		// We just want a multiply by 4, so we'll boost the VCO to 1 GHz and divide by 10.
		// 
		wire SST_FB;
		wire sys_clk_mmcm;
		wire sys_clk_div4_mmcm;
		wire sys_clk_div8_ps_mmcm;
		wire mmcm_reset = reset_reg[0];

		MMCME2_ADV #(
		.BANDWIDTH("OPTIMIZED"), // Jitter programming ("HIGH","LOW","OPTIMIZED")
		.CLKFBOUT_MULT_F(40.0), // Multiply value for all CLKOUT (2.000-64.000).
		.CLKFBOUT_PHASE(0.0), // Phase offset in degrees of CLKFB (0.00-360.00).
		// CLKIN_PERIOD: Input clock period in ns to ps resolution (i.e. 33.333 is 30 MHz).
		.CLKIN1_PERIOD(40.0),
		.CLKIN2_PERIOD(40.0),
		.CLKOUT0_DIVIDE_F(5.0), // WCLK (400 MHz)
		.CLKOUT1_DIVIDE(10.0),	 // SYSCLK (100 MHz)
		.CLKOUT2_DIVIDE(40.0),	 // SYSCLK_DIV4 (25 MHz)
		.CLKOUT3_DIVIDE(80.0),	 // SYSCLK_DIV8_PS (12.5 MHz)
		.CLKOUT3_USE_FINE_PS("TRUE"),
		// CLKOUT0_DUTY_CYCLE - CLKOUT6_DUTY_CYCLE: Duty cycle for CLKOUT outputs (0.01-0.99).
		.CLKOUT0_DUTY_CYCLE(0.5),
		// CLKOUT0_PHASE - CLKOUT6_PHASE: Phase offset for CLKOUT outputs (-360.000-360.000).
		.CLKOUT0_PHASE(0.0),
		.DIVCLK_DIVIDE(1), // Master division value (1-106)		
		.REF_JITTER1(0.0),
		.REF_JITTER2(0.0),
		.STARTUP_WAIT("FALSE")) u_mmcm(	.CLKIN1(sys_clk_in),
													.CLKIN2(),
													.CLKOUT0(wclk_mmcm),
													.CLKOUT1(sys_clk_mmcm),
													.CLKOUT2(sys_clk_div4_mmcm),
													.CLKOUT3(sys_clk_div8_ps_mmcm),
													.PSCLK(ps_clk_i),
													.PSEN(ps_en_i),
													.PSINCDEC(ps_incdec_i),
													.PSDONE(ps_done_o),
													.LOCKED(),
													.RST(mmcm_reset),
													.PWRDWN(1'b0),
													.CLKFBOUT(mmcm_fb_out),
													.CLKFBIN(mmcm_fb_in),
													.CLKINSEL(1'b1),
													.CLKFBSTOPPED(),
													.CLKINSTOPPED());
		BUFG u_sysclk(.I(sys_clk_mmcm),.O(sys_clk_o));
		BUFG u_sysclk_fb(.I(mmcm_fb_out),.O(mmcm_fb_in));
		BUFG u_sysclk_div4(.I(sys_clk_div4_mmcm),.O(sys_clk_div4_o));
		BUFG u_wclk(.I(wclk_mmcm),.O(wclk_o));
		BUFG u_clkps(.I(sys_clk_div8_ps_mmcm),.O(sys_clk_div8_ps_o));
		
		reg phase_select_flag = 0;
		wire phase_select_flag_sysclk;
		reg [1:0] phase_select_sysclk = {2{1'b0}};

		reg [3:0] div4_flag_quadrature = {4{1'b0}};		
		wire div4_flag = div4_flag_quadrature[phase_select_sysclk];

		always @(posedge clk_i) begin
			if (sel_phase_select && wb_we_i) begin
				phase_select <= wb_dat_i[1:0];
				phase_select_flag <= 1;
			end else begin
				phase_select_flag <= 0;
			end
		end
		flag_sync u_phase_wr_sync(.in_clkA(phase_select_flag),.clkA(clk_i),.out_clkB(phase_select_flag_sysclk),.clkB(sys_clk_o));
		always @(posedge sys_clk_o) begin
			if (phase_select_flag_sysclk)
				phase_select_sysclk <= phase_select;
		end
		
        reg toggle_sysclk_div4 = 0;
        reg [1:0] toggle_sysclk_div4_in_sysclk = 2'b00;
        reg toggle_edge_detect_in_sysclk = 0;
        reg delay_edge_detect = 0;
        reg sys_clk_div4_flag = 0;
        reg sync_reset_req = 0;
        reg sync_reg = 0;
		// OK, so this is complicated. First, we generate a toggle flop in the 25 MHz domain.
		// Just gives us a register to re-register in the 100 MHz domain. Doesn't matter its phase,
		// we're just looking for edges.
		always @(posedge sys_clk_div4_o) begin
			toggle_sysclk_div4 <= ~toggle_sysclk_div4;
		end
		always @(posedge sys_clk_o) begin
			toggle_sysclk_div4_in_sysclk <= {toggle_sysclk_div4_in_sysclk[0],toggle_sysclk_div4};
			// So when toggle_sysclk_div4_in_sysclk is '01'/'10', that means we are at (first time)
			// sysclk: _-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_- 
			//    clk: ________--------________--------
			// toggle: _________----------------_______
			// tginsy: 00000000001133333333333333220000
			// detflg: 00000000000011000000000000110000
			//   flg1: 00000000000000110000000000001100
			//   flg2: 00000000000000001100000000000011
			//
			// detflg looks for '01' or '10' in the 2-bit shift register.
			// We then delay that flag 2 cycles, and that then indicates
			// the *first* cycle in the 4-cycle clock period.
			// 
			// Timing analysis should guarantee that the toggle_sysclk_div4 -> toggle_sysclk_div4_in_sysclk
			// transition happens cleanly, and the clock periods are long enough that it's tough to imagine
			// skew causing a problem.
			toggle_edge_detect_in_sysclk <= (toggle_sysclk_div4_in_sysclk == 2'b10 || toggle_sysclk_div4_in_sysclk == 2'b01);
			delay_edge_detect <= toggle_edge_detect_in_sysclk;
			div4_flag_quadrature <= {div4_flag_quadrature[2:0],delay_edge_detect};
			sys_clk_div4_flag <= div4_flag;

			if (sys_clk_div4_flag) begin
				if (sync_reset_req) sync_reg <= 0;
				else sync_reg <= ~sync_reg;
			end
			if (sync_reset_i) sync_reset_req <= 1;
			else if (sys_clk_div4_flag) sync_reset_req <= 0;
		end														
		
		assign sys_clk_div4_flag_o = sys_clk_div4_flag;		
        assign sync_o = sync_reg;		

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

        generate
            if (SPI_DEBUG=="TRUE") begin : SPIILA
                spi_ila u_spi_ila(.clk(clk_i),
                                    .probe0( spi_sck ),
                                    .probe1( CS_B ),
                                    .probe2( MOSI ),
                                    .probe3( MISO ));
            end
        endgenerate
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
        assign shout_o = ssincr_tdo_reg;
        
        assign SST_SEL = ~reset_reg[30:29];

        assign burst_size_o = reset_reg[9:8];
endmodule
