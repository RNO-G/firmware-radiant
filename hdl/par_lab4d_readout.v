`timescale 1ns / 1ps
// Updated version of the LAB4D readout. This one doesn't actually attempt to store it in a RAM,
// it just outputs the data. The data can then pass through another part (e.g. the calram) and
// *then* be collected into memory.
module par_lab4d_readout #(
        parameter NUM_SS_INCR = 2,
        parameter NUM_SRCLK = 2,
        parameter SRCLK_DIFFERENTIAL = "TRUE",
        parameter [NUM_SRCLK-1:0] SRCLK_POLARITY = {NUM_SRCLK{1'b0}},
        parameter NUM_LAB4 = 24,
        parameter LAB4_BITS = 12,
        parameter LAB4_SAMPLE_BITS = 12,
        parameter HEADER_BITS = 4,
        parameter [NUM_LAB4-1:0] DOE_POLARITY = {NUM_LAB4{1'b0}},
        parameter SURF5_COMPAT = "FALSE"
    )
    (
        input clk_i,
        input sys_clk_i,
        input wclk_i,
        // readout controls from LAB4 controller
        // flag to start LAB4D readout
        input readout_i,
        // flag to reset the window counter
        input readout_counter_rst_i,
        // these will probably have to be made per-LAB eventually
        input [HEADER_BITS-1:0] readout_header_i,
        // and these likely too. These tell what portion of the counter to
        // replace with the header: e.g. 1100 means replace top 2 bits,
        // 1000 replace top bit, 1110 replace top 3 bits. (Bottom one always needs to be zero).
        input [HEADER_BITS-1:0] readout_counter_mask_i,        
        input readout_rst_i,
        output complete_o,
        
        
        input [15:0] prescale_i,
        
        // Data coming out
        output [NUM_LAB4*LAB4_BITS-1:0]     lab_dat_o,
        // Headers for each.
        output [NUM_LAB4*HEADER_BITS-1:0]   lab_header_o,
        // Sample counter. 
        output [NUM_LAB4*LAB4_SAMPLE_BITS-1:0] lab_sample_o,
        output [NUM_LAB4-1:0]               lab_wr_o,
        
        input [NUM_LAB4-1:0] DOE_LVDS_P,
        input [NUM_LAB4-1:0] DOE_LVDS_N,
        
        output [NUM_SS_INCR-1:0] SS_INCR,
        output [NUM_SRCLK-1:0] SRCLK_P,
        output [NUM_SRCLK-1:0] SRCLK_N      
    );
    
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

    wire data_wr;
    wire [NUM_LAB4*L4_BITS-1:0] data;    

	wire [6:0] sample_counter;
	wire [3:0] bit_counter;
	wire dbg_ss_incr;
	wire dbg_srclk;
	par_lab4d_data_shift_register #(.NUM_LAB4(NUM_LAB4),.NUM_SS_INCR(NUM_SS_INCR),.NUM_SRCLK(NUM_SRCLK))
	                                          u_shreg(.sys_clk_i(sys_clk_i),.wclk_i(wclk_i),
													  .readout_i(readout_i),.readout_rst_i(readout_rst_i),
													  .done_o(complete_o),.dat_o(data),
													  .dat_wr_o(data_wr),.prescale_i(prescale_i),
													  .ss_incr_o(dbg_ss_incr),.srclk_o(dbg_srclk),
													  .bit_counter_o(bit_counter),
													  .sample_counter_o(sample_counter),.DOE(DOE),.SS_INCR(SS_INCR),.SRCLK(SRCLK),
													  .sample_debug(sample_debug));
    

    generate
        genvar i;
        for (i=0;i<NUM_LAB4;i=i+1) begin : DO            
            // Window counter is per-LAB4... maybe?
            reg [4:0]  window_counter = {5{1'b0}};
            reg [11:0] data_reg = {12{1'b0}};
            reg [HEADER_BITS-1:0] header_reg = {HEADER_BITS{1'b0}};
            reg        data_wr_reg = 0;
            reg [11:0] lab_sample = {12{1'b0}};
            // register the data to help routing. might not be needed.
            always @(posedge sys_clk_i) begin : DR
                data_wr_reg <= data_wr;
                if (data_wr) data_reg <= data[L4_BITS*i +: L4_BITS];
                
                if (data_wr) header_reg <= readout_header_i;
                
                if (readout_counter_rst_i) window_counter <= {5{1'b0}};
                else if (complete_o) window_counter <= window_counter + 1;
                
                // Sample counter is:
                // (window_counter[4:2] & ~readout_counter_mask_i[3:1]) | (readout_header_i[3:1] & readout_counter_mask_i[3:1]) (3 bits)
                // window_counter[1:0] (2 bits)
                // sample_counter[6:0] (7 bits)
                //
                // This means we can swap from 1024 events/sample -> 2048 events/sample just by changing the input mask and changing
                // the LAB4 controller core (and the trigger handler obviously)
                if (data_wr) begin
                    lab_sample[6:0] <= sample_counter[6:0];
                    lab_sample[8:7] <= window_counter[1:0];
                    lab_sample[11:9] <= (window_counter[4:2] & ~readout_counter_mask_i[3:1]) | (readout_header_i[3:1] & readout_counter_mask_i[3:1]);
                end
            end
            assign lab_sample_o[LAB4_SAMPLE_BITS*i +: LAB4_SAMPLE_BITS] = lab_sample;
            assign lab_dat_o[L4_BITS*i +: L4_BITS] = data_reg;
            assign lab_header_o[HEADER_BITS*i +: HEADER_BITS] = header_reg;
            assign lab_wr_o[i] = data_wr_reg;
        end
    endgenerate
    
endmodule
