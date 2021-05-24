`timescale 1ns / 1ps
// ripped from par_lab4d_ram mostly
`include "radiant_debug.vh"
`include "wishbone.vh"
module par_lab4d_fifo #(
        parameter NUM_LAB4 = 24,
        parameter LAB4_BITS = 12,
        parameter DATA_BITS = 16
    )(
        input clk_i,
        input rst_i,
        `WBS_NAMED_PORT(wb, 32, 16, 4),
        
        input sys_clk_i,
        input [NUM_LAB4*DATA_BITS-1:0] lab_dat_i,
        input [NUM_LAB4-1:0] lab_wr_i,
        
        input fifo_rst_i,
        output fifo_empty_o
    );
    
    localparam DEBUG = `LAB4_FIFO_DEBUG;
    
    // if NUM_LAB4=24, then L4W = 5
    localparam L4W=$clog2(NUM_LAB4);
	localparam DATA_OUT_SIZE = (1<<L4W);

    // start at top, go back as many as needed.
    // In other words, we split the 16-bit space evenly among as many as needed.
    // So with 24 LAB4s, each one would get 2048 addresses, or 0x000-0x7FF.
    wire [L4W-1:0] l4_sel = wb_adr_i[16-L4W +: L4W];

	
	reg [31:0] data_out_registered = {32{1'b0}};
    
    // ultrasimple at first!
    wire [NUM_LAB4-1:0] lab_read;
    wire [31:0] data_out[DATA_OUT_SIZE-1:0];

	localparam FSM_BITS=2;
	localparam [FSM_BITS-1:0] IDLE = 0;
	localparam [FSM_BITS-1:0] READ = 1;
	localparam [FSM_BITS-1:0] WTF = 2;
	localparam [FSM_BITS-1:0] ACK = 3;
	reg [FSM_BITS-1:0] state = IDLE;

	wire fifo_read = (wb_cyc_i && wb_stb_i && !wb_we_i) && (state == IDLE);

	always @(posedge clk_i) begin
	   data_out_registered <= data_out[l4_sel];
    end
	
	always @(posedge clk_i) begin
	
		case (state)
			IDLE: if (wb_cyc_i && wb_stb_i && !wb_we_i) state <= ACK;	// read is 1 here
			READ: state <= ACK;														// data is at output here
			WTF: state <= ACK;														// why is this here?!?!?
			ACK: state <= IDLE;														// data is at register here
		endcase
	end    

    wire [NUM_LAB4-1:0] fifo_empty;
    assign fifo_empty_o = &fifo_empty;
    generate
        genvar i,j;
        for (i=0;i<NUM_LAB4;i=i+1) begin : RL
            wire [31:0] raw_data_out;
            assign lab_read[i] = fifo_read && (l4_sel == i);
            lab4d_fifo u_fifo(.wr_clk(sys_clk_i),.wr_en(lab_wr_i[i]),.din(lab_dat_i[DATA_BITS*i +: DATA_BITS]),
                              .rd_clk(clk_i),.rd_en(lab_read[i]),.dout(raw_data_out),
                              .empty(fifo_empty[i]),
                              .rst(fifo_rst_i));
            // reorder: the data comes out of the FIFO essentially big endian.
            // This ensures that if we dump the data out as a stream of bytes
            // it *looks* like a little endian array of 16-bit words
            assign data_out[i] = { raw_data_out[15:0], raw_data_out[31:16] };                              
        end
        for (j=NUM_LAB4;j<DATA_OUT_SIZE;j=j+1) begin : DM
            assign data_out[j] = data_out[ j % (DATA_OUT_SIZE>>1) ];
        end
        if (DEBUG == "TRUE") begin : ILA
            lab_fifo_debug u_ila(.clk(sys_clk_i),.probe0(fifo_rst_i),.probe1(fifo_empty));
         end
    endgenerate
    
    
    assign wb_ack_o = (state == ACK);
    assign wb_dat_o = data_out_registered;
endmodule
