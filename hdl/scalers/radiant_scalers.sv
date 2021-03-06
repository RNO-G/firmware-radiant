`timescale 1ns / 1ps
// This is a pretty generic WISHBONE interface
// prescalable scaler module. Cost is 1 DSP/scaler.
// Works up to 64 scalers.
// Scalers can all be individually prescaled.
// Prescaling is linear. Scalers are dual-packed
// in 32 bits.
// Address space:
// 0x40000-0x407FF : control registers
// 0x40000-0x40800 : scaler outputs
// Control registers:
// 0x40000: bit [31] use PPS instead of internal scaler update [30:0] scaler update period (in 20 ns intervals).
//          There is NO update period readback. Just not possible.
// 0x40004: scaler prescale control. bits[31:16] set the address of the scaler prescale to control, bits[7:0] control the prescale level.
//          There is NO prescale readback. Just not possible.
// 0x40080: scalers to map to 0x40800 (bit 0 is ignored, write only even #s)
// 0x40084: scalers to map to 0x40804
// ..
// 0x400FC: scalers to map to 0x408FC
// 0x40088:
// 0x40800+: scalers (2 per 32-bit)
//
// The way scaler mapping works is that they initially just map straight (0/1 read out at 0x40800, 2/3 at 0x40804, etc.)
// but if you want to move them to put, say, 16/17 at 0x40800, just write 16 into 0x40080.
// Note that this just sets the READBACK position. This just allows formatting the event output (allowing you to compress
// unused channels).
// This ends up not being that hard, as it's just a remapping of the address input (so it's a 64x6 RAM).
//
// Scalers operate purely in the 50 MHz domain. No reason to run them faster.
`include "wishbone.vh"
module radiant_scalers #(parameter NUM_SCALERS=32)(
        // this is PPS in the *clock* domain
        input                   pps_i,
        input [NUM_SCALERS-1:0] scal_i,
        input clk_i,
        input rst_i,
        `WBS_NAMED_PORT(wb, 32, 16, 4)        
    );
        
    localparam NUM_SCALERS_EVEN = (NUM_SCALERS % 2) ? NUM_SCALERS+1 : NUM_SCALERS;
    localparam NUM_SCALERS_ADDR_BITS = $clog2(NUM_SCALERS_EVEN);
    localparam NUM_SCALERS_EXPANDED = (1<<NUM_SCALERS_ADDR_BITS);
    localparam NUM_DUAL_SCALERS = NUM_SCALERS_EVEN>>1;
    localparam NUM_DUAL_SCALERS_ADDR_BITS = $clog2(NUM_DUAL_SCALERS);
    localparam NUM_DUAL_SCALERS_EXPANDED = (1 << NUM_DUAL_SCALERS_ADDR_BITS);
    // tack on an unused bit at the top if we're not even
    wire [NUM_SCALERS_EVEN-1:0] scal_in = (NUM_SCALERS_EVEN == NUM_SCALERS) ? scal_i : { 1'b0, scal_i };    

    wire [31:0] dual_scaler_expanded[NUM_DUAL_SCALERS_EXPANDED-1:0];
    wire scaler_reset;
    reg scaler_resetting = 0;
    wire [NUM_DUAL_SCALERS-1:0] scaler_reset_complete;
    wire scaler_reset_done = scaler_reset_complete[0];
    
    reg [NUM_SCALERS_ADDR_BITS-1:0] scaler_update_addr = {NUM_SCALERS_ADDR_BITS{1'b0}};
    reg [7:0] scaler_prescale = {8{1'b0}};
                
    reg scaler_use_pps = 0;
    reg [1:0] scaler_update = 0;
    reg scaler_updating = 0;

    wire scaler_timer_update;
    wire scaler_timer_write = (wb_cyc_i && wb_stb_i && wb_we_i && !wb_adr_i[11] && !wb_adr_i[7] && !wb_adr_i[2]);
    wire scaler_timer_reset = scaler_timer_write;

    reg scaler_timer_writereg = 0;
    wire scalers_now = (scaler_use_pps) ? pps_i : scaler_timer_update;
    wire [NUM_DUAL_SCALERS-1:0] scalers_valid;
    wire scaler_update_done = scalers_valid[0];
    
    reg [31:0] scaler_data_out = {32{1'b0}};
    wire [4:0] scaler_addr;        

    // period, in 1 us intervals
    reg [30:0] scaler_period = 31'd1000000;
    // There are 20 clocks per microsecond, so if we generate a divide by 10
    // we can just use the scaler period upshifted by 2, which costs us nothing.
    wire scaler_ce;
    clk_div_ce #(.CLK_DIVIDE(4)) u_sce(.clk(clk_i),.ce(scaler_ce));
    // 16, 30, 2 = 48
    wire [47:0] scaler_period_in = { {16{1'b0}}, scaler_period, 2'b00 };

    // Load up defaults on the DSPs initially.
    reg initial_load_done = 0;
    reg initial_load = 0;

    reg global_scaler_reset = 1;
    reg [5:0] global_reset_delay = {6{1'b0}};

    always @(posedge clk_i) begin
        if (initial_load) initial_load_done <= 1;
        initial_load <= scaler_ce && !initial_load_done;

        if (global_reset_delay[5]) global_scaler_reset <= 0;
        
        global_reset_delay <= { global_reset_delay[4:0], 1'b1 };
    end

    // Mux the data. Scaler_addr needs to be upshifted so it represents the scaler number.
    wire [31:0] control_data = (wb_adr_i[2]) ? {32{1'b0}} : { scaler_use_pps, scaler_period };
    wire [31:0] nonscaler_data = (wb_adr_i[7]) ? { {26{1'b0}}, scaler_addr, 1'b0 } : control_data;
    wire [31:0] data_out = (wb_adr_i[11]) ? scaler_data_out : nonscaler_data;
    
    generate
        genvar i,j;
        for (i=0;i<NUM_SCALERS_EXPANDED;i=i+1) begin : SCL
            if (i < NUM_DUAL_SCALERS) begin : SCR
                wire this_dual_scaler = (scaler_update_addr[1 +: NUM_DUAL_SCALERS_ADDR_BITS] == i);
                wire low_prescale_select = (this_dual_scaler && !scaler_update_addr[0]) || global_scaler_reset;
                wire high_prescale_select = (this_dual_scaler && scaler_update_addr[0]) || global_scaler_reset;
                wire [1:0] prescale_en = { high_prescale_select, low_prescale_select };
                wire [47:0] scaler_value;
                dual_prescaled_dsp_scalers #(.PIPELINE_INPUT("FALSE"))
                    u_scal( .fast_clk_i(clk_i),
                            .fast_rst_i(scaler_reset || global_scaler_reset),
                            .fast_rst_done_o(scaler_reset_complete[i]),
                            .prescale_en_i(prescale_en),
                            .prescale_i({2{scaler_prescale}}),
                            .count_i( { scal_in[2*i + 1], scal_in[2*i] } ),
                            .update_i(scaler_update[1]),
                            .value_o( scaler_value ),
                            .value_valid_o( scalers_valid[i] ));
                // Scalers peel off the *top* 16 bits of each.
                assign dual_scaler_expanded[i] = { scaler_value[32 +: 16], scaler_value[8 +: 16] };
            end else begin : DUM
                // e.g. if NUM_DUAL_SCALERS = 12, this is 8
                assign dual_scaler_expanded[i] = dual_scaler_expanded[i % (NUM_DUAL_SCALERS_EXPANDED>>1)];
            end
        end
    endgenerate

    localparam [31:0] INIT0 = 32'hAAAAAAAA; // 1010 repeating
    localparam [31:0] INIT1 = 32'hCCCCCCCC; // 1100 repeating
    localparam [31:0] INIT2 = 32'hF0F0F0F0; // 11110000 repeating
    localparam [31:0] INIT3 = 32'hFF00FF00; // 1111111100000000 repeating
    localparam [31:0] INIT4 = 32'hFFFF0000; // see above pattern
    // The D-input here is upshifted, so you write scaler number.
    RAM32X1S #(.INIT(INIT0)) u_bit0(.A0(wb_adr_i[2]),
                                    .A1(wb_adr_i[3]),
                                    .A2(wb_adr_i[4]),
                                    .A3(wb_adr_i[5]),
                                    .A4(wb_adr_i[6]),
                                    .D(wb_dat_i[1]),
                                    .O(scaler_addr[0]),
                                    .WE(wb_cyc_i && wb_stb_i && wb_we_i && wb_sel_i[0] && wb_adr_i[7] && !wb_adr_i[11]),
                                    .WCLK(clk_i));
    RAM32X1S #(.INIT(INIT1)) u_bit1(.A0(wb_adr_i[2]),
                                    .A1(wb_adr_i[3]),
                                    .A2(wb_adr_i[4]),
                                    .A3(wb_adr_i[5]),
                                    .A4(wb_adr_i[6]),
                                    .D(wb_dat_i[2]),
                                    .O(scaler_addr[1]),
                                    .WE(wb_cyc_i && wb_stb_i && wb_we_i && wb_sel_i[0] && wb_adr_i[7] && !wb_adr_i[11]),
                                    .WCLK(clk_i));
    RAM32X1S #(.INIT(INIT2)) u_bit2(.A0(wb_adr_i[2]),
                                    .A1(wb_adr_i[3]),
                                    .A2(wb_adr_i[4]),
                                    .A3(wb_adr_i[5]),
                                    .A4(wb_adr_i[6]),
                                    .D(wb_dat_i[3]),
                                    .O(scaler_addr[2]),
                                    .WE(wb_cyc_i && wb_stb_i && wb_we_i && wb_sel_i[0] && wb_adr_i[7] && !wb_adr_i[11]),
                                    .WCLK(clk_i));
    RAM32X1S #(.INIT(INIT3)) u_bit3(.A0(wb_adr_i[2]),
                                    .A1(wb_adr_i[3]),
                                    .A2(wb_adr_i[4]),
                                    .A3(wb_adr_i[5]),
                                    .A4(wb_adr_i[6]),
                                    .D(wb_dat_i[4]),
                                    .O(scaler_addr[3]),
                                    .WE(wb_cyc_i && wb_stb_i && wb_we_i && wb_sel_i[0] && wb_adr_i[7] && !wb_adr_i[11]),
                                    .WCLK(clk_i));
    RAM32X1S #(.INIT(INIT4)) u_bit4(.A0(wb_adr_i[2]),
                                    .A1(wb_adr_i[3]),
                                    .A2(wb_adr_i[4]),
                                    .A3(wb_adr_i[5]),
                                    .A4(wb_adr_i[6]),
                                    .D(wb_dat_i[5]),
                                    .O(scaler_addr[4]),
                                    .WE(wb_cyc_i && wb_stb_i && wb_we_i && wb_sel_i[0] && wb_adr_i[7] && !wb_adr_i[11]),
                                    .WCLK(clk_i));
    // Scaler *reads* need a state machine plus prescale writes
    localparam FSM_BITS = 2;
    localparam [FSM_BITS-1:0] IDLE = 0;
    localparam [FSM_BITS-1:0] WAIT_UPDATE = 1;
    localparam [FSM_BITS-1:0] WAIT_RESET = 2;
    localparam [FSM_BITS-1:0] ACK = 3;
    reg [FSM_BITS-1:0] state = IDLE;
    always @(posedge clk_i) begin 
        scaler_timer_writereg <= scaler_timer_write;
       
        scaler_data_out <= dual_scaler_expanded[ scaler_addr[ 0 +: NUM_DUAL_SCALERS_ADDR_BITS ] ];
        
        if (rst_i) state <= IDLE;
        else begin
            case(state)
                IDLE: if (wb_cyc_i && wb_stb_i) begin
                    // ack writes immediately
                    if (wb_we_i) begin
                        if (scaler_reset) state <= WAIT_RESET;
                        else state <= ACK;
                    end 
                    // control reads ack immediately
                    else if (!wb_adr_i[11]) state <= ACK;
                    // if scalers aren't about to update, ack immediately
                    else if (!scaler_updating) state <= ACK;
                    // otherwise wait for them to finish
                    else state <= WAIT_UPDATE;
                end
                WAIT_UPDATE: if (!scaler_updating) state <= ACK;
                WAIT_RESET: if (!scaler_resetting) state <= ACK;
                ACK: state <= IDLE;
            endcase
        end
        if (rst_i || scaler_timer_reset) scaler_updating <= 0;
        else if (scalers_now) scaler_updating <= 1;
        else if (scaler_update_done) scaler_updating <= 0;
        
        if (rst_i) scaler_resetting <= 1'b0;
        else if (scaler_reset) scaler_resetting <= 1'b1;
        else if (scaler_reset_done) scaler_resetting <= 1'b0;
        
        // Delay the actual update flag just to ensure the capture's OK.
        scaler_update <= { scaler_update[0], scalers_now };
        
        if (scaler_timer_write && wb_sel_i[3]) scaler_use_pps <= wb_dat_i[31];        
        
        if (scaler_timer_write) begin
            if (wb_sel_i[0]) scaler_period[7:0] <= wb_dat_i[7:0];
            if (wb_sel_i[1]) scaler_period[15:8] <= wb_dat_i[15:8];
            if (wb_sel_i[2]) scaler_period[23:16] <= wb_dat_i[23:16];
            if (wb_sel_i[3]) scaler_period[30:24] <= wb_dat_i[30:24];
        end
        
        if (wb_cyc_i && wb_stb_i && wb_we_i && !wb_adr_i[11] && !wb_adr_i[7] && wb_adr_i[2]) begin
            if (wb_sel_i[0]) scaler_prescale <= wb_dat_i[7:0];
            if (wb_sel_i[3]) scaler_update_addr <= wb_dat_i[24 +: NUM_SCALERS_ADDR_BITS];
        end        
    end

    assign scaler_reset = (state == IDLE && wb_cyc_i && wb_stb_i && wb_we_i && !wb_adr_i[11] && !wb_adr_i[7] && wb_adr_i[2] && wb_sel_i[3] );
    
    
    // We can just shove in wb input
    // because internally there's only a 1 clock delay, and above there's
    // also a 1 clock delay in acking.
    // Also, this is dirt slow, so we remap 
    dsp_counter_terminal_count #(.FIXED_TCOUNT("FALSE"),.RESET_TCOUNT_AT_RESET("FALSE"))
        u_scaltimer( .clk_i(clk_i),.rst_i(rst_i),.count_i(scaler_ce),.tcount_i(scaler_period_in),
                     .update_tcount_i(scaler_timer_writereg || initial_load),.tcount_reached_o(scaler_timer_update) );


    assign wb_ack_o = (state == ACK);
    assign wb_err_o = 1'b0;
    assign wb_rty_o = 1'b0;
    assign wb_dat_o = data_out;
    
endmodule
                                    