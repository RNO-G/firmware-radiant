`timescale 1ns / 1ps
`include "wishbone.vh"
module radiant_overlord_core #(parameter NUM_CH = 24)(
        // register side
        input clk_i,
        input rst_i,
        `WBS_NAMED_PORT(wb, 32, 9, 4),
        // full threshold output
        output [11:0] readout_full_thresh_o,
        
        // trigger side, this stuff is all in radiant_trigger_overlord
        input sys_clk_i,
        // sysclk pps flag
        input pps_i,
        // actual trigger output to the LAB4 trigger module
        output trig_o,
        // eventually this will push to the event core to make sure that a (dead) header gets
        // generated from each trigger occurring during a dead period
        output deadtrig_o,
        // this goes to the trigger output
        output ext_trig_o,
        // and this goes to the event builder to push out a real event
        output trig_done_o,
        // Internal trigger flag
        input int_trig_i,
        // External trigger flag
        input ext_trig_i,
        // Indicates LAB4 controller is actually running.
        input readout_running_i,
        // This input comes from the LAB4 controller        
        input readout_done_i,
        // this input comes from the LAB4 FIFO
        input [NUM_CH-1:0] readout_full_i
    );
    
    reg [2:0] overlord_trig_en = {3{1'b0}};
    wire [2:0] overlord_trig_enabled;
    reg [2:0] overlord_ext_en = {3{1'b0}};
    reg overlord_soft_inhibit = 0;
    reg [4:0] overlord_ext_len = {5{1'b0}};
    // our full threshold is
    // 0 = 3073 (C01)
    // 1 = 2049 (801)
    // 2 = 1025 (401)
    // 3 = 0001 (001)
    // which obviously converts into just 2 bits
    reg [1:0] overlord_num_bufs = {2{1'b0}};
    assign readout_full_thresh_o = { ~overlord_num_bufs, 10'h001 };
    wire [31:0] overlord_config_reg = { 3'b000, overlord_ext_len, {5{1'b0}}, overlord_num_bufs, overlord_soft_inhibit , {5{1'b0}}, overlord_ext_en, {5{1'b0}}, overlord_trig_enabled };    
    reg overlord_soft_trig = 0;
    reg overlord_soft_clr = 0;
    wire [31:0] overlord_control_reg = {32{1'b0}};
    
    wire trigger_busy_sysclk;
    (* ASYNC_REG = "TRUE" *)
    reg [1:0] trigger_busy = {2{1'b0}};
    wire overlord_waiting_soft_clr_sysclk;
    (* ASYNC_REG = "TRUE" *)
    reg [1:0] overlord_waiting_soft_clr = {2{1'b0}};
    wire [31:0] overlord_status_reg = { {30{1'b0}}, overlord_waiting_soft_clr[1], trigger_busy[1] };
    
    // Make fifo_mask_reg fixed size, we'll let optimization take care of it
    reg [31:0] fifo_mask_reg = {32{1'b0}};
    wire [NUM_CH-1:0] fifo_mask = fifo_mask_reg[0 +: NUM_CH];
    wire [31:0] overlord_chcfg_reg = { {(32-NUM_CH){1'b0}}, fifo_mask };
    
    wire readout_full = |(readout_full_i & ~fifo_mask);
    
    // register 0x00: CONFIG
    // bits[2:0] overlord trigger enable. Bit[0] = master enable. Bit[1] = external trigger enable Bit[2] pps enable
    // bits[10:8] external trigger output. Bit[0] = enable any. Bit[1] = enable soft trig. Bit [2] enable pps output.
    // bit[16] : CPU soft flow control. One trigger, then inhibit until you clear.
    // bits[18:17]: number of buffers per event. Needed so overlord can check FIFO.
    // bits[31:24] : 
    // register 0x04: CTRLSTATUS
    // on a write:
    // bit[0] : soft trigger
    // bit[1] : clear soft inhibit
    // on a read:
    // bit[0] = trigger busy
    // bit[1] = waiting on a soft trigger clear
    // extra stuff we'll work on later
    // register 0x08: CHANCFG
    // bits[23:0] - if *set*, the FIFO full status of these inputs are *ignored*.
    
    // all of our registers are actually in SYSCLK domain so we'll just state machine it
    localparam FSM_BITS=2;
    localparam [FSM_BITS-1:0] IDLE = 0;
    localparam [FSM_BITS-1:0] WRITE_SYSCLK = 1;
    localparam [FSM_BITS-1:0] WAIT_WRITE = 2;
    localparam [FSM_BITS-1:0] ACK = 3;
    reg [FSM_BITS-1:0] state = IDLE;

    wire write_clk = (state == WRITE_SYSCLK);
    wire write_sysclk;
    wire write_ack_clk;
    flag_sync u_sync(.in_clkA(write_clk),.clkA(clk_i),.out_clkB(write_sysclk),.clkB(sys_clk_i));
    flag_sync u_ack(.in_clkA(write_sysclk),.clkA(sys_clk_i),.out_clkB(write_ack_clk),.clkB(clk_i));

    reg [31:0] wb_dat_out = {32{1'b0}};
    
    wire [31:0] wb_dat_mux[3:0];
    assign wb_dat_mux[0] = overlord_config_reg;
    assign wb_dat_mux[1] = overlord_status_reg;
    assign wb_dat_mux[2] = overlord_chcfg_reg;
    assign wb_dat_mux[3] = wb_dat_mux[1];
    
    always @(posedge clk_i) begin
        trigger_busy <= { trigger_busy[0], trigger_busy_sysclk };
        overlord_waiting_soft_clr <= { overlord_waiting_soft_clr[0], overlord_waiting_soft_clr_sysclk };
        
        
    
        if (rst_i) state <= IDLE;
        else case(state)
            IDLE: if (wb_cyc_i && wb_stb_i) begin
                    if (wb_we_i) state <= WRITE_SYSCLK;
                    else state <= ACK;
                  end                  
            WRITE_SYSCLK: state <= WAIT_WRITE;
            WAIT_WRITE: if (write_ack_clk) state <= ACK;
            ACK: state <= IDLE;
        endcase
        if (wb_cyc_i && wb_stb_i && !wb_we_i) begin
            wb_dat_out <= wb_dat_mux[wb_adr_i[3:2]];
        end
    end
    always @(posedge sys_clk_i) begin
        if (write_sysclk && {wb_adr_i[4:2],2'b00} == 5'h00) begin
            if (wb_sel_i[0]) overlord_trig_en <= wb_dat_i[0 +: 3];
            if (wb_sel_i[1]) overlord_ext_en <= wb_dat_i[8 +: 3];
            if (wb_sel_i[2]) begin
                overlord_soft_inhibit <= wb_dat_i[16];
                overlord_num_bufs <= wb_dat_i[18:17];
            end
            if (wb_sel_i[3]) overlord_ext_len <= wb_dat_i[24 +: 5];
        end
        if (write_sysclk && {wb_adr_i[4:2],2'b00} == 5'h04) begin
            if (wb_sel_i[0]) begin
                overlord_soft_trig <= wb_dat_i[0];
                overlord_soft_clr <= wb_dat_i[1];
            end
        end else begin
            overlord_soft_trig <= 1'b0;
            overlord_soft_clr <= 1'b0;
        end
        
        if (write_sysclk && {wb_adr_i[4:2],2'b00} == 5'h08) begin
            if (wb_sel_i[0]) fifo_mask_reg[7:0] <= wb_dat_i[7:0];
            if (wb_sel_i[1]) fifo_mask_reg[15:8] <= wb_dat_i[15:8];
            if (wb_sel_i[2]) fifo_mask_reg[23:16] <= wb_dat_i[23:16];
            if (wb_sel_i[3]) fifo_mask_reg[31:24] <= wb_dat_i[31:24];
        end
    end

    // I dunno do something.
    wire trigger_dead;

    radiant_trigger_overlord u_overlord(.sys_clk_i(sys_clk_i),
                                        .pps_i(pps_i),
                                        .int_trig_i(int_trig_i),
                                        .ext_trig_i(ext_trig_i),
                                        .soft_trig_i(overlord_soft_trig),
                                        .trig_o(trig_o),
                                        .deadtrig_o(deadtrig_o),
                                        .ext_trig_o(ext_trig_o),
                                        .trig_done_o(trig_done_o),
                                        .soft_flow_ctrl_i(overlord_soft_inhibit),
                                        .soft_flow_clr_i(overlord_soft_clr),
                                        .soft_flow_waiting_o(overlord_waiting_soft_clr_sysclk),
                                        .en_i(overlord_trig_en),
                                        .enabled_o(overlord_trig_enabled),
                                        .ext_en_i(overlord_ext_en),
                                        .ext_len_i(overlord_ext_len),
                                        .dead_o(trigger_dead),
                                        .trigger_busy_o(trigger_busy_sysclk),
                                        .readout_running_i(readout_running_i),
                                        .readout_done_i(readout_done_i),
                                        .readout_full_i(readout_full));

    assign wb_ack_o = (state == ACK);
    assign wb_dat_o = wb_dat_out;
    assign wb_err_o = 1'b0;
    assign wb_rty_o = 1'b0;
endmodule
