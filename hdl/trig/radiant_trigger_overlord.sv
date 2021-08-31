`timescale 1ns / 1ps
// This is the overall Trigger Governor for the RADIANT.
//
// Triggering for the RADIANT's a bit weird to understand, because things
// have kindof been mishmashed together. I'll try to clarify terminology here.
//
// 1) Call reading out a LAB4D a "readout."
// 2) Call a set of reading 1024 samples a "readout sequence."
// 3) A *trigger* therefore generates a header and (possibly multiple) readout sequences,
//    and triggers an automatic DMA sequence when complete (an "event").
//
// We need to maintain the ability to force a *readout sequence* - which is *not* the same
// thing as forcing a trigger. This is for things like CalRam pedestal runs, for instance.
//
// Sadly we called a "forced readout sequence" a "forced trigger" in the LAB4D control.
// This is SEPARATE from a "forced trigger" here, which actually generates a header.
// "Forced triggers" *used* to generate a DMA sequence, but we're getting rid of that
// now.
//
// So, for instance, you can trigger a "forced readout sequence" by doing the 'forced trigger'
// thing in the LAB4 controller, but that *will not* trigger a DMA. It'll accumulate data in
// the CalRam and possibly store data in the FIFO. Which you can then DMA out if you want to.
//
// The reason for doing this is that the trigger overlord keeps track of stuff like how much
// space there is available in the LAB4 FIFO and hopefully eventually whether or not there's
// an available window to switch to.
module radiant_trigger_overlord
        #( parameter EXT_LOGIC_TRUE = 0 )
        (
        input sys_clk_i,
        input pps_i,
        input int_trig_i,
        input [1:0] int_trig_type_i,
        // this is a sys_clk flag, generated from a trig_clk capture of ext_trig's rising edge
        input ext_trig_i,
        input soft_trig_i,
        output trig_o,
        // bit[1:0] int trig bit
        // bit[2] ext trig bit
        // bit[3] soft trig bit
        // bit[4] pps trig bit
        output [15:0] trig_info_o,
        output deadtrig_o,
        output ext_trig_o,
        output trig_done_o,

        // soft trigger flow control
        input  soft_flow_ctrl_i,
        // clear inhibit when in soft flow ctrl land
        input soft_flow_clr_i,
        // asserted when the only thing holding us back is the soft flow
        output soft_flow_waiting_o,
        // asserted when we're in a trigger
        output trigger_busy_o,
        // bit 0 : enable any
        // bit 1 : enable external trig
        // bit 2 : enable pps trig
        input [2:0] en_i,
        output [2:0] enabled_o,
        // bit 0 : enable any
        // bit 1 : enable soft trig
        // bit 2 : enable pps trig
        // internal triggers are always enabled if enable_any
        input [2:0] ext_en_i,
        // oneshot length
        input [4:0] ext_len_i,
    
        output dead_o,
        input readout_running_i,
        input readout_done_i,
        input readout_full_i
    );
    // actual internal enables
    reg [2:0] enables = {3{1'b0}};

    // indicator to hold that readout complete happened
    reg readout_done_seen = 0;
    
    // jeez this is a lot
    reg soft_inhibit = 0;
    // indicator that soft inhibit is only thing holding us back
    reg just_soft_inhibit_waiting = 0;

    // This goes high when any kind of trigger occurs,
    // and right now, it only goes down when readout_done_i goes low AND
    // readout_full_i is low.
    reg trigger_busy = 0;
    reg trigger_was_busy = 0;

    // this goes when an external trigger comes in and we're busy
    reg dead_trigger_happened = 0;    
    
    reg pps_trigger = 0;
    reg ext_pps_trigger = 0;
    reg ext_soft_trigger = 0;
        
    reg trigger = 0;
    (* IOB = "TRUE" *)
    reg ext_trigger = ~EXT_LOGIC_TRUE;
    
    reg ext_trigger_flag = 0;
    wire ext_trigger_clr;
    SRLC32E u_trigger_oneshot(.D(ext_trigger_flag),.A(ext_len_i),.CE(1'b1),.CLK(sys_clk_i),.Q(ext_trigger_clr));
        
    
    reg [5:0] trig_type = {6{1'b0}};
    reg ext_trig_rereg = 0;
    reg int_trig_rereg = 0;
    reg [1:0] int_trig_type_rereg = {2{1'b0}};
    reg pps_trig_rereg = 0;
    reg soft_trig_rereg = 0;
    
    reg trigger_rereg = 0;
    
    always @(posedge sys_clk_i) begin
        if (readout_running_i) enables <= en_i;
        else enables <= {3{1'b0}};

        if (!enables[0] || !trigger_busy) readout_done_seen <= 1'b0;
        else if (readout_done_i) readout_done_seen <= 1'b1;
    
        if (!soft_flow_ctrl_i || soft_flow_clr_i) soft_inhibit <= 1'b0;
        else if (trigger) soft_inhibit <= 1'b1;
        
        pps_trigger <= pps_i && enables[2];
        ext_pps_trigger <= pps_i && enables[2] && ext_en_i[2];
        ext_soft_trigger <= soft_trig_i && ext_en_i[1];
    
        ext_trig_rereg <= ext_trig_i && enables[1];
        pps_trig_rereg <= pps_trigger;
        int_trig_type_rereg <= int_trig_type_i;
        int_trig_rereg <= int_trig_i;
        soft_trig_rereg <= soft_trig_i;
        
        trigger_rereg <= trigger;
        
        if (trigger && !trigger_rereg) begin
            trig_type[1:0] <= int_trig_type_rereg;
            trig_type[2] <= ext_trig_rereg;
            trig_type[3] <= soft_trig_rereg;
            trig_type[4] <= pps_trig_rereg;
            trig_type[5] <= int_trig_rereg;
        end
        
        if (enables[0] && (pps_trigger || soft_trig_i || int_trig_i || (ext_trig_i && enables[1])) && !trigger_busy) trigger <= 1;
        else trigger <= 0;        
        
        if (enables[0] && ext_en_i[0] && (ext_pps_trigger || ext_soft_trigger || int_trig_i) && !trigger_busy) ext_trigger <= EXT_LOGIC_TRUE;
        else if (ext_trigger_clr || !ext_en_i[0] || !en_i[0]) ext_trigger <= ~EXT_LOGIC_TRUE;

        if (enables[0] && ext_en_i[0] && (ext_pps_trigger || ext_soft_trigger || int_trig_i) && !trigger_busy) ext_trigger_flag <= 1;
        else ext_trigger_flag <= 1'b0;
                                
        if (!enables[0] || (readout_done_seen && !readout_full_i && !soft_inhibit)) trigger_busy <= 1'b0;
        else if (trigger) trigger_busy <= 1;
        
        if (enables[0] && readout_done_seen && !readout_full_i) just_soft_inhibit_waiting <= trigger_busy;
        else just_soft_inhibit_waiting <= 1'b0;
        
        trigger_was_busy <= trigger_busy;

        if (ext_trig_i && enables[1] && trigger_busy) dead_trigger_happened <= 1;
        else dead_trigger_happened <= 1'b0;
    end
    
    assign dead_o = trigger_busy;
    assign deadtrig_o = dead_trigger_happened;
    assign trig_o = trigger;
    assign ext_trig_o = ext_trigger;
    assign trig_done_o = !trigger_busy && trigger_was_busy;
    
    assign trigger_busy_o = trigger_busy;
    assign soft_flow_waiting_o = just_soft_inhibit_waiting;
        
    assign enabled_o = enables;
    assign trig_info_o = trig_type;
endmodule
