`timescale 1ns / 1ps
// Trigger oneshot.
module radiant_trig_oneshot( input fast_clk_i,
                             input slow_clk_i,                             
                             input rst_i,                             
                             input trigger_i,
                             input stuck_ce_i,
                             output trig_o,
                             output scal_o
    );

    // I AM TOTALLY RELYING ON SPEED HERE, FOLKS
    // I'm clocking the input DDR, so I'll catch ANYTHING that's up for at least 1.25 ns.
    // That should be fine (oh... I hope). If I *really* need to I'll redo this with
    // an OSERDES and bump it up to Something Stupid.
    
    // SAME_EDGE mode here means the negative edge is "first", not that I care.
    wire [1:0] trigger_in;
    IDDR #(.DDR_CLK_EDGE("SAME_EDGE"),.SRTYPE("ASYNC")) u_input(.D(trigger_i),.C(fast_clk_i),.CE(1'b1),.R(rst_i),
                                                                .Q1(trigger_in[1]),
                                                                .Q2(trigger_in[0]));
    reg [1:0] trigger = {2{1'b0}};
    reg trigger_flag = 0;
    reg trigger_or = 0;
    wire trigger_clear;
    reg [1:0] scaler_regs = {2{1'b0}};
    reg scaler_flag = 0;
    wire scaler_busy;
    (* ASYNC_REG = "TRUE" *)
    reg [1:0] trigger_resync = {2{1'b0}};
    reg [3:0] stuck_check = {4{1'b0}};
    wire scal_raw;
    reg scaler_out = 0;
    flag_sync u_scaler_sync(.in_clkA(scaler_flag),.out_clkB(scal_raw),.clkA(fast_clk_i),.clkB(slow_clk_i),.busy_clkA(scaler_busy));
    always @(posedge fast_clk_i) begin
        // also acts to synchronize
        trigger_or <= (trigger_in[1] || trigger_in[0]);
        trigger <= { trigger[0], trigger_or };
        trigger_flag <= trigger_or && !trigger[0];        

        scaler_regs <= { scaler_regs[0], trigger[1] };
        scaler_flag <= (scaler_regs == 2'b01) && !scaler_busy;                
    end

    always @(posedge slow_clk_i) begin
        trigger_resync <= { trigger_resync[0], trigger[1] };
        if (!trigger_resync[1]) stuck_check <= {4{1'b0}};
        else stuck_check <= { stuck_check[2:0], trigger_resync[1] };
        scaler_out <= stuck_check[3] || scal_raw;        
    end

    assign trig_o = trigger_flag;
    assign scal_o = scaler_out;
endmodule
