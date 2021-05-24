`timescale 1ns / 1ps
`include "dsp_macros.vh"
// "oneshot_i" = 0 here implies a 17.5 ns coincidence
// window, whereas "oneshot_i" = (all 1s) implies 
// 327.5 ns window.
//
// note: en_i, oneshot_i and threshold_i are functionally
// asynchronous, they're captured effectively on the falling edge
// of rst_i
module radiant_trigger(
            input [23:0] trig_i,
            input [23:0] en_i,
            input [19:0] oneshot_i,
            input rst_i,
            input trig_clk_i,
            input [4:0] threshold_i,
            output trigger_o
    );
    
    // The basic RADIANT trigger module acts as a 24-way add
    // of all of the enabled inputs for it.
    //
    // We rely on speed - we're running at 400 MHz, and we do
    // 1 clock: if input is enabled, capture and start oneshot. 
    // 2 clock: 4x 6->3 carry-save reductions (ok really just an add) -> A/B/C/D 3-bit values
    // 3 clock: add A/B, add C/D, reroute to input of DSP
    // 4 clock: add AB+CD, reroute to input of DSP
    // 5 clock: compute popcount-ABCD. carry out is trigger
    // 6 clock: fan out carry
    //
    // The oneshot generator works by
    // input -> set FF if enabled
    //       -> toggle indicator if enabled and FF not set
    //                 -> through SRL
    //                      -> XOR output of SRL and indicator
    //                             -> clears FF
    // this is a *non-retriggerable oneshot*. Singles rates need to be low enough that occupancy
    // isn't an issue.
    // the delay line itself is a bit tricky: we chain 4x SRLC32s together and user specifies
    // *all* delays as a 20-bit value. This gives us up to ~320 ns plus a fair amount extra.
    // I ain't doin' the split up in hardware, the math is easy.
    reg [23:0]  trigger_in = {24{1'b0}};
    reg [23:0]  trigger_toggle = {24{1'b0}};
    wire [23:0] trigger_toggle_out;
    reg [23:0] trigger_toggle_out_reg = {24{1'b0}};
    reg [23:0] trigger_toggle_out_rereg = {24{1'b0}};
    reg [23:0]  trigger_clear = {24{1'b0}};
    // limit this guy's max fanout and let duplication handle it
    (* KEEP = "TRUE" *)
    (* MAX_FANOUT = 8 *)
    reg local_reset = 0;
    always @(posedge trig_clk_i) local_reset <= rst_i;
    reg [23:0] trigger_en = {24{1'b0}};
    
    generate
        genvar i;
        for (i=0;i<24;i=i+1) begin : OS
            wire [2:0] delay_chain;
            (* KEEP = "TRUE" *)
            reg oneshot_reset = 0;
            reg [19:0] oneshot = {20{1'b0}};
            always @(posedge trig_clk_i) begin : OSLGC
                oneshot_reset <= local_reset;
                // capture when trigger first enabled, and only then.
                // The trick here is that local reset goes first
                // so we go
                // lr: 0001111000
                // or: 0000111100
                // and so the oneshot values are captured the first clock after
                // but this remains a FF only input - no LUTs - since we're
                // treating it as a sync reset and clock enable

                if (local_reset) oneshot <= {20{1'b0}};
                else if (oneshot_reset)  oneshot <= oneshot_i;
                
                // same trick
                if (local_reset) trigger_en[i] <= 1'b0;
                else if (oneshot_reset) trigger_en[i] <= en_i[i];
                
                if (trigger_clear[i] || !trigger_en[i] || oneshot_reset) trigger_in[i] <= 0;
                else if (trig_i[i]) trigger_in[i] <= 1;
                
                if (!trigger_en[i] || oneshot_reset) trigger_toggle[i] <= 1'b0;
                else if (trig_i[i] && !trigger_in[i]) trigger_toggle[i] <= ~trigger_toggle[i];

                trigger_toggle_out_reg[i] <= trigger_toggle_out[i];
                trigger_toggle_out_rereg[i] <= trigger_toggle_out_reg[i];
                
                trigger_clear[i] <= trigger_toggle_out_reg[i] ^ trigger_toggle_out_rereg[i];
            end
            SRLC32E u_delayA(.CLK(trig_clk_i),.A(oneshot[0 +: 5]),.CE(1'b1),.D(trigger_toggle[i]),.Q(delay_chain[0]));
            SRLC32E u_delayB(.CLK(trig_clk_i),.A(oneshot[5 +: 5]),.CE(1'b1),.D(delay_chain[0]),.Q(delay_chain[1]));
            SRLC32E u_delayC(.CLK(trig_clk_i),.A(oneshot[10+: 5]),.CE(1'b1),.D(delay_chain[1]),.Q(delay_chain[2]));
            SRLC32E u_delayD(.CLK(trig_clk_i),.A(oneshot[15+: 5]),.CE(1'b1),.D(delay_chain[2]),.Q(trigger_toggle_out[i]));
        end
    endgenerate
    
    // OK, so now we have 24 gated trigger inputs. Add 'em up.
    wire [2:0] stage1_sum[3:0];
    fast_csa63_adder #(.NBITS(1)) u_add0(.CLK(trig_clk_i),.CE(1'b1),.RST(1'b0),.A(trigger_in[0]),
                                                                               .B(trigger_in[1]),
                                                                               .C(trigger_in[2]),
                                                                               .D(trigger_in[3]),
                                                                               .E(trigger_in[4]),
                                                                               .F(trigger_in[5]),
                                                                               .SUM(        stage1_sum[0][0]),
                                                                               .CARRY(      stage1_sum[0][1]),
                                                                               .CCARRY( stage1_sum[0][2]));
    fast_csa63_adder #(.NBITS(1)) u_add1(.CLK(trig_clk_i),.CE(1'b1),.RST(1'b0),.A(trigger_in[6]),
                                                                               .B(trigger_in[7]),
                                                                               .C(trigger_in[8]),
                                                                               .D(trigger_in[9]),
                                                                               .E(trigger_in[10]),
                                                                               .F(trigger_in[11]),
                                                                               .SUM(        stage1_sum[1][0]),
                                                                               .CARRY(      stage1_sum[1][1]),
                                                                               .CCARRY( stage1_sum[1][2]));
    fast_csa63_adder #(.NBITS(1)) u_add2(.CLK(trig_clk_i),.CE(1'b1),.RST(1'b0),.A(trigger_in[12]),
                                                                               .B(trigger_in[13]),
                                                                               .C(trigger_in[14]),
                                                                               .D(trigger_in[15]),
                                                                               .E(trigger_in[16]),
                                                                               .F(trigger_in[17]),
                                                                               .SUM(        stage1_sum[2][0]),
                                                                               .CARRY(      stage1_sum[2][1]),
                                                                               .CCARRY( stage1_sum[2][2]));
    fast_csa63_adder #(.NBITS(1)) u_add3(.CLK(trig_clk_i),.CE(1'b1),.RST(1'b0),.A(trigger_in[18]),
                                                                               .B(trigger_in[19]),
                                                                               .C(trigger_in[20]),
                                                                               .D(trigger_in[21]),
                                                                               .E(trigger_in[22]),
                                                                               .F(trigger_in[23]),
                                                                               .SUM(        stage1_sum[3][0]),
                                                                               .CARRY(      stage1_sum[3][1]),
                                                                               .CCARRY( stage1_sum[3][2]));

    // Now the DSP. Since the opmode needs to be the same for each, the popcount threshold gets shoved in inverted.
    // So for instance a threshold of 0 (actually 1) is FFF, and *any* enabled input will trip carry. Etc.
    wire [11:0] dspAB_in[3:0];
    wire [11:0] dspC_in[3:0];
    wire [47:0] dspAB = { dspAB_in[3], dspAB_in[2], dspAB_in[1], dspAB_in[0] };
    wire [29:0] dspA = dspAB[18 +: 30];
    wire [17:0] dspB = dspAB[0 +: 18];
    wire [47:0] dspC =  { dspC_in[3], dspC_in[2], dspC_in[1], dspC_in[0] };
    wire [11:0] dspP_out[3:0];
    wire [47:0] dspP;
    reg [4:0] threshold = {5{1'b0}};
    assign dspP_out[0] = dspP[0     +: 12];
    assign dspP_out[1] = dspP[12    +: 12];
    assign dspP_out[2] = dspP[24    +: 12];
    assign dspP_out[3] = dspP[36    +: 12];    
    assign dspAB_in[0] = { {9{1'b0}}, stage1_sum[0] };
    assign dspAB_in[1] = { {9{1'b0}}, stage1_sum[2] };
    assign dspC_in[0] =  { {9{1'b0}}, stage1_sum[1] };
    assign dspC_in[1] =  { {9{1'b0}}, stage1_sum[3] };
    assign dspAB_in[2] = dspP_out[0];
    assign dspC_in[2] = dspP_out[1];
    assign dspAB_in[3] = dspP_out[2];
    assign dspC_in[3] = { {7{1'b1}}, ~threshold };
    wire [3:0] carryout;
    (* KEEP = "TRUE" *)
    reg dsp_reset = 0;
    always @(posedge trig_clk_i) begin
        dsp_reset <= local_reset;
        // same trick as above
        // we can't do this to the DSP registers because they're SIMD and the clock enables
        // are common
        if (local_reset) threshold <= {5{1'b0}};
        else if (dsp_reset) threshold <= threshold_i;        
    end
    DSP48E1 #( .AREG(1),.BREG(1),.CREG(1),.PREG(1),.CARRYINREG(1'b0), `D_UNUSED_ATTRS, `CONSTANT_MODE_ATTRS, `NO_MULT_ATTRS,
               .USE_PATTERN_DETECT("NO_PATDET"),.USE_SIMD("FOUR12")
                 )
            u_counter( `D_UNUSED_PORTS,
                        // sigh
                        .RSTINMODE(1'b0),.CECTRL(1'b0),
                        .RSTALUMODE(1'b0),.CEALUMODE(1'b0),.CEM(1'b0),
                        .CECARRYIN(1'b0),.CEB1(1'b0),.CEA1(1'b0),
                        .CARRYIN(1'b0),
                        .CARRYINSEL(3'h0),                        
                       .A( dspA ), .CEA2(1'b1), .RSTA(dsp_reset),
                       .B( dspB ), .CEB2(1'b1), .RSTB(dsp_reset),
                       .C( dspC ), .CEC(1'b1), .RSTC(dsp_reset),                       
                       .CEP(1'b1),.RSTP(dsp_reset),
                       .OPMODE({ `Z_OPMODE_0, `Y_OPMODE_C, `X_OPMODE_AB }),
                       .ALUMODE(`ALUMODE_SUM_ZXYCIN),
                       .INMODE( {5{1'b0}} ),
                       .CLK(trig_clk_i),
                       .P( dspP ),
                       .CARRYOUT( carryout ) );

    reg trigger = 0;
    always @(posedge trig_clk_i) trigger <= carryout[`QUAD_DSP_CARRY3];

    assign trigger_o = trigger;                     
endmodule
