`timescale 1ns/1ps

// Pedestal and zero-crossing calculator for a LAB4D.
//
// This works by using 3x36 KiB block RAMs to store both pedestal data
// and zero-crossing data in special acquisition modes.
//
// The 36 KiB block RAMs are organized as 9x4096, and connected to the
// output of 2 DSPs.
//
// In pedestal mode, DSPA is used for summation, and DSPB is used only
// as a pass-through. The input is stored, and the value is fetched from
// RAM. The low 9 bits are fed into the A-port and pre-added to the
// data, whereas the top 18 bits are added at the C-port. The current value
// is fed to the D port. (The A input is cascaded up to DSPB for use
// in zero-crossing mode). The B input (multiplier) is just set to 1 in this mode.
//
// DSPA in Ped/ZC mode uses ALUMODE = 0000
// DSPA in Ped mode uses INMODE =  0100 (in ZC mode uses 0110)
// DSPA in Ped/ZC mode uses OPMODE = 0110101 (same in ZC mode)
// DSPA needs AREG=1, ACASCREG=1, DREG=1, BREG=1, CREG=1, CARRYINSEL=000, CARRYINSELREG=0,PREG=0,
//            ADREG=0, MREG=0
//
// The high BRAMs write 4 clocks after dat_wr, and the low BRAM clocks 5 clocks after.
//
//
// In zero-crossing mode, we want to count the number of *rising edges*. These are
// cases where the previous value was less than or equal to zero, and next was more than zero.
// In order to do that, we need to do (ped - value). In this case P[47] is set if greater.
// To get ped (C) - value (D*B) we need ALUMODE = 0011.
//
// To do this right, we want (nonpositive) -> (positive). So what we do is flip the previous P[47]
// and feed it into CARRYIN, so it'll count up if the first one's nonpositive. Then we qualify
// the write by DSPA's P[47] in ZC mode.
//
// 
// Here we assume the input goes from  "1000->1001" and the pedestal is "1000" in both cases (output as 512000)
// clk DSPA_D DSPA_Dreg DSPA_C DSPA_Creg DSPA_A DSPA_Areg DSPB_Areg DSPB_Creg DSPA_PC47=DSPB_PCIN47              DSPB_Pin
// 0   1000   X         512000 X         accum0 X         X         X         X                                  X
// 1   1001   1000      512000 512000    accum1 accum0    X         X         X                                  X
// 2   ...    1001      ...    512000    accum2 accum1    X         X         0                                  X
// 3   ...    ...       ...    ...       accum3 accum2    accum0    0         1                                  -1+((accum0+1)*1+1) = accum0+1
//
// Contrast with NO rising edge:
// clk DSPA_D DSPA_Dreg DSPA_C DSPA_Creg DSPA_A DSPA_Areg DSPB_Areg DSPB_Creg DSPA_PC47=DSPB_PCIN47              DSPB_Pin
// 0   1001   X         512000 X         accum0 X         X         X         X                                  X
// 1   1001   1001      512000 512000    accum1 accum0    X         X         X                                  X
// 2   ...    1001      ...    512000    accum2 accum1    X         X         1                                  X
// 3   ...    ...       ...    ...       accum3 accum2    accum0    1         1                                  -2+((accum0+1)*1+1) = accum0
// (note this is because ~1 = -2)
//
// So DSPB in ZC mode uses ALUMODE = 0001       (and 0000 in normal mode).
//    DSPB in ZC mode uses  INMODE = 0100       (and 0000 in normal mode).
//    DSPB in ZC mode uses  OPMODE = 0110101    (and 0010000 in normal mode).
//    DSPB in ZC mode uses CARRYINSEL = 011     (and 000 in normal mode)
// DSPB needs AREG = 2, DREG = 0, CREG = 1, CARRYINSELREG = 1
//
// DSPB also sets PATTERNDETECT to 0xFF to indicate when a ZC run has to terminate.
//
// AMAZING SLEAZE!
//
// Resetting the DSPs can be done by writing to the magic write address of 31 (which selects all of them).
// for all 4096 values. Yes, tedious, but simplest/least resource intensive method.
//
module calram_pedestal  #(parameter LAB4_BITS=12,
                          parameter DEBUG = "FALSE")
                         ( input                 sys_clk_i,
                           // Inputs from the LAB4 readout.
                           input [LAB4_BITS-1:0] lab_dat_i,
                           input                 lab_wr_i,
                           input [11:0]          lab_adr_i,
                           // Inputs from control.
                           input                 en_i,
                           input                 config_wr_i,
                           input                 zc_mode_i,
                           output                zc_full_o,
                           // This becomes the RSTP input on the DSPs.
                           // If you set this and run a full 4096 in pedestal mode, all 3 BRAMs zero automatically.
                           // If you set this and run a full 4096 in zc mode, the ZC BRAM (low bits) zero automatically.
                           input                 zero_i,
                           // done_o goes high when we're done processing a sample.
                           output                done_o,
                           // clk_i side (RAM interface)                           
                           input                 clk_i,
                           input                 bram_en_i,
                           input                 bram_wr_i,
                           output                ack_o,
                           input [11:0]          adr_i,
                           input  [26:0]         dat_i,
                           output [26:0]         dat_o );

    // The pedestal calram takes in the data as they come in, fetches the address from RAM,
    // increments it, and writes it back in.
    // Nice thing about this is that it's basically just a DSP connected to a dual-port RAM.
    // RAM's data in => DSP data out.
    // DSP   data in => RAM data out
    // RAM's addr in => dat_adr_i
    reg [4:0] bram_en_delay = {5{1'b0}};    
    wire      dspA_valid = bram_en_delay[3];

    reg       ped_mode = 0;
    reg       bram_hwr = 0;
    reg       bram_lwr = 0;
    wire      zc_positive;
    
    reg       zc_full = 0;
    wire      dspB_match;
    // This is just impressively dumb, there's got to be a simpler way to do this.
    // Tried to do it by screwing with the DSP, but can't see a way to do it
    // easily because the OPMODE/CARRYINSEL clock enables are common.
    // Maybe if I killed OPMODE's register and just special-timed it against
    // ped_mode. Not sure.
    always @(posedge sys_clk_i) begin
        // store ped mode or not
        if (config_wr_i) ped_mode <= !zc_mode_i;

        // OK, we need to sequence things to capture the pipeline delays.
        bram_en_delay <= { bram_en_delay[3:0], lab_wr_i && en_i };        
        // dspA becomes valid at bram_en_delay[3]
        bram_hwr <= bram_en_delay[2] && ped_mode;
        // dspB is valid right after, but we only count if positive OR if we're zeroing. 
        bram_lwr <= bram_en_delay[3] && (ped_mode || zc_positive || zero_i);
        
        if (config_wr_i) zc_full <= 0;
        else if (dspB_match && bram_lwr) zc_full <= 1;
    end
    assign done_o = bram_en_delay[4];

    localparam FSM_BITS=3;
    localparam [FSM_BITS-1:0] IDLE = 0;
    localparam [FSM_BITS-1:0] READ_0 = 1;   // en asserts on read
    localparam [FSM_BITS-1:0] READ_1 = 2;   // regce asserts
    localparam [FSM_BITS-1:0] READ_ACK = 3;      // ack asserts on read
    localparam [FSM_BITS-1:0] WRITE_ACK = 4; // en, we, ack asserts on write
    reg [FSM_BITS-1:0] state = IDLE;
    always @(posedge clk_i) begin
        case (state)
            IDLE: if (bram_en_i) begin
                if (bram_wr_i) state <= WRITE_ACK;
                else state <= READ_0;
            end
            READ_0: state <= READ_1;
            READ_1: state <= READ_ACK;
            READ_ACK: if (!bram_en_i) state <= IDLE;
            WRITE_ACK: if (!bram_en_i) state <= IDLE;
        endcase
    end

    wire bramif_en = (state == READ_0 || state == WRITE_ACK);
    wire bramif_wr = (state == WRITE_ACK);
    wire bramif_regce = (state == READ_1);
    
    wire [47:0] dspA_out;    
    assign zc_positive = dspA_out[47];
    wire [3:0]  dspA_carryout;
    wire [47:0] dspB_out;
    wire [47:0] dspA_cascP_dspB;
    wire [29:0] dspA_cascA_dspB;
    wire        dspA_carrycasc_dspB;
    
    // all of these are captured at config_wr_i, which goes
    // to:
    // dspA: CECTRL (opmode/carryinsel)
    // dspB: CECTRL/CEALUMODE/CEINMODE
    
    wire [4:0] dspA_inmode = { 2'b00, 1'b1, zc_mode_i, 1'b0 };
    wire [6:0] dspA_opmode = 7'b0110101;
    wire [3:0] dspA_alumode = { 2'b00, zc_mode_i, zc_mode_i };
    wire [2:0] dspA_carryinsel = 3'b000;
    // dspA's B input is either 1 (ped mode) or 512 (zc mode).
    // Thought about letting it be something else, but doesn't make sense.
    wire [17:0] dspA_b = { {8{1'b0}}, zc_mode_i, {8{1'b0}}, !zc_mode_i };

    // This is like, the fifteenth time I've done this.
    wire [4:0] dspB_inmode = 5'b00000;                                          // always just A2
    wire [6:0] dspB_opmode = { 1'b0, zc_mode_i, 1'b1, 1'b0, zc_mode_i, 1'b0, zc_mode_i };      // either Z=PCIN, X/Y=0 or Z=C, X/Y = mult out
    wire [3:0] dspB_alumode = 4'b0000;                                                         // always Z+X+Y+CIN
    wire [2:0] dspB_carryinsel = 3'b000;                                                       // always CARRYIN (never used)

    wire [17:0] dspB_b = 18'd1;
    wire [47:0] dspB_c = { {46{1'b0}}, ~dspA_out[47] };
    
    wire [8:0] bram_outA[2:0];
    wire [2:0] bram_wea = { bram_hwr, bram_hwr, bram_lwr };
    
    wire [8:0] bram_inA[2:0];
    assign bram_inA[0] = dspB_out[0 +: 9];
    assign bram_inA[1] = dspA_out[9 +: 9];
    assign bram_inA[2] = dspA_out[18 +: 9];
    
    // OK, so here are our 3 BRAMs.
    generate
        genvar i;
        for (i=0;i<3;i=i+1) begin : BRAM
            BRAM_TDP_MACRO #(.DOA_REG(1), .BRAM_SIZE("36Kb"),.READ_WIDTH_A(9),.WRITE_WIDTH_A(9),.WRITE_MODE_A("WRITE_FIRST"),
                             .DOB_REG(1), .READ_WIDTH_B(9),.WRITE_WIDTH_B(9),.WRITE_MODE_B("WRITE_FIRST"))
                        bram( .RSTA(1'b0),.RSTB(1'b0),
                              .DIA(bram_inA[i]),.ADDRA(lab_adr_i[11:0]),.WEA(bram_wea[i]),.ENA(en_i),.DOA(bram_outA[i]),.REGCEA(en_i),.CLKA(sys_clk_i),
                              .DIB(dat_i[9*i +: 9]),.ADDRB(adr_i[11:0]),.WEB(bramif_wr),.ENB(bramif_en),.REGCEB(bramif_regce),.DOB(dat_o[9*i +: 9]),.CLKB(clk_i));
        end
    endgenerate
    // DSP A: this is the pedestal builder.
    // DSPA needs AREG=1, ACASCREG=1, DREG=1, BREG=1, CREG=1, CARRYINSEL=000, CARRYINSELREG=0,PREG=1,
    //            ADREG=0, MREG=0
    DSP48E1 #(.ADREG(0),.AREG(1),.ACASCREG(1),.DREG(1),.BREG(1),.BCASCREG(1),.ALUMODEREG(0),.INMODEREG(1),.MREG(0),.PREG(1),.CREG(1),.OPMODEREG(0),
              .CARRYINREG(0),.CARRYINSELREG(0),
              .USE_DPORT("TRUE"),
              .USE_MULT("MULTIPLY")) 
              u_dspA(  .A({ {21{1'b0}}, bram_outA[0] }),.D( { {13{1'b0}}, lab_dat_i }),.B(dspA_b),.C({{21{1'b0}},bram_outA[2],bram_outA[1],{9{1'b0}}}),.CEC(en_i),.CEB2(config_wr_i),
                       .RSTA(1'b0),.RSTB(1'b0),.RSTC(1'b0),.RSTD(1'b0),.RSTCTRL(1'b0),.RSTINMODE(1'b0),.RSTM(1'b0),.RSTP(zero_i),
                       .ACOUT(dspA_cascA_dspB),
                       .PCOUT(dspA_cascP_dspB),
                       .CARRYIN(0),
                       .INMODE(dspA_inmode),.CEINMODE(config_wr_i),
                       .OPMODE(dspA_opmode),.ALUMODE(dspA_alumode),.CARRYINSEL(dspA_carryinsel),
                       .CEA2(en_i),.CED(lab_wr_i),.CEP(en_i),
                       .P(dspA_out),
                       .CLK(sys_clk_i));

    // OPMODE's the only screwy one.
    DSP48E1 
            #(.ADREG(0),.AREG(1),.DREG(0),.BREG(0),.BCASCREG(0),.ALUMODEREG(0),.INMODEREG(0),.OPMODEREG(1),.CARRYINSELREG(0),
              .MREG(0),.PREG(1),.CREG(1),.CARRYINREG(0),
              .A_INPUT("CASCADE"),
              .USE_PATTERN_DETECT("PATDET"),
              .SEL_PATTERN("PATTERN"),
              .PATTERN(48'd511),
              .MASK(48'd0),
              .SEL_MASK("MASK"),
              .USE_DPORT("TRUE"),
              .USE_MULT("MULTIPLY")) 
              u_dspB( .ACIN(dspA_cascA_dspB),
                      .PCIN(dspA_cascP_dspB),
                      .CARRYIN(1'b0),
                      .B(dspB_b),
                      .C(dspB_c),
                      .D(0),
                      .CEA2(en_i),
                      .CEC(dspA_valid),
                      .CEP(dspA_valid),
                      .INMODE(dspB_inmode),
                      .OPMODE(dspB_opmode),.CARRYINSEL(dspB_carryinsel),.CECTRL(config_wr_i),
                      .ALUMODE(dspB_alumode),.CEALUMODE(config_wr_i),
                      .P(dspB_out),
                      .PATTERNDETECT(dspB_match),
                       .RSTA(1'b0),.RSTB(1'b0),.RSTC(1'b0),.RSTD(1'b0),.RSTCTRL(1'b0),.RSTINMODE(1'b0),.RSTM(1'b0),.RSTP(zero_i),
                       .CLK(sys_clk_i));                      

    // debugging takes:
    // 1: lab_dat_i
    // 2: lab_wr_i
    // 3: lab_adr_i
    // 4,5,6: bram_in (concat)
    // 7: bram_wr (3 bit)
    // 8: en_i
    generate
        if (DEBUG == "TRUE") begin : DBG
            calram_debug_ila u_ila(.clk(sys_clk_i),
                                   .probe0(lab_dat_i),
                                   .probe1(lab_wr_i),
                                   .probe2(lab_adr_i),
                                   .probe3(bram_inA[0]),
                                   .probe4(bram_inA[1]),
                                   .probe5(bram_inA[2]),
                                   .probe6(bram_wea),
                                   .probe7(en_i));
       end
   endgenerate
    

    // That's it: that's all this costs us (well, I mean, it's 48 BRAMs over everything so...)
    assign ack_o = (state == READ_ACK || state == WRITE_ACK);        
    assign zc_full_o = zc_full && !ped_mode;
endmodule
                           
                           