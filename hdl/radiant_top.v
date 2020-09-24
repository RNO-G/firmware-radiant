`timescale 1ns / 1ps
`include "wishbone.vh"
`include "lab4.vh"
module radiant_top( input SYS_CLK_P,
                    input SYS_CLK_N,
                    output PULSE_P,
                    output PULSE_N,
                    input CLK50,
                    output CLK50_EN,
                    input BM_RX,
                    output BM_TX,
                    output [1:0] CDAT_TDI,
                    output [1:0] CCLK_TMS,
                    output [1:0] SCLK_TCK,
                    inout [1:0] SSINCR_TDO,
                    output [1:0] RAMP,
                    
                    output [23:0] PCLK,
                    output [23:0] WCLK_P,
                    output [23:0] WCLK_N,
                    output [23:0] SIN,
                    
                    output [19:0] WR,

                    output [1:0] SRCLK,
                    
                    input [23:0] DOE_P,
                    input [23:0] DOE_N,

                    input [1:0] MONTIMING_P,
                    input [1:0] MONTIMING_N,
                    inout SYNCMON,
                    
                    output REGCLR,
                    
                    output [1:0] SST_SEL,
                    
                    input [23:0] TRIG,
                    input [23:0] THRESH,
                    output [23:0] THRESH_PWM,

                    output JTAGENB,
                    output MOSI,
                    input MISO,
                    output CS_B,
                    output HOLDB,
                    output WPB,
                    output F_LED
    );

    parameter [31:0] IDENT = "RDNT";
    parameter [3:0] VER_MAJOR = 0;
    parameter [3:0] VER_MINOR = 0;
    parameter [7:0] VER_REV = 2;
    localparam [15:0] FIRMWARE_VERSION = { VER_MAJOR, VER_MINOR, VER_REV };
    parameter [15:0] FIRMWARE_DATE = {16{1'b0}};
    localparam [31:0] DATEVERSION = { FIRMWARE_DATE, FIRMWARE_VERSION };

    localparam [23:0] WCLK_POLARITY =       24'b000101111111000100110011;
    localparam [1:0] MONTIMING_POLARITY = 2'b11;
    // TRIG goes negative: so to get a positive trigger, we put TRIG on the negative
    // side and THRESH on the positive side. Normally, TRIG > THRESH so it's
    // zero. If trig goes below thresh, then it's positive.
    // So POLARITY is 1 whenever TRIG is going into a P input.
    localparam [23:0] TRIG_POLARITY =       24'b011011001001001010010110;

    // this is the CPLD montiming polarity. We fix it here just to allow the paths to all be identical.
    // Probably unimportant, but whatever.
    localparam [23:0] CPLD_MT_POLARITY =    24'b010011100001010011100001;
        
        
    // polarity of the DOE inputs        
    localparam [23:0] DOE_POLARITY =        24'b001011111110000001111001;
    
    // polarity of the SRCLK outputs
    localparam [1:0] SRCLK_POLARITY = 2'b00;
        
    // sysclk coming in is 25 MHz.
    wire sysclk_in;
    // sysclk is 100 MHz
    wire sysclk;
    // sysclk_div4 is 25 MHz
    wire sysclk_div4; 
    // this is a flag synchronizing sysclk to sysclk_div4
    wire sysclk_div4_flag;
    // phase shift clock (12.5 MHz)
    wire sysclk_div8_ps;
    wire ps_clk;
    wire ps_en;
    wire ps_incdec;
    wire ps_done;
    // global 12.5 MHz sync
    wire sync;
    // 200 MHz
    wire wclk;
    IBUFDS u_sysclk_ibuf(.I(SYS_CLK_P),.IB(SYS_CLK_N),.O(sysclk_in));

    `WB_DEFINE( bmc , 32, 22, 4 );
    `WB_DEFINE( spic, 32, 22, 4 );
    `WB_DEFINE( pciec,32, 22, 4 );
    
    `WB_DEFINE( rad_id_ctrl, 32, 16, 4);
    `WB_DEFINE( l4_ctrl, 32, 16, 4);
    `WB_DEFINE( l4_ram, 32, 16, 4);
    `WB_DEFINE( trig, 32, 16, 4);
    `WB_DEFINE( scal, 32, 16, 4);
    
    `WBM_KILL( scal, 32);
    
    `WB_KILL(spic, 32, 22, 4);
    `WB_KILL(pciec, 32, 22, 4);
        
    // The boardman_interface is "close enough" to a WISHBONE classic interface, we just set
    // reg_en = cyc = stb
    // wr = we
    // wstrb = sel
    // and set ack_i = (ack | err | rty)
    assign bmc_cyc_o = bmc_stb_o;
    // it's always a 32-bit interface, the interface's low bits just generate the byte enables.
    assign bmc_adr_o[1:0] = 2'b00;
    boardman_interface #(.CLOCK_RATE(50000000),.BAUD_RATE(115200)) u_bmif(.clk(CLK50),.rst(1'b0),.BM_RX(BM_RX),.BM_TX(BM_TX),
                                                                          .adr_o(bmc_adr_o[21:2]),
                                                                          .en_o(bmc_stb_o),
                                                                          .wr_o(bmc_we_o),
                                                                          .wstrb_o(bmc_sel_o),
                                                                          .dat_o(bmc_dat_o),
                                                                          .dat_i(bmc_dat_i),
                                                                          .ack_i(bmc_ack_i || bmc_err_i || bmc_rty_i));
    
    wbc_intercon u_intercon( .clk_i(CLK50),.rst_i(1'b0),
                            `WBS_CONNECT( bmc ,     bmc ),
                            `WBS_CONNECT( spic,     spic),
                            `WBS_CONNECT( pciec,    pciec),
                            `WBM_CONNECT( rad_id_ctrl, rad_id_ctrl),
                            `WBM_CONNECT( l4_ctrl, l4_ctrl),
                            `WBM_CONNECT( l4_ram, l4_ram),
                            `WBM_CONNECT( trig , trig),
                            `WBM_CONNECT( scal , scal));

    wire [1:0] ss_incr;
    wire [1:0] sclk;
    wire [1:0] shout;
    wire [`LAB4_WR_WIDTH-1:0] reset_wr;
    wire [1:0] invert_montiming;
    rad_id_ctrl #(.DEVICE(IDENT),.VERSION(DATEVERSION),.MONTIMING_POLARITY(CPLD_MT_POLARITY)) u_id(.clk_i(CLK50),.rst_i(1'b0),
                     `WBS_CONNECT( rad_id_ctrl, wb ),
                     .sys_clk_in(sysclk_in),
                     .sys_clk_o(sysclk),
                     .sys_clk_div4_o(sysclk_div4),
                     .sys_clk_div4_flag_o(sysclk_div4_flag),
                     .sync_o(sync),
                     .sync_reset_i(),
                     .wclk_o(wclk),
                     
                     .reset_wr_o(reset_wr),
                     .invert_montiming_o(invert_montiming),
                     
                     .sys_clk_div8_ps_o(sysclk_div8_ps),
                     .ps_clk_i(ps_clk),
                     .ps_en_i(ps_en),
                     .ps_incdec_i(ps_incdec),
                     .ps_done_o(ps_done),
                                          
                     .internal_led_i(counter[23]),
                     .ss_incr_i(ss_incr),
                     .sclk_i(sclk),
                     .shout_o(shout),
                     .JTAGENB(JTAGENB),
                     .SST_SEL(SST_SEL),
                     .SSINCR_TDO(SSINCR_TDO),
                     .CDAT_TDI(CDAT_TDI),
                     .SCLK_TCK(SCLK_TCK),
                     .CCLK_TMS(CCLK_TMS),
                     .MOSI(MOSI),
                     .MISO(MISO),
                     .CS_B(CS_B),
                     .F_LED(F_LED));
    
    wire [1:0] montiming_bar;
    wire [1:0] montiming;
    (* IOB = "TRUE" *)
    reg [1:0] montiming_reg = {2{1'b0}};
    generate
        genvar m;
        for (m=0;m<2;m=m+1) begin : MT
            if (MONTIMING_POLARITY[m] == 1'b0) begin : POS
                IBUFDS_DIFF_OUT u_mt_ibuf(.I(MONTIMING_P[m]),.IB(MONTIMING_N[m]),.O(montiming[m]),.OB(montiming_bar[m]));
            end else begin : NEG
                IBUFDS_DIFF_OUT u_mt_ibuf(.I(MONTIMING_N[m]),.IB(MONTIMING_P[m]),.O(montiming_bar[m]),.OB(montiming[m]));
            end
        end
    endgenerate
    always @(posedge sysclk) montiming_reg <= montiming;
    
    wire readout;
    wire [3:0] readout_header;
    wire readout_test_pattern;
    wire readout_fifo_rst;
    wire readout_rst;
    wire [23:0] readout_fifo_empty = {24{1'b0}};
    wire [9:0] readout_empty_size;
    wire [3:0] readout_prescale;
    wire readout_complete;
    wire trigger_in;
    lab4d_controller #(.NUM_LABS(24),.NUM_MONTIMING(2),.NUM_SCLK(2),.NUM_REGCLR(1),.NUM_RAMP(2),
                       .NUM_SHOUT(2),.NUM_WR(4),.WCLK_POLARITY(WCLK_POLARITY))    
                     u_controller( .clk_i(CLK50),
                                   .rst_i(1'b0),
                                   `WBS_CONNECT(l4_ctrl, wb),
                                    // MAKE THESE REAL
                                   .sys_clk_i(sysclk),
                                   .sys_clk_div4_flag_i(sysclk_div4_flag),
                                   .sync_i(sync),
                                   .wclk_i(wclk),
                                   
                                   .reset_wr_i(reset_wr),
                                   .invert_montiming_i(invert_montiming),
                                   
                                   .trig_i(trigger_in),
                                   
                                   .clk_ps_i(sysclk_div8_ps),
                                   .ps_clk_o(ps_clk),
                                   .ps_en_o(ps_en),
                                   .ps_incdec_o(ps_incdec),
                                   .ps_done_i(ps_done),
                                   .MONTIMING_B(montiming_bar),
                                    // unused, but needs to be connected to an (unused) IO
                                   .sync_mon_io(SYNCMON),
                                   
                                   .readout_o(readout),
                                   .readout_header_o(readout_header),
                                   .readout_test_pattern_o(readout_test_pattern),
                                   .readout_fifo_rst_o(readout_fifo_rst),
                                   .readout_rst_o(readout_rst),
                                   .readout_fifo_empty_i(readout_fifo_empty),
                                   .readout_empty_size_o(readout_empty_size),
                                   .prescale_o(readout_prescale),
                                   .complete_i(readout_complete),
                                   
                                   .montiming_i(montiming_reg),
                                   // END MAKE THESE REAL
                                   .SIN(SIN),
                                   .SCLK(sclk),
                                   .PCLK(PCLK),
                                   .REGCLR(REGCLR),
                                   .RAMP(RAMP),
                                   .WCLK_P(WCLK_P),
                                   .WCLK_N(WCLK_N),
                                   .SHOUT(shout),
                                   .WR(WR));
    par_lab4d_ram #(.NUM_SS_INCR(2),.NUM_SRCLK(2),.SRCLK_POLARITY(SRCLK_POLARITY),.NUM_LAB4(24),.DOE_POLARITY(DOE_POLARITY),.SRCLK_DIFFERENTIAL("FALSE"))
            u_l4ram(.clk_i(CLK50),
                    .rst_i(1'b0),
                    `WBS_CONNECT(l4_ram, wb),
                    `WBS_CONNECT(spic, wbdma),
                    .sys_clk_i(sysclk),
                    .wclk_i(wclk),
                    .readout_test_pattern_i(readout_test_pattern),
                    .readout_i(readout),
                    .readout_header_i(readout_header),
                    .readout_rst_i(readout_rst),
                    .readout_fifo_rst_i(readout_fifo_rst),
                    .readout_empty_size_i(readout_empty_size),
                    .readout_fifo_empty_o(readout_fifo_empty),
                    .prescale_i(readout_prescale),
                    .complete_o(readout_complete),
                    .DOE_LVDS_P(DOE_P),
                    .DOE_LVDS_N(DOE_N),
                    .SS_INCR(ss_incr),
                    .SRCLK_P(SRCLK),
                    // not differential
                    .SRCLK_N());
                                                                                  
    radiant_trig_top #(.TRIG_POLARITY(TRIG_POLARITY)) u_trig(.clk_i(CLK50),.rst_i(1'b0),
                                                             `WBS_CONNECT(trig, wb),
                                                             .pwm_clk_i(wclk),
                                                             .TRIG(TRIG),
                                                             .THRESH(THRESH),
                                                             .THRESH_PWM(THRESH_PWM));                                  
    
    reg [24:0] counter = {25{1'b0}};
    
    always @(posedge sysclk) counter <= counter[23:0] + 1;
        
    wire pulse_out;
    ODDR u_oddrpulse(.D1(1'b1),.D2(1'b0),.CE(counter[24]),.C(sysclk),.S(1'b0),.R(1'b0),.Q(pulse_out));
    OBUFDS u_obufpulse(.I(pulse_out),.O(PULSE_P),.OB(PULSE_N));
        
    assign CLK50_EN = 1'b1;
    assign WPB = 1'b1;
    assign HOLDB = 1'b1;
endmodule
