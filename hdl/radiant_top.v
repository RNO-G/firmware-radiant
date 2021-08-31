`timescale 1ns / 1ps
`include "wishbone.vh"
`include "lab4.vh"
`include "radiant_debug.vh"
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
                    
                    inout [1:0] RAMP,
                    
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

                    // PPS
                    input PPS,
                    
                    // sync output, hopefully...
                    output SYNC_P,
                    output SYNC_N,
                    
                    // Trigger INPUT/OUTPUT FINALLY
                    input TRIGIN_P,
                    input TRIGIN_N,
                    
                    output TRIGOUT_P,
                    output TRIGOUT_N,
                    
                    // not actually differential in this usage
                    // we just drive them to ground (on both sides) to maybe
                    // improve signal integrity and reserve differential usage
                    // for later                                      
                    input CB_SCLK_P,
                    output CB_SCLK_N,
                    
                    input CB_MOSI_P,
                    output CB_MOSI_N,
                    
                    output CB_MISO_P,
                    output CB_MISO_N,
                    
                    input CB_CS_B_P,
                    output CB_CS_B_N,

                    output PROG_FULL,

                    output JTAGENB,
                    output MOSI,
                    input MISO,
                    output CS_B,
                    output HOLDB,
                    output WPB,
                    output F_LED
    );

    wire main_reset = 1'b0;

    // drive virtual grounds
    assign CB_SCLK_N = 1'b0;
    assign CB_MOSI_N = 1'b0;
    assign CB_MISO_N = 1'b0;
    assign CB_CS_B_N = 1'b0;

    parameter SIMULATION = "FALSE";

    parameter DUAL_BANK = "TRUE";

    parameter [31:0] IDENT = "RDNT";
    parameter [3:0] VER_MAJOR = 0;
    parameter [3:0] VER_MINOR = 3;
    parameter [7:0] VER_REV = 2;
    localparam [15:0] FIRMWARE_VERSION = { VER_MAJOR, VER_MINOR, VER_REV };
    // gets pulled in by Tcl script.
    // bits[4:0] = day
    // bits[8:5] = month
    // bits[15:9] = 2-digit year
    parameter [15:0] FIRMWARE_DATE = {16{1'b0}};
    localparam [31:0] DATEVERSION = { FIRMWARE_DATE, FIRMWARE_VERSION };

    // These are parameters since simulation can overwrite them
    // to avoid stupidity.
    parameter [23:0] WCLK_POLARITY =       24'b000101111111000100110011;
    parameter [1:0] MONTIMING_POLARITY = 2'b11;

    // TRIG goes negative: so to get a positive trigger, we put TRIG on the negative
    // side and THRESH on the positive side. Normally, TRIG > THRESH so it's
    // zero. If trig goes below thresh, then it's positive.
    // So POLARITY is 1 whenever TRIG is going into a P input.
    parameter [23:0] TRIG_POLARITY =       24'b011011001001001010010110;

    // this is the CPLD montiming polarity. We fix it here just to allow the paths to all be identical.
    // Probably unimportant, but whatever.
    parameter [23:0] CPLD_MT_POLARITY =    24'b010011100001010011100001;
        
        
    // polarity of the DOE inputs        
    parameter [23:0] DOE_POLARITY =        24'b001001111110000011111001;
    
    // polarity of the SRCLK outputs
    parameter [1:0] SRCLK_POLARITY = 2'b00;
        
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
    wire lab_sync;
    // 200 MHz
    wire wclk;
    // 400 MHz
    wire trigclk;
    IBUFDS u_sysclk_ibuf(.I(SYS_CLK_P),.IB(SYS_CLK_N),.O(sysclk_in));

    `WB_DEFINE( bmc , 32, 22, 4 );
    `WB_DEFINE( spid, 32, 22, 4 );
    `WB_DEFINE( pciec,32, 22, 4 );
      
    // not really 16 bits, actually only 15
    `WB_DEFINE( rad_id_ctrl, 32, 16, 4);
    // not really 16 bits, actually only 15
    `WB_DEFINE( spic, 32, 16, 4);
    `WB_DEFINE( l4_ctrl, 32, 16, 4);
    `WB_DEFINE( l4_ram, 32, 16, 4);
    `WB_DEFINE( trig, 32, 16, 4);
    `WB_DEFINE( scal, 32, 18, 4);
    `WB_DEFINE( calram, 32, 19, 4);

    `WB_KILL(pciec, 32, 22, 4);
        
    // The boardman_interface is "close enough" to a WISHBONE classic interface, we just set
    // reg_en = cyc = stb
    // wr = we
    // wstrb = sel
    // and set ack_i = (ack | err | rty)
    assign bmc_cyc_o = bmc_stb_o;
    // it's always a 32-bit interface, the interface's low bits just generate the byte enables.
    assign bmc_adr_o[1:0] = 2'b00;
    wire [1:0] burst_size;
    boardman_interface #(.SIMULATION(SIMULATION),.CLOCK_RATE(50000000),.BAUD_RATE(1000000)) u_bmif(.clk(CLK50),.rst(1'b0),.BM_RX(BM_RX),.BM_TX(BM_TX),
                                                                          .adr_o(bmc_adr_o[21:2]),
                                                                          .en_o(bmc_stb_o),
                                                                          .wr_o(bmc_we_o),
                                                                          .wstrb_o(bmc_sel_o),
                                                                          .dat_o(bmc_dat_o),
                                                                          .dat_i(bmc_dat_i),
                                                                          .ack_i(bmc_ack_i || bmc_err_i || bmc_rty_i),
                                                                          .burst_size_i(burst_size));
    
    wbc_intercon u_intercon( .clk_i(CLK50),.rst_i(1'b0),
                            `WBS_CONNECT( bmc ,     bmc ),
                            `WBS_CONNECT( spid,     spid),
                            `WBS_CONNECT( pciec,    pciec),
                            `WBM_CONNECT( rad_id_ctrl, rad_id_ctrl),
                            `WBM_CONNECT( spic, spic),
                            `WBM_CONNECT( l4_ctrl, l4_ctrl),
                            `WBM_CONNECT( l4_ram, l4_ram),
                            `WBM_CONNECT( trig , trig),
                            `WBM_CONNECT( calram, calram),
                            `WBM_CONNECT( scal , scal));

    wire [1:0] ss_incr;
    wire [1:0] sclk;
    wire [1:0] shout;
    wire [`LAB4_WR_WIDTH-1:0] reset_wr;
    wire [1:0] invert_montiming;
    wire spiclk;
    wire [1:0] rampdone_in;
    
    wire [23:0] lab4_channel_disable;
    wire pps_flag;
    wire sync_en;
    reg [24:0] counter = {25{1'b0}};
    rad_id_ctrl #(.DEVICE(IDENT),.VERSION(DATEVERSION),.MONTIMING_POLARITY(CPLD_MT_POLARITY)) u_id(.clk_i(CLK50),.rst_i(main_reset),
                     `WBS_CONNECT( rad_id_ctrl, wb ),
                     .sys_clk_in(sysclk_in),
                     .sys_clk_o(sysclk),
                     .sys_clk_div4_o(sysclk_div4),
                     .sys_clk_div4_flag_o(sysclk_div4_flag),
                     .sync_o(lab_sync),
                     .sync_reset_i(),
                     .wclk_o(wclk),
                     .spiclk_o(spiclk),
                     .trigclk_o(trigclk),
                     .reset_wr_o(reset_wr),
                     .invert_montiming_o(invert_montiming),
                     
                     .lab4_channel_disable_o(lab4_channel_disable),                     
                     
                     .sys_clk_div8_ps_o(sysclk_div8_ps),
                     .ps_clk_i(ps_clk),
                     .ps_en_i(ps_en),
                     .ps_incdec_i(ps_incdec),
                     .ps_done_o(ps_done),
                                       
                     .burst_size_o(burst_size),
                                          
                     .internal_led_i(counter[23]),
                     .ss_incr_i(ss_incr),
                     .sclk_i(sclk),
                     .shout_o(shout),
                     
                     .rampdone_i(rampdone_in),
                     
                     .pps_flag_o(pps_flag),
                     
                     .sync_en_o(sync_en),
                     
                     .PPS(PPS),
                     
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
    wire readout_counter_rst;
    
    wire [23:0] readout_fifo_empty;
    wire [9:0] readout_empty_size;
    wire [3:0] readout_prescale;
    wire readout_complete;
    wire trigger_in;
    
    // ALL OF THIS is sysclk domain
    wire event_begin;
    wire event_done;
    wire dma_req;
    wire dma_rdy;
    wire readout_running;
    
    lab4d_controller #(.NUM_LABS(24),.NUM_MONTIMING(2),.NUM_SCLK(2),.NUM_REGCLR(1),.NUM_RAMP(2),
                       .NUM_SHOUT(2),.NUM_WR(4),.WCLK_POLARITY(WCLK_POLARITY),
                       .DUAL_BANK(DUAL_BANK))    
                     u_controller( .clk_i(CLK50),
                                   .rst_i(1'b0),
                                   `WBS_CONNECT(l4_ctrl, wb),
                                    // MAKE THESE REAL
                                   .sys_clk_i(sysclk),
                                   .sys_clk_div4_flag_i(sysclk_div4_flag),
                                   .sync_i(lab_sync),
                                   .wclk_i(wclk),
                                                                      
                                   .reset_wr_i(reset_wr),
                                   .invert_montiming_i(invert_montiming),
                                   
                                   .trig_i(trigger_in),
                                   .event_o(event_begin),
                                   .event_done_o(event_done),
                                   
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
                                   .readout_counter_rst_o(readout_counter_rst),
                                   .readout_fifo_empty_i(readout_fifo_empty),
                                   .readout_empty_size_o(readout_empty_size),
                                   .readout_running_o(readout_running),
                                   .prescale_o(readout_prescale),
                                   .complete_i(readout_complete),
                                   
                                   .montiming_i(montiming_reg),
                                   .ramp_in_o(rampdone_in),
                                   // END MAKE THESE REAL
                                   
                                   .lab4_channel_disable_i(lab4_channel_disable),
                                   
                                   .SIN(SIN),
                                   .SCLK(sclk),
                                   .PCLK(PCLK),
                                   .REGCLR(REGCLR),
                                   .RAMP(RAMP),
                                   .WCLK_P(WCLK_P),
                                   .WCLK_N(WCLK_N),
                                   .SHOUT(shout),
                                   .WR(WR));

//    // sysclk domain
//    wire [24*12-1:0] lab_dat;
//    wire [23:0] lab_wr;      
//    wire [23:0] lab_stop;                             
                                   
//    par_lab4d_ram #(.NUM_SS_INCR(2),.NUM_SRCLK(2),.SRCLK_POLARITY(SRCLK_POLARITY),.NUM_LAB4(24),.DOE_POLARITY(DOE_POLARITY),.SRCLK_DIFFERENTIAL("FALSE"))
//            u_l4ram(.clk_i(CLK50),
//                    .rst_i(1'b0),
//                    `WBS_CONNECT(l4_ram, wb),
//                    // In the SURF5 the LAB4D RAM had two ports, and swapped between them
//                    // We're not going to do that: we're treating the WB bus as fully shared.
//                    `WBS_CONNECT(l4_ram_dummy, wbdma),
//                    .sys_clk_i(sysclk),
//                    .wclk_i(wclk),
//                    .readout_test_pattern_i(readout_test_pattern),
//                    .readout_i(readout),
//                    .readout_header_i(readout_header),
//                    .readout_rst_i(readout_rst),
//                    .readout_fifo_rst_i(readout_fifo_rst),
//                    .readout_empty_size_i(readout_empty_size),
//                    .readout_fifo_empty_o(readout_fifo_empty),
//                    .prescale_i(readout_prescale),
//                    .complete_o(readout_complete),
                    
//                    .lab_dat_o(lab_dat),
//                    .lab_wr_o(lab_wr),
//                    .lab_stop_o(lab_stop),
                    
//                    .DOE_LVDS_P(DOE_P),
//                    .DOE_LVDS_N(DOE_N),
//                    .SS_INCR(ss_incr),
//                    .SRCLK_P(SRCLK),
//                    // not differential
//                    .SRCLK_N());
    
    // pass to CalRam
    wire [24*12-1:0] lab_dat;
    wire [24*4-1:0]  lab_header;
    wire [24*12-1:0] lab_sample;
    wire [24-1:0]    lab_wr;
    // readout ONLY. RAM is stored after CalRam
    par_lab4d_readout #(.NUM_SS_INCR(2),.NUM_SRCLK(2),.SRCLK_POLARITY(SRCLK_POLARITY),.NUM_LAB4(24),.DOE_POLARITY(DOE_POLARITY),.SRCLK_DIFFERENTIAL("FALSE"))
            u_l4ram(.clk_i(CLK50),
                    .sys_clk_i(sysclk),
                    .wclk_i(wclk),
                    .readout_i(readout),
                    .readout_counter_rst_i(readout_counter_rst),
                    .readout_rst_i(readout_rst),

                    .readout_header_i(readout_header),
                    // this is fixed, defines the bits in the header that are address
                    .readout_counter_mask_i(4'hC),
                    
                    .prescale_i(readout_prescale),
                    .complete_o(readout_complete),
                    
                    .lab_dat_o(lab_dat),
                    .lab_header_o(lab_header),
                    .lab_sample_o(lab_sample),
                    .lab_wr_o(lab_wr),
                    
                    .DOE_LVDS_P(DOE_P),
                    .DOE_LVDS_N(DOE_N),
                    .SS_INCR(ss_incr),
                    .SRCLK_P(SRCLK),
                    // not differential
                    .SRCLK_N());

    // CalRam OUTPUT
    wire [24*16-1:0] labcal_dat;
    wire [24-1:0]    labcal_wr;
    
    // hook up the calram
    wb_calram_v2 u_calram(.clk_i(CLK50),
                       .rst_i(main_reset),
                       `WBS_CONNECT(calram, wb),
                       .sys_clk_i(sysclk),
                       .lab_dat_i(lab_dat),
                       .lab_wr_i(lab_wr),
                       .lab_header_i(lab_header),
                       .lab_sample_i(lab_sample),
                       .lab_dat_o(labcal_dat),
                       .lab_wr_o(labcal_wr));
    
    wire event_fifo_reset;
    wire event_fifo_empty;
    wire [11:0] readout_full_thresh;
    wire [23:0] readout_full;
    par_lab4d_fifo #(.NUM_LAB4(24))
        u_fifo( .clk_i(CLK50),
                .rst_i(1'b0),
                `WBS_CONNECT(l4_ram, wb),
                .sys_clk_i(sysclk),
                .lab_dat_i(labcal_dat),
                .lab_wr_i(labcal_wr),
                .fifo_empty_o(event_fifo_empty),
                .fifo_rst_i(event_fifo_reset),
                .full_thresh_i(readout_full_thresh),
                .full_o(readout_full));

    // WHO THE HECK KNOWS RIGHT NOW
    wire [15:0] trig_info;
    wire [31:0] event_info = { {16{1'b0}}, trig_info };

    // this is a board-to-board sync
    wire sync;                                                           
    wire pulse; 
    localparam NUM_SCALERS = 24;                      
    wire [NUM_SCALERS-1:0] scaler_inputs;
    wire [23:0] trig_scalers; 
    assign scaler_inputs[0 +: 24] = trig_scalers;
    radiant_trig_top #(.TRIG_POLARITY(TRIG_POLARITY)) u_trig(.clk_i(CLK50),.rst_i(1'b0),
                                                             `WBS_CONNECT(trig, wb),
                                                             .pwm_clk_i(wclk),
                                                             .sys_clk_i(sysclk),
                                                             .trig_clk_i(trigclk),
                                                             
                                                             .pps_i(pps_flag),
                                                             .sync_o(sync),
                                                             
                                                             .event_i(event_begin),
                                                             .event_info_i(event_info),
                                                             
                                                             .event_ready_o(dma_req),
                                                             .event_readout_ready_i(dma_rdy),
                                                             
                                                             .event_fifo_reset_o(event_fifo_reset),
                                                             .event_fifo_empty_i(event_fifo_empty),
                                                             
                                                             .pulse_o(pulse),
                                                             .scal_o(trig_scalers),
                                                             
                                                             .full_trig_o(trigger_in),
                                                             .trig_info_o(trig_info),
                                                             // god what a naming mess
                                                             .readout_done_i(event_done),
                                                             .readout_full_i(readout_full),
                                                             .readout_full_thresh_o(readout_full_thresh),
                                                             .readout_running_i(readout_running),
                                                             
                                                             .TRIGIN_P(TRIGIN_P),
                                                             .TRIGIN_N(TRIGIN_N),
                                                             .TRIGOUT_P(TRIGOUT_P),
                                                             .TRIGOUT_N(TRIGOUT_N),
                                                             
                                                             .TRIG(TRIG),
                                                             .THRESH(THRESH),
                                                             .THRESH_PWM(THRESH_PWM));                                  

    // Scalers. They run totally in the 50 MHz domain.
    wire pps_flag_clk;
    flag_sync u_clkA(.in_clkA(pps_flag),.clkA(sysclk),.clkB(CLK50),.out_clkB(pps_flag_clk));
    radiant_scalers #(.NUM_SCALERS(NUM_SCALERS)) u_scalers(.clk_i(CLK50),.rst_i(1'b0),
                                                  `WBS_CONNECT(scal, wb),                                                  
                                                  .pps_i(pps_flag_clk),
                                                  .scal_i( scaler_inputs ));                                                  
    
    OBUFTDS u_sync(.I(sync),.O(SYNC_P),.OB(SYNC_N),.T(!sync_en));
    
    always @(posedge sysclk) begin
        counter <= counter[23:0] + 1;
    end
//    reg [1:0]  pulse_catch = {2{1'b0}};
//    reg        pulse_enable = 0;
//    // just... do it every 16 for now. We need to make this controllable
//    // via a register and use a freakin DSP
//        // counter[0] would be every 2
//        // counter[1] would be every 4
//        // counter[2] would be every 8
//        // counter[3] would be every 16
//        // counter[4] would be every 32
//        pulse_catch <= {pulse_catch[0],counter[5]};
//        pulse_enable <= pulse_catch[0] && !pulse_catch[1];

        
//    wire pulse_out;
//    ODDR u_oddrpulse(.D1(1'b1),.D2(1'b0),.CE(pulse_enable),.C(sysclk),.S(1'b0),.R(1'b0),.Q(pulse_out));
    OBUFDS u_obufpulse(.I(pulse),.O(PULSE_P),.OB(PULSE_N));
    
    // SPI DMA module!
    spidma u_spidma( .wb_clk_i(CLK50),
                     .wb_rst_i(1'b0),
                     `WBS_CONNECT( spic , wb ),
                     `WBM_CONNECT( spid , wbdma ),
                     // LATER: this will come from the event module in the trigger path
                     .dma_req_i(dma_req),
                     .dma_rdy_o(dma_rdy),
                     .fast_clk_i(spiclk),
                     .PROG_FULL(PROG_FULL),
                     .SCLK(CB_SCLK_P), 
                     .MISO(CB_MISO_P), 
                     .MOSI(CB_MOSI_P), 
                     .CS_B(CB_CS_B_P));
                     

//    // OK, dumbass ILA to test SPI stuff.
//    // We'll try running at 200 MHz. If that works I should be able to handle that clock rate.
//    // Maybe literally just run an IOFIFO as a single-bit FIFO and just keep it full. Will see.
//    // The damn I/O technically can't toggle this fast, but we don't need it to.
//    (* IOBUF = "TRUE" *)
//    reg dbg_sclk = 0;
    
//    reg dbg_sclk_reg = 0;
    
//    (* IOBUF = "TRUE" *)
//    reg dbg_mosi = 0;    
//    (* IOBUF = "TRUE" *)
//    reg dbg_miso = 0;
    
//    reg ila_miso = 0;    
    
//    (* IOBUF = "TRUE" *)
//    reg dbg_cs = 0;
    
//    reg dbg_cs_reg = 0;
//    // OK: so let's try to do this via an OSERDES now.

//    // LET'S DO THE SPI SHUFFLE!
////    reg do_load = 0;
////    wire [31:0] value_in;
////    reg [31:0] value_in_rereg = {32{1'b0}};
////    reg [31:0] shift_reg = {32{1'b0}};
////    vio_spi_test u_vio(.clk(wclk),.probe_out0(value_in));
    
//    // *This* is nominally fast enough.
//    //
//    // The key is that we need to make sure MISO's clock enable
//    // is (dbg_sclk && !dbg_sclk_reg) NO MATTER WHAT.
//    // We might as well add do_load to that list as well: but what
//    // we'll do is offset do_load by 1, so we'll actually do:
//    // 
////    always @(posedge wclk) begin
////        value_in_rereg <= value_in;
        
////        do_load <= !dbg_cs && dbg_cs_reg;

////        // 1st clock with dbg_sclk high is AT MOST 10 ns after SCLK rises
////        // 2nd clock (to get back out) is AT MOST 15 ns.
////        // MAAYBE this will work?
////        // note that if this works we'll *actually* hook it up to a FIFO
////        // (maybe an OUT_FIFO) and then adapt the width so we can feed it
////        // slower.
////        if (do_load) shift_reg <= value_in_rereg;
////        else if (dbg_sclk && !dbg_sclk_reg) shift_reg <= {shift_reg[30:0], 1'b0};        
    
////        dbg_sclk <= CB_SCLK_P;
////        dbg_sclk_reg <= dbg_sclk;
////        dbg_mosi <= CB_MOSI_P;
////        dbg_miso <= (dbg_sclk && !dbg_sclk_reg) ? shift_reg[30] : shift_reg[31];
////        dbg_cs <= CB_CS_B_P;
////        dbg_cs_reg <= dbg_cs;
        
////        ila_miso <= (dbg_sclk && !dbg_sclk_reg) ? shift_reg[30] : shift_reg[31];
////    end
////    assign CB_MISO_P = dbg_miso;
////    cb_spi_ila u_ila(.clk(wclk),.probe0(dbg_sclk),.probe1(dbg_mosi),.probe2(ila_miso),.probe3(dbg_cs));

//    wire [7:0] s_axis_tdata;
//    wire       vio_load;
//    reg        vio_load_reg = 0;
//    reg       s_axis_tvalid = 0;
//    wire      s_axis_tready;
//    always @(posedge wclk) begin
//        vio_load_reg <= vio_load;
//        if (vio_load && !vio_load_reg) s_axis_tvalid <= 1;
//        else if (s_axis_tready) s_axis_tvalid <= 0;
//    end
//    fast_spi_vio_debug u_vio(.clk(wclk),.probe_in0(s_axis_tvalid),.probe_out0(s_axis_tdata),.probe_out1(vio_load));    
//    fast_spi_fifo u_fifo(.aclk(wclk),.aresetn(1'b1),.s_axis_tdata(s_axis_tdata),.s_axis_tvalid(s_axis_tvalid),.s_axis_tready(s_axis_tready),
//                         .SCLK(CB_SCLK_P),
//                         .MISO(CB_MISO_P),
//                         .MOSI(CB_MOSI_P),
//                         .CS_B(CB_CS_B_P));
    assign CLK50_EN = 1'b1;
    assign WPB = 1'b1;
    assign HOLDB = 1'b1;
endmodule
