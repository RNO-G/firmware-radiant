`timescale 1ns / 1ps
`include "wishbone.vh"
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
    parameter [7:0] VER_REV = 1;
    localparam [15:0] FIRMWARE_VERSION = { VER_MAJOR, VER_MINOR, VER_REV };
    parameter [15:0] FIRMWARE_DATE = {16{1'b0}};
    localparam [31:0] DATEVERSION = { FIRMWARE_DATE, FIRMWARE_VERSION };

    wire sysclk;
    IBUFDS u_sysclk_ibuf(.I(SYS_CLK_P),.IB(SYS_CLK_N),.O(sysclk));

    `WB_DEFINE( bmc , 32, 22, 4 );
    `WB_DEFINE( spic, 32, 22, 4 );
    `WB_DEFINE( pciec,32, 22, 4 );
    
    `WB_DEFINE( rad_id_ctrl, 32, 16, 4);
    `WB_DEFINE( l4_ctrl, 32, 16, 4);
    `WB_DEFINE( l4_ram, 32, 16, 4);
    `WB_DEFINE( trig, 32, 16, 4);
    `WB_DEFINE( scal, 32, 16, 4);
    
    `WBM_KILL( l4_ctrl, 32);
    `WBM_KILL( l4_ram, 32);
    `WBM_KILL( trig, 32);
    `WBM_KILL( scal, 32);
    
    `WB_KILL(spic, 32, 20, 4);
    `WB_KILL(pciec, 32, 20, 4);
        
    // The boardman_interface is "close enough" to a WISHBONE classic interface, we just set
    // reg_en = cyc = stb
    // wr = we
    // wstrb = sel
    // and set ack_i = (ack | err | rty)
    assign bmc_cyc_o = bmc_stb_o;
    // it's always a 32-bit interface
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

    rad_id_ctrl #(.DEVICE(IDENT),.VERSION(DATEVERSION)) u_id(.clk_i(CLK50),.rst_i(1'b0),
                     `WBS_CONNECT( rad_id_ctrl, wb ),
                     .internal_led_i(counter[23]),
                     .ss_incr_i(2'b00),
                     .sclk_i(2'b00),
                     .JTAGENB(JTAGENB),
                     .SSINCR_TDO(SSINCR_TDO),
                     .CDAT_TDI(CDAT_TDI),
                     .SCLK_TCK(SCLK_TCK),
                     .CCLK_TMS(CCLK_TMS),
                     .MOSI(MOSI),
                     .MISO(MISO),
                     .CS_B(CS_B),
                     .F_LED(F_LED));
                     
    
    reg [24:0] counter = {25{1'b0}};
    
    always @(posedge sysclk) counter <= counter[23:0] + 1;
    
    vio_0 u_vio(.clk(CLK50),.probe_in0(counter[23]));
    
    wire pulse_out;
    ODDR u_oddrpulse(.D1(1'b1),.D2(1'b0),.CE(counter[24]),.C(sysclk),.S(1'b0),.R(1'b0),.Q(pulse_out));
    OBUFDS u_obufpulse(.I(pulse_out),.O(PULSE_P),.OB(PULSE_N));
        
    assign CLK50_EN = 1'b1;
    assign WPB = 1'b1;
    assign HOLDB = 1'b1;
endmodule
