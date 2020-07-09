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
                    output F_LED
    );

    wire sysclk;
    IBUFDS u_sysclk_ibuf(.I(SYS_CLK_P),.IB(SYS_CLK_N),.O(sysclk));

    `WB_DEFINE( boardman , 32, 20, 4 );
    
    wire [19:0] reg_addr;
    wire        reg_en;
    wire        reg_wr;
    wire [3:0]  reg_wstrb;
    wire [31:0] reg_data_out;
    reg [31:0] test_reg = {32{1'b0}};
    wire [31:0] reg_data_in = (reg_addr[0] ? test_reg : "RDNT");
    
    always @(posedge CLK50) begin
        if (reg_en && reg_wr) begin
            if (reg_wstrb[0]) test_reg[7:0] <= reg_data_out[7:0];
            if (reg_wstrb[1]) test_reg[15:8] <= reg_data_out[15:8];
            if (reg_wstrb[2]) test_reg[23:16] <= reg_data_out[23:16];
            if (reg_wstrb[3]) test_reg[31:24] <= reg_data_out[31:24];
        end
    end
    
    // The boardman_interface is "close enough" to a WISHBONE classic interface, we just set
    // reg_en = cyc = stb
    // wr = we
    // wstrb = sel
    // and set ack_i = (ack | err | rty)
    assign boardman_cyc_o = boardman_stb_o;
    boardman_interface #(.CLOCK_RATE(50000000),.BAUD_RATE(115200)) u_bmif(.clk(CLK50),.rst(1'b0),.BM_RX(BM_RX),.BM_TX(BM_TX),
                                                                          .adr_o(boardman_adr_o),
                                                                          .en_o(boardman_stb_o),
                                                                          .wr_o(boardman_we_o),
                                                                          .wstrb_o(boardman_sel_o),
                                                                          .dat_o(boardman_dat_o),
                                                                          .dat_i(boardman_dat_i),
                                                                          .ack_i(boardman_ack_i || boardman_err_i || boardman_rty_i));
    
    
    reg [24:0] counter = {25{1'b0}};
    
    always @(posedge sysclk) counter <= counter[23:0] + 1;
    
    vio_0 u_vio(.clk(CLK50),.probe_in0(counter[23]));
    
    wire pulse_out;
    ODDR u_oddrpulse(.D1(1'b1),.D2(1'b0),.CE(counter[24]),.C(sysclk),.S(1'b0),.R(1'b0),.Q(pulse_out));
    OBUFDS u_obufpulse(.I(pulse_out),.O(PULSE_P),.OB(PULSE_N));
        
    assign CLK50_EN = 1'b1;
    assign F_LED = counter[23];
endmodule
