`timescale 1ns/1ps
`include "wishbone.vh"
module pwm_wrap(
        input clk_i,
        input rst_i,
		`WBS_NAMED_PORT(wb, 32, 32, 4),     
		input pwm_clk_i,       
        output [23:0] THRESH_PWM
    );
    
    assign wb_rty_o = 1'b0;

    // adapt to AXI
    // we're just going to ignore wb_stall, hopefully it's not a problem
    // the guy who did this adapter sadly had no clue about naming conventions
    
    // screw prots
    wire [8:0] awaddr;
    wire        awvalid;
    wire        awready;
    wire [31:0] wdata;
    wire        wvalid;
    wire        wready;
    wire [3:0]  wstrb;
    wire [1:0]  bresp;
    wire        bvalid;
    wire        bready;
    wire [8:0] araddr;
    wire        arvalid;
    wire        arready;
    wire [31:0] rdata;
    wire        rready;
    wire        rvalid;
    wire [1:0]  rresp;


    wbm2axilite #(.C_AXI_ADDR_WIDTH(9))
        u_axi_adapt( .i_clk(clk_i),
                     .i_reset(rst_i),
                     .i_wb_cyc(wb_cyc_i),
                     .i_wb_stb(wb_stb_i),
                     .i_wb_we(wb_we_i),
                     // I.... have no idea WTF this is doing...
                     .i_wb_addr(wb_adr_i[2 +: 7]),
                     .i_wb_data(wb_dat_i),
                     .i_wb_sel(wb_sel_i),
                     .o_wb_stall(),
                     .o_wb_ack(wb_ack_o),
                     .o_wb_data(wb_dat_o),
                     .o_wb_err(wb_err_o),
                     
                     .o_axi_awvalid(awvalid),
                     .i_axi_awready(awready),
                     .o_axi_awaddr(awaddr),
                     
                     .o_axi_wvalid(wvalid),
                     .i_axi_wready(wready),
                     .o_axi_wdata(wdata),
                     .o_axi_wstrb(wstrb),
                     
                     .i_axi_bvalid(bvalid),
                     .o_axi_bready(bready),
                     .i_axi_bresp(bresp),
                     
                     .o_axi_arvalid(arvalid),
                     .i_axi_arready(arready),
                     .o_axi_araddr(araddr),
                     
                     .i_axi_rvalid(rvalid),
                     .o_axi_rready(rready),
                     .i_axi_rdata(rdata),
                     .i_axi_rresp(rresp));
    
/*    
    pwm_wbtoaxi u_axi_adapt(.s_wb_dat_w(wb_dat_i),
                            .s_wb_dat_r(wb_dat_o),
                            .s_wb_stb(wb_stb_i),
                            .s_wb_cyc(wb_cyc_i),
                            .s_wb_adr(wb_adr_i),
                            .s_wb_we(wb_we_i),
                            .s_wb_sel(wb_sel_i),
                            .s_wb_ack(wb_ack_o),
                            .s_wb_err(wb_err_o),
                            .s_wb_rty(wb_rty_o),
                            .s_wb_aclk(clk_i),
                            .s_wb_areset(rst_i),
                            .s_wb_lock(1'b0),
                            .s_wb_stall(),
                            .m_axi4_lite_aclk(clk_i),
                            .m_axi4_lite_aresetn(!rst_i),
                            .m_axi4_lite_awaddr(awaddr),
                            .m_axi4_lite_awprot(),
                            .m_axi4_lite_awvalid(awvalid),
                            .m_axi4_lite_awready(awready),
                            .m_axi4_lite_wdata(wdata),
                            .m_axi4_lite_wstrb(wstrb),
                            .m_axi4_lite_wvalid(wvalid),
                            .m_axi4_lite_wready(wready),
                            .m_axi4_lite_bresp(bresp),
                            .m_axi4_lite_bvalid(bvalid),
                            .m_axi4_lite_bready(bready),
                            .m_axi4_lite_araddr(araddr),
                            .m_axi4_lite_arready(arready),
                            .m_axi4_lite_arprot(),
                            .m_axi4_lite_arvalid(arvalid),
                            .m_axi4_lite_rdata(rdata),
                            .m_axi4_lite_rresp(rresp),
                            .m_axi4_lite_rvalid(rvalid),
                            .m_axi4_lite_rready(rready));
*/
    assign rresp = 2'b00;
    wire [23:0] pwm_out;
    wire [23:0] pwm_oeb;                            
    // sigh these are all uppercase
    axi_pwm_core u_pwm_core(.S_AXI_ACLK(clk_i),.S_AXI_ARESETN(!rst_i),
                            .S_AXI_ARADDR(araddr[8:0]),
                            .S_AXI_ARVALID(arvalid),
                            .S_AXI_ARREADY(arready),
                            .S_AXI_AWADDR(awaddr[8:0]),
                            .S_AXI_AWVALID(awvalid),
                            .S_AXI_AWREADY(awready),
                            .S_AXI_WDATA(wdata),
                            .S_AXI_WVALID(wvalid),
                            .S_AXI_WREADY(wready),
                            .S_AXI_WSTRB(wstrb),
                            .S_AXI_BRESP(bresp),
                            .S_AXI_BVALID(bvalid),
                            .S_AXI_BREADY(bready),
                            .S_AXI_RDATA(rdata),
                            .S_AXI_RVALID(rvalid),
                            .S_AXI_RREADY(rready),
                            .PWM_CLK(pwm_clk_i),
                            .PWM(pwm_out),
                            .PWM_OEB(pwm_oeb));
    generate
        genvar i;
        for (i=0;i<24;i=i+1) begin : IOB
            OBUFT u_obuf(.I(pwm_out[i]),.T(pwm_oeb[i]),.O(THRESH_PWM[i]));
        end
    endgenerate

endmodule