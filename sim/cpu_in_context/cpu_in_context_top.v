`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/12/2022 09:16:59 PM
// Design Name: 
// Module Name: cpu_in_context_top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module cpu_in_context_top(

);

    reg clk;
    wire clk_114;
    wire clk_28;

    wire PLL_LOCKED;

    reg reset_n;

    // Generate clock signal
    initial begin
        #0 clk = 1'b0;
    end

    always begin
        #10 clk = ~clk;
    end

    // Reset signal
    initial begin
        #0  reset_n = 1'b0;

        // Release the reset after 20 clock cycles
        repeat (20) @(posedge clk);
        reset_n = 1'b1;
    end

    // Finish when done
    initial begin

        wait (ADR_OUT[31:16] == 'h00aa);

        $display("Success!! Write done to address $%x, finishing simulation ...", ADR_OUT);
        $finish;
    end

    // When failing this exit is triggered
    initial begin

        wait (ADR_OUT[31:16] == 'h00ff);

        $display("Finished simulation with ERROR ...");
        $display("Error code: $%.4h", ADR_OUT[15:0]);
        $finish;
    end

    // SDRAM interface
    wire [16-1:0] SDRAM_DQ;
    wire [13-1:0] SDRAM_A;
    wire SDRAM_DQML;
    wire SDRAM_DQMH;
    wire SDRAM_nWE;
    wire SDRAM_nCAS;
    wire SDRAM_nRAS;
    wire SDRAM_nCS;
    wire [1:0] SDRAM_BA;
    wire SDRAM_CLK;
    wire SDRAM_CKE;

    // Block ram/rom interface
    wire [31:0]    ADR_OUT;
    wire [31:0]    DATA_IN_B;
    wire [31:0]    DATA_OUT;
    reg   [2:0]    IPLn;
    wire           DBENn_B;
    wire           DTACK_B;
    wire           DTACK_S;
    wire           ASn;
    wire           RWn_S;
    wire           RWn_B;
    // wire  [1:0]    DSACKn;
    wire  [1:0]    SIZE;
    wire           UDS;
    wire           LDS;
    wire           UDS2;
    wire           LDS2;

    always @(posedge clk) begin
        IPLn = 3'b000;
        if (reset_n == 1'b1) begin
            IPLn = 3'b111;
        end
    end

    // To the minimal minimig analog
    virtual_top #(

    ) my_virtual_top (
        .CLK_IN(clk),
        .CLK_114(CLK_114),
        .CLK_28(CLK_28),
        .PLL_LOCKED(PLL_LOCKED),
        .RESET_N(reset_n),

        .ADR_OUT(ADR_OUT),
        .DATA_IN_B(DATA_IN_B),
        .DATA_IN_S(),
        .DATA_OUT(DATA_OUT),
        .DBENn_B(DBENn_B),
        .DTACK_B(DTACK_B),
        .DTACK_S(DTACK_S),
        .ASn(ASn),
        // .DSACKn(DSACKn),
        .IPLn(IPLn),
        .RWn_S(RWn_S),
        .RWn_B(RWn_B),
        // .SIZE(SIZE),
        .UDS(UDS),
        .LDS(LDS),
        .UDS2(UDS2),
        .LDS2(LDS2),

        .SDRAM_DQ(SDRAM_DQ), // SDRAM Data bus 16 Bits
        .SDRAM_A(SDRAM_A), // SDRAM Address bus 13 Bits
        .SDRAM_DQML(SDRAM_DQML), // SDRAM Low-byte Data Mask
        .SDRAM_DQMH(SDRAM_DQMH), // SDRAM High-byte Data Mask
        .SDRAM_nWE(SDRAM_nWE), // SDRAM Write Enable
        .SDRAM_nCAS(SDRAM_nCAS), // SDRAM Column Address Strobe
        .SDRAM_nRAS(SDRAM_NRAS), // SDRAM Row Address Strobe
        .SDRAM_nCS(SDRAM_nCS), // SDRAM Chip Select
        .SDRAM_BA(SDRAM_BA), // SDRAM Bank Address
        .SDRAM_CLK(SDRAM_CLK), // SDRAM Clock
        .SDRAM_CKE(SDRAM_CKE) // SDRAM Clock Enable
    );


    // Low memory interface to simulated block rom
    blockram_030_bridge_tb #(

    ) my_blockram_tb (
        .clk(CLK_114),

        .ADR_OUT(ADR_OUT),
        .DATA_IN(DATA_IN_B),
        .DATA_OUT(DATA_OUT),
        .DBENn(DBENn_B),
        .ASn(ASn),
        .RWn(RWn_B),
        .DTACK(DTACK_B),
        .UDS(UDS),
        .LDS(LDS),
        .UDS2(UDS2),
        .LDS2(LDS2)
    );

    // Simulated SDRAM
    sdr16mx16 #(
    .addr_bits(13),
    .data_bits(16),
    .cols_bits(9),
    .mem_sizes(4194304)
    ) mysdr16mx16 (
        .Dq(SDRAM_DQ),
        .Addr(SDRAM_A),
        .Ba(SDRAM_BA),
        .Clk(SDRAM_CLK),
        .Cke(SDRAM_CKE),
        .Cs_n(SDRAM_nCS),
        .Ras_n(SDRAM_nRAS),
        .Cas_n(SDRAM_nCAS),
        .We_n(SDRAM_nWE),
        .LDQM(SDRAM_DQML),
        .UDQM(SDRAM_DQMH)
    );
endmodule
