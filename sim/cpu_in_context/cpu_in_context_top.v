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
    reg reset_n;

    // Generate clock signal
    initial begin
        #0 clk = 1'b0;
    end

    always begin
        #5 clk = ~clk;
    end

    // Reset signal
    initial begin
        #0  reset_n = 1'b0;

        // Release the reset after 20 clock cycles
        repeat (20) @(posedge clk);
        reset_n = 1'b1;
    end

    // To the minimal minimig analog
    virtual_top #(

    ) my_virtual_top (
        .CLK_IN(clk),
        .CLK_114(),
        .CLK_28(),
        .PLL_LOCKED(),
        .RESET_N(),

        .SDRAM_DQ(), // SDRAM Data bus 16 Bits
        .SDRAM_A(), // SDRAM Address bus 13 Bits
        .SDRAM_DQML(), // SDRAM Low-byte Data Mask
        .SDRAM_DQMH(), // SDRAM High-byte Data Mask
        .SDRAM_nWE(), // SDRAM Write Enable
        .SDRAM_nCAS(), // SDRAM Column Address Strobe
        .SDRAM_nRAS(), // SDRAM Row Address Strobe
        .SDRAM_nCS(), // SDRAM Chip Select
        .SDRAM_BA(), // SDRAM Bank Address
        .SDRAM_CLK(), // SDRAM Clock
        .SDRAM_CKE() // SDRAM Clock Enable
    );

    // Simulated SDRAM
    sdr16mx16 #(
    .addr_bits(13),
    .data_bits(16),
    .cols_bits(9),
    .mem_sizes(4194304)
    ) mysdr16mx16 (
        .Dq(),
        .Addr(),
        .Ba(),
        .Clk(clk),
        .Cke(),
        .Cs_n(),
        .Ras_n(),
        .Cas_n(),
        .We_n(),
        .LDQM(),
        .UDQM()
    );
endmodule
