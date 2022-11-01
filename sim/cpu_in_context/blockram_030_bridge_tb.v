`timescale 1ns/1ns

module blockram_030_bridge_tb #(
    parameter BIN_FILE="my_file.bin",
    parameter EX3_SIZE=1024
) (
    input               clk,

    input       [31:0]  ADR_OUT,
    input       [31:0]  DATA_IN,
    output reg  [31:0]  DATA_OUT,

    output              DBENn,
    input               ASn,
    input               RWn,
    output reg          DTACK,
    input               UDS,
    input               LDS,
    input               UDS2,
    input               LDS2
);

    // RAM array
    reg [7:0] ex3_memory [0:EX3_SIZE];

    // Read bin data into RAM array
    integer fd, status, k;
    initial begin
        for (k=0; k<EX3_SIZE; k=k+1) begin
            ex3_memory[k] = 8'h00;
        end

        fd = $fopen (
            // relative in the future?
            "/home/paul/work/fpga/Xilinx/wf68k30/software/asm/basic-load.bin",
            "rb");

        // $readmemb("asm/basic-load.bin", ex3_memory);
        status = $fread(ex3_memory, fd);
    end

    // Setup the cpu ports
    // Synchronous signaling
    reg [31:0] adr_al;
    reg [1:0] adr_of;

    // Split the bus into bytes
    wire [7:0] data_in_b [0:3];

    assign {data_in_b[0], data_in_b[1], data_in_b[2], data_in_b[3]} = DATA_IN;

    // Translate the 
    reg [1:0] cur_offset;
    always @(posedge clk) begin
        // tg68_dtack <= tg68_as;
        // STERMn <= ASn;
        adr_al = {ADR_OUT[31:2], 2'b0};
        adr_of = ADR_OUT[1:0];

        // Ack the bus for 32bit
        if (~DBENn && ~ASn) begin
            if (RWn == 1'b1) begin
                // Asynchronous Read cycle
                // Set the correct data on the data bus
                // if (DATA_OUT != 32'b0) begin
                DTACK <= 1'b0;
                // end

                // Allign reads to long words
                DATA_OUT <= {ex3_memory[adr_al], ex3_memory[adr_al+1], ex3_memory[adr_al+2], ex3_memory[adr_al+3]};
            end else begin

                $display ("time=%0t a=0x%0h d=0x%0h", $time, adr_al, DATA_OUT);
                // Write the right bytes
                if (~UDS)  ex3_memory[adr_al] <= data_in_b[0];
                if (~LDS)  ex3_memory[adr_al+1] <= data_in_b[1];
                if (~UDS2) ex3_memory[adr_al+2] <= data_in_b[2];
                if (~LDS2) ex3_memory[adr_al+3] <= data_in_b[3];

                DTACK <= 1'b0;

            end
        end else begin
            DATA_OUT <= 32'h0;
            DTACK <=  1'b1;
        end
    end
endmodule