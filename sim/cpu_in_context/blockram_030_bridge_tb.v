`timescale 1ns/1ns

module blockram_030_bridge_tb #(
    parameter BIN_FILE="my_file.bin",
    parameter EX3_SIZE=256
) (
    input               clk,

    input     [31:0]    ADR_OUT,
    output    [31:0]    DATA_IN,
    input     [31:0]    DATA_OUT,

    output     [2:0]    IPLn,
    output              DBENn,
    input               RWn,
    output reg [1:0]    DSACKn,
    input      [1:0]    SIZE
);

    // RAM array
    reg [7:0] ex3_memory [0:EX3_SIZE] ;

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
    reg [7:0] data_out_b [0:3];

    // Translate the 
    always @(posedge clk) begin
        IPLn <= 3'b111; // No interrupts pending

        // tg68_dtack <= tg68_as;
        // STERMn <= ASn;
        {data_out_b[0], data_out_b[1], data_out_b[2], data_out_b[3]} = DATA_OUT;
        adr_al = {ADR_OUT[31:2], 2'b0};
        adr_of = ADR_OUT[1:0];

        // Ack the bus for 32bit
        if (~DBENn) begin
            if (RWn == 1'b1) begin
                // Asynchronous Read cycle
                // Set the correct data on the data bus
                if (DATA_IN != 32'b0) begin
                    DSACKn[0] <= 1'b0;
                    DSACKn[1] <= 1'b0;
                end

                // Allign reads to long words
                DATA_IN <= {ex3_memory[adr_al], ex3_memory[adr_al+1], ex3_memory[adr_al+2], ex3_memory[adr_al+3]};
            end else begin
                $display ("time=%0t a=0x%0h d=0x%0h", $time, adr_al, DATA_OUT);

                // Asynchronous write cycle
                case (SIZE) // @suppress "Default clause missing from case statement"
                    2'b00: begin
                        // Long word write
                        case (adr_of) // @suppress "Default clause missing from case statement"
                            2'b00: begin
                                ex3_memory[ADR_OUT]   = data_out_b[0];
                                ex3_memory[ADR_OUT+1] = data_out_b[1];
                                ex3_memory[ADR_OUT+2] = data_out_b[2];
                                ex3_memory[ADR_OUT+3] = data_out_b[3];
                            end
                            2'b01: begin
                                ex3_memory[ADR_OUT]   = data_out_b[1];
                                ex3_memory[ADR_OUT+1] = data_out_b[2];
                                ex3_memory[ADR_OUT+2] = data_out_b[3];
                            end
                            2'b10: begin
                                ex3_memory[ADR_OUT]   = data_out_b[2];
                                ex3_memory[ADR_OUT+1] = data_out_b[3];
                            end
                            2'b11: begin
                                ex3_memory[ADR_OUT]   = data_out_b[3];
                            end
                        endcase
                    end
                    2'b01: begin
                        // Byte write
                        ex3_memory[ADR_OUT] = data_out_b[adr_of];
                    end
                    2'b10: begin
                        // Word write
                        case (adr_of) // @suppress "Default clause missing from case statement"
                            2'b00,
                            2'b01,
                            2'b10: begin
                                ex3_memory[ADR_OUT] <= data_out_b[adr_of];
                                ex3_memory[ADR_OUT+1] <= data_out_b[adr_of+1];
                            end
                            2'b11: begin
                                ex3_memory[ADR_OUT] <= data_out_b[adr_of];
                            end
                        endcase
                    end
                    2'b11: begin
                        // Three byte transfer
                        case (adr_of) // @suppress "Default clause missing from case statement"
                            2'b00,
                            2'b01: begin
                                ex3_memory[ADR_OUT] <= data_out_b[adr_of];
                                ex3_memory[ADR_OUT+1] <= data_out_b[adr_of+1];
                                ex3_memory[ADR_OUT+2] <= data_out_b[adr_of+2];
                            end
                            2'b10: begin
                                ex3_memory[ADR_OUT] <= data_out_b[adr_of];
                                ex3_memory[ADR_OUT+1] <= data_out_b[adr_of+1];
                            end
                            2'b11: begin
                                ex3_memory[ADR_OUT] <= data_out_b[adr_of];
                            end
                        endcase


                    end
                endcase

                DSACKn[0] <= 1'b0;
                DSACKn[1] <= 1'b0;

            end
        end else begin
            DATA_IN <= 32'h0;
            // if (DATA_IN == 32'h0) begin
            // 10 - LL = 32-bit, LH - 16-bit, HL = 8-bit
            DSACKn[0] <= 1'b1;
            DSACKn[1] <= 1'b1;
        end
    end
endmodule