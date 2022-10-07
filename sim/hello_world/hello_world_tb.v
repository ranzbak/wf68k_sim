`timescale 1ns/1ns

module hello_world_tb (


);
    reg clk;




    // -- Address and data:
    wire [31:0] ADR_OUT;
    reg  [31:0] DATA_IN;
    wire [31:0] DATA_OUT;
    wire        DATA_EN;

    // -- System control:
    reg         BERRn;
    reg         RESET_INn;
    wire        RESET_OUT;
    reg         HALT_INn;
    wire        HALT_OUTn;

    // -- Processor status:
    wire [2:0]  FC_OUT;

    // -- Interrupt control:
    reg         AVECn;
    reg [2:0]   IPLn;
    wire        IPENDn;

    // -- Aynchronous bus control:
    reg [1:0]   DSACKn;
    wire[1:0]   SIZE;
    wire        ASn;
    wire        RWn;
    wire        RMCn;
    wire        DSn;
    wire        ECSn;
    wire        OCSn;
    wire        DBENn;
    wire        BUS_EN;

    // --Synchronous bus control:
    reg         STERMn;

    // -- Status controls:
    wire        STATUSn;
    wire        REFILLn;

    // -- Bus arbitration control:
    reg         BRn;
    wire        BGn;
    reg         BGACKn;

    // Generate clock signal
    initial begin
        #0 clk = 1'b0;
    end

    always begin
        #5 clk = ~clk;
    end

    // Reset signal 
    initial begin
        RESET_INn = 1'b0;
        HALT_INn = 1'b0;

        AVECn <= 1'b1;
        BERRn <= 1'b1;
        BRn <= 1'b1;
        BGACKn <= 1'b1;

        // Ack the bus for 32bit
        DSACKn[0] <= 1'b1;
        DSACKn[1] <= 1'b1;

        // Init Ack fer synchronous bus cycles
        // Signal high, this test does asynch
        STERMn <= 1'b1;

        // Release reset after 10 clock cycles
        repeat (20) @(posedge clk);
        // RESET and HALT release
        RESET_INn = ~RESET_INn;
        HALT_INn = ~HALT_INn;

    end

    /////////////////////////////////////////////////////
    // Test program for the CPU to run
    /////////////////////////////////////////////////////
    integer fd, status, k;
    localparam EX3_SIZE = 256;
    reg [7:0] ex3_memory [0:EX3_SIZE] ;
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
    // TODO: Runs at 16-bit not 32
    reg [31:0] adr_al;
    always @(posedge clk) begin
        IPLn <= 3'b111; // No interrupts pending


        // tg68_dtack <= tg68_as;
        // STERMn <= ASn;

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
                adr_al = {ADR_OUT[31:2], 2'b0};
                DATA_IN <= {ex3_memory[adr_al], ex3_memory[adr_al+1], ex3_memory[adr_al+2], ex3_memory[adr_al+3]};
            end else begin
                $display ("time=%0t a=0x%0h d=0x%0h", $time, adr_al, DATA_OUT);

                adr_al = {ADR_OUT[31:2], 2'b0};
                // Asynchronous write cycle
                if (SIZE == 2'b01) begin
                    // Byte write
                    ex3_memory[ADR_OUT] = DATA_OUT[7:0];
                end else if (SIZE == 2'b10) begin
                    // Word write
                    case (ADR_OUT[1:0]) // @suppress "Default clause missing from case statement"
                        2'b00: begin
                            ex3_memory[ADR_OUT] <= DATA_OUT[31:24];
                            ex3_memory[ADR_OUT+1] <= DATA_OUT[23:16];
                        end
                        2'b01: begin
                            ex3_memory[ADR_OUT] <= DATA_OUT[23:16];
                            ex3_memory[ADR_OUT+1] <= DATA_OUT[15:8];
                        end
                        2'b10: begin
                            ex3_memory[ADR_OUT] <= DATA_OUT[15:8];
                            ex3_memory[ADR_OUT+1] <= DATA_OUT[7:0];
                        end
                        2'b11: begin
                            ex3_memory[ADR_OUT] <= DATA_OUT[7:0];
                            ex3_memory[ADR_OUT+1] <= DATA_OUT[31:24];
                        end
                    endcase
                end else begin
                    // Long word write
                    ex3_memory[adr_al] = DATA_OUT[31:24];
                    ex3_memory[adr_al+1] = DATA_OUT[23:16];
                    ex3_memory[adr_al+2] = DATA_OUT[15:8];
                    ex3_memory[adr_al+3] = DATA_OUT[7:0];
                end

                DSACKn[0] <= 1'b0;
                DSACKn[1] <= 1'b0;

            end
        end else begin
            DATA_IN <= 32'h0;
            // if (DATA_IN == 32'h0) begin
            // 10 - LL = 32-bit, LH - 16-bit, HL = 8-bit
            DSACKn[0] <= 1'b1;
            DSACKn[1] <= 1'b1;
            // end
        end




    end

    // Finish when done
    initial begin

        wait (ADR_OUT == 'h100);

        $display("Write done to address $100, finishing simulation ...");
        $finish;
    end


    // Instanciate the WF68k030 core

    WF68K30L_TOP #(
    .NO_PIPELINE(1'b0), // If true the maincontroller workin scalarmode.
    .NO_LOOP(1'b0) // If true the DBcc loop mechanism is disabled.
    ) mycpu030 (
        .CLK(clk),

        // --Addressand data:
        .ADR_OUT(ADR_OUT),
        .DATA_IN(DATA_IN),
        .DATA_OUT(DATA_OUT),
        .DATA_EN(DATA_EN),

        // -- System control:
        .BERRn(BERRn),
        .RESET_INn(RESET_INn),
        .RESET_OUT(RESET_OUT),
        .HALT_INn(HALT_INn),
        .HALT_OUTn(HALT_OUTn),

        // -- Processor status:
        .FC_OUT(FC_OUT),

        // -- Interrupt control:
        .AVECn(AVECn),
        .IPLn(IPLn),
        .IPENDn(IPENDn),

        // -- Aynchronous bus control:
        .DSACKn(DSACKn),
        .SIZE(SIZE),
        .ASn(ASn),
        .RWn(RWn),
        .RMCn(RMCn),
        .DSn(DSn),
        .ECSn(ECSn),
        .OCSn(OCSn),
        .DBENn(DBENn),
        .BUS_EN(BUS_EN),

        // -- Synchronous bus control:
        .STERMn(STERMn),

        // -- Status controls:
        .STATUSn(STATUSn),
        .REFILLn(REFILLn),

        // -- Bus arbitration control:
        .BRn(BRn),
        .BGn(BGn),
        .BGACKn(BGACKn)
    );

endmodule