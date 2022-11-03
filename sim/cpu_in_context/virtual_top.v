
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/12/2022 09:16:59 PM
// Design Name: 
// Module Name: virtual_top
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

`include "global_defines.v"

module virtual_top #(
    parameter hostonly = 0,
    parameter debug = 1,
    parameter havertg = 0,
    parameter haveaudio = 0,
    parameter havec2p = 0,
    parameter havespirtc = 0,
    parameter havei2c = 0,
    parameter havevpos = 0,
    parameter ram_68meg = 0
)(
    // Clock signals
    input wire            CLK_IN,
    output wire           CLK_114,
    output wire           CLK_28,
    output wire           PLL_LOCKED,
    input wire            RESET_N,

    // SDRAM
    inout  wire [ 16-1:0] SDRAM_DQ, // SDRAM Data bus 16 Bits
    output wire [ 13-1:0] SDRAM_A, // SDRAM Address bus 13 Bits
    output wire           SDRAM_DQML, // SDRAM Low-byte Data Mask
    output wire           SDRAM_DQMH, // SDRAM High-byte Data Mask
    output wire           SDRAM_nWE, // SDRAM Write Enable
    output wire           SDRAM_nCAS, // SDRAM Column Address Strobe
    output wire           SDRAM_nRAS, // SDRAM Row Address Strobe
    output wire           SDRAM_nCS, // SDRAM Chip Select
    output wire [  2-1:0] SDRAM_BA, // SDRAM Bank Address
    output wire           SDRAM_CLK, // SDRAM Clock
    output wire           SDRAM_CKE, // SDRAM Clock Enable

    // CPU memory bus rom data
    output     [31:0]   ADR_OUT,
    output reg [31:0]   DATA_IN_B,
    output reg [31:0]   DATA_IN_S,
    input      [31:0]   DATA_OUT,

    input      [2:0]    IPLn,
    output reg          DBENn_B,
    output reg          RWn_S,
    output reg          RWn_B,
    input               DTACK_B,
    input               DTACK_S,
    output              ASn,
    output              UDS,
    output              LDS,
    output              UDS2,
    output              LDS2
);
    // Clock signals
    wire           clk_sdram;

    // sdram
    wire           reset_out;
    wire [  4-1:0] sdram_cs;
    wire [  2-1:0] sdram_dqm;
    wire [  2-1:0] sdram_ba;

    // SDRAM
    assign SDRAM_CKE        = 1'b1;
    assign SDRAM_CLK        = clk_sdram;
    assign SDRAM_nCS        = sdram_cs[0];
    assign SDRAM_DQML       = sdram_dqm[0];
    assign SDRAM_DQMH       = sdram_dqm[1];
    assign SDRAM_BA         = sdram_ba;

    // tg68
    wire           tg68_rst;
    reg  [ 16-1:0] tg68_dat_in;
    reg  [ 16-1:0] tg68_dat_in2;
    wire [ 16-1:0] tg68_dat_out;
    wire [ 16-1:0] tg68_dat_out2;
    wire [ 22-1:0] ram_address;
    wire           _ram_bhe = 1'b1; // Disable all bytes
    wire           _ram_ble = 1'b1;
    wire           _ram_bhe2 = 1'b1;
    wire           _ram_ble2 = 1'b1;
    wire           _ram_we = 1'b1; // Read at all times
    wire           _ram_oe = 1'b1; // disable output enable 
    wire [ 32-1:0] tg68_adr;
    wire [  3-1:0] tg68_IPL;
    reg            tg68_dtack;
    wire           tg68_as;
    wire           tg68_uds;
    wire           tg68_lds;
    wire           tg68_uds2;
    wire           tg68_lds2;
    wire           tg68_rw;
    wire           tg68_ena7RD;
    wire           tg68_ena7WR;
    wire           tg68_ena28;
    wire [ 32-1:0] tg68_cout;
    wire [ 32-1:0] tg68_cin;
    wire           tg68_cpuena;
    wire [  4-1:0] cpu_config;
    wire [4:0]     board_configured;
    wire           turbochipram;
    wire           turbokick;
    wire [1:0]     slow_config;
    wire           aga;
    wire           cache_inhibit;
    wire           cacheline_clr;
    wire [ 32-1:0] tg68_cad;
    wire [  7-1:0] tg68_cpustate;
    wire           tg68_nrst_out;
    //wire           tg68_cdma;
    wire           tg68_clds;
    wire           tg68_cuds;
    wire [ 14-1:0] tg68_CACR_out;
    wire [ 32-1:0] tg68_VBR_out;
    wire           tg68_ovr;

    wire           RWn;
    // wire           DBENn;
    reg  [ 32-1:0] DATA_IN;

    // MCMM Clock manager core
    amiga_clk amiga_clk (
        .rst          (1'b0             ), // async reset input
        .clk_in       (CLK_IN           ), // input clock     ( 50.000000MHz)
        .clk_114      (CLK_114          ), // output clock c0 (114.750000MHz)
        .clk_sdram    (clk_sdram        ), // output clock c2 (114.750000MHz, -146.25 deg)
        .clk_28       (CLK_28           ), // output clock c1 ( 28.687500MHz)
        .clk7_en      (clk7_en          ), // output clock 7 enable (on 28MHz clock domain)
        .clk7n_en     (clk7n_en         ), // 7MHz negedge output clock enable (on 28MHz clock domain)
        .c1           (c1               ), // clk28m clock domain signal synchronous with clk signal
        .c3           (c3               ), // clk28m clock domain signal synchronous with clk signal delayed by 90 degrees
        .cck          (cck              ), // colour clock output (3.54 MHz)
        .eclk         (eclk             ), // 0.709379 MHz clock enable output (clk domain pulse)
        .locked       (PLL_LOCKED       ), // pll locked output
        .ntsc         (                 )
    );

    // CPU Wrapper with new CPU
    WF68K_interface
    #(
    .havertg(1'b0),
    .haveaudio(1'b0),
    .havec2p(1'b0)
    ) wf68k (
        .clk          (CLK_114          ),
        .nReset       (tg68_rst         ),
        .clkena_in    (tg68_ena28       ),
        .IPL          (tg68_IPL         ),
        .dtackn        (tg68_dtack       ),
        .addr         (tg68_adr         ),
        .data_read    (tg68_dat_in      ),
        .data_read2   (tg68_dat_in2     ),
        .data_write   (tg68_dat_out     ),
        .data_write2  (tg68_dat_out2    ),
        .as           (tg68_as          ),
        .uds          (tg68_uds         ),
        .lds          (tg68_lds         ),
        .uds2         (tg68_uds2        ),
        .lds2         (tg68_lds2        ),
        .rw           (tg68_rw          ),
        .ena7RDreg    (tg68_ena7RD      ),
        .ena7WRreg    (tg68_ena7WR      ),
        .fromram      (tg68_cout        ),
        .toram        (tg68_cin         ),
        .ramready     (tg68_cpuena      ),
        .turbochipram (turbochipram     ),
        .turbokick    (turbokick        ),
        .slow_config  (slow_config      ),
        .aga          (aga              ),
        .cache_inhibit(cache_inhibit    ),
        .cacheline_clr(cacheline_clr    ),
        .ziiram_active(board_configured[0]),
        .ziiiram_active(board_configured[1]),
        .ziiiram2_active(board_configured[2]),
        .ziiiram3_active(board_configured[3]),
        //  .fastramcfg   ({&memcfg[5:4],memcfg[5:4]}),
        .eth_en       (1'b1), // TODO eth in the future?
        .sel_eth      (),
        .frometh      (16'b0),
        .ethready     (1'b0),
        .ramaddr      (tg68_cad         ),
        .cpustate     (tg68_cpustate    ),
        .nResetOut    (tg68_nrst_out    ),
        // .skipFetch    (                 ),
        .ramlds       (tg68_clds        ),
        .ramuds       (tg68_cuds        ),
        .CACR_out     (tg68_CACR_out    ),
        .VBR_out      (tg68_VBR_out     ),
        // RTG signals
        .rtg_addr(),
        .rtg_vbend(),
        .rtg_ext(),
        .rtg_pixelclock(),
        .rtg_clut(),
        .rtg_16bit(),
        .rtg_clut_idx(),
        .rtg_clut_r(),
        .rtg_clut_g(),
        .rtg_clut_b(),
        .audio_buf(),
        .audio_ena(),
        .audio_int(),
        // Amiga to host signals
        .host_req(),
        .host_ack(),
        .host_q()
    );

    /**
     * Switch between the block RAM and SDRAM bus
     * The bottom 8Kb (12-bits are block RAM), above that SDRAM
     */
    reg [1:0] DBENn_S; // Enable SDRAM

    // reg [31:0] DATA_IN_S; // Data out (to CPU) SDRAM

    always @(posedge CLK_114) begin
        if (~RESET_N) begin

        end else if (tg68_adr[31:19]==12'h000) begin
            DBENn_S = 1'b1; // Keep SDRAM disabled
            DBENn_B <= 1'b0; // Allow control from block RAM
            DATA_IN <= DATA_IN_B; // DATA From block ram
            tg68_dat_in <= DATA_OUT[15:0]; // Data in low
            tg68_dat_in2 <=  DATA_OUT[31:16]; // Data in high
            DATA_IN_B[15:0] <= tg68_dat_out; // Data out low
            DATA_IN_B[31:16] <= tg68_dat_out2; // Data out high
            RWn_B   <= RWn; // Read write to block RAM
            RWn_S   <= 1'b1; // Keep SDRAM in read
            tg68_dtack <= DTACK_B; // Request ack from block RAM/ROM
        end else begin
            DBENn_S <= 1'b0; // Allow control from SDRAM
            DBENn_B <= 1'b1;
            // DATA_IN <= DATA_IN_S; // Data from SDRAM
            DATA_IN_S[31:16] <= tg68_dat_out;
            DATA_IN_S[15:0] <= tg68_dat_out2;
            RWn_B   <= 1'b1; // Keep block ram in read
            RWn_S   <= RWn; // Data from SDRAM
            tg68_dtack <= DTACK_S; // Data ack not active from block ram
        end
    end

    sdram_ctrl sdram (
        .cache_rst    (tg68_rst         ),
        .cache_inhibit(cache_inhibit    ),
        .cacheline_clr(cacheline_clr    ),
        .cpu_cache_ctrl (tg68_CACR_out    ),

        // Interface to SDRam
        .sdata        (SDRAM_DQ         ),
        .sdaddr       (SDRAM_A[12:0]    ),
        .dqm          (sdram_dqm        ),
        .sd_cs        (sdram_cs         ),
        .ba           (sdram_ba         ),
        .sd_we        (SDRAM_nWE        ),
        .sd_ras       (SDRAM_nRAS       ),
        .sd_cas       (SDRAM_nCAS       ),
        .sysclk       (CLK_114          ),
        .reset_in     (sdctl_rst        ),

        // Host CPU
        .hostWR       (hostWR           ),
        .hostAddr     (hostaddr         ),
        .hostwe       (host_we           ),
        .hostce       (host_ramreq      ),
        .hostbytesel  (hostbytesel      ),
        .hostRD       (host_ramdata     ),
        .hostena      (host_ramack      ),

        // Amiga CPU
        .cpuWR        (tg68_cin         ), // TODO: 32bit wide
        .cpuAddr      (tg68_cad[31:2]   ), // TODO: 32bit wide?
        .cpuU         (tg68_cuds        ), // TODO: SIZE and DTACK?
        .cpuL         (tg68_clds        ),
        .cpuU2        (tg68_cuds2       ), // TODO: SIZE and DTACK?
        .cpuL2        (tg68_clds2       ),
        .cpustate     (tg68_cpustate    ),
        .cpuRD        (tg68_cout        ), // TODO: 32bit wide
        .cpuena       (tg68_cpuena      ),

        // Amiga chip ram
        // TODO enable later 
        //  .cpu_dma      (tg68_cdma        ),
        .chipWR       (ram_data         ),
        .chipWR2      (tg68_dat_out2    ),
        .chipAddr     ({1'b0, ram_address[22:1]}),
        .chipU        (_ram_bhe         ),
        .chipL        (_ram_ble         ),
        .chipU2       (_ram_bhe2        ),
        .chipL2       (_ram_ble2        ),
        .chipRW       (_ram_we          ),
        .chip_dma     (_ram_oe          ),
        .clk7_en      (clk7_en          ),
        .chipRD       (ramdata_in       ),
        .chip48       (chip48           ),

        // RTG memory
        .rtgAddr      (rtg_addr_mangled ),
        .rtgce        (rtg_ramreq       ),
        .rtgfill      (rtg_fill         ),
        .rtgRd        (rtg_fromram      ),

        // Audio memory
        .audAddr      (aud_ramaddr      ),
        .audce        (aud_ramreq       ),
        .audfill      (aud_fill         ),
        .audRd        (aud_fromram      ),

        .reset_out    (reset_out        ),
        .enaWRreg     (tg68_ena28       ),
        .ena7RDreg    (tg68_ena7RD      ),
        .ena7WRreg    (tg68_ena7WR      )
    );

    // Assign incoming signals
    assign tg68_rst = RESET_N;
    assign ASn = tg68_as;

    // Assign output signals
    assign ADR_OUT = tg68_adr;
    assign RWn = tg68_rw;
    assign UDS = tg68_uds;
    assign LDS = tg68_lds;
    assign UDS2 = tg68_uds2;
    assign LDS2 = tg68_lds2;

    assign tg68_IPL = IPLn; // TODO Solve error here

endmodule