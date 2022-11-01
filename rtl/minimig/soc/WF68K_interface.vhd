------------------------------------------------------------------------------
------------------------------------------------------------------------------
--                                                                          --
-- Copyright (c) 2009-2011 Tobias Gubener                                   --
-- Subdesign fAMpIGA by TobiFlex                                            --
--                                                                          --
-- This is the TOP-Level for TG68KdotC_Kernel to generate 68K Bus signals   --
--                                                                          --
-- This source file is free software: you can redistribute it and/or modify --
-- it under the terms of the GNU General Public License as published        --
-- by the Free Software Foundation, either version 3 of the License, or     --
-- (at your option) any later version.                                      --
--                                                                          --
-- This source file is distributed in the hope that it will be useful,      --
-- but WITHOUT ANY WARRANTY; without even the implied warranty of           --
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            --
-- GNU General Public License for more details.                             --
--                                                                          --
-- You should have received a copy of the GNU General Public License        --
-- along with this program.  If not, see <http://www.gnu.org/licenses/>.    --
--                                                                          --
------------------------------------------------------------------------------
------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;        -- @suppress "Deprecated package"

entity WF68K_interface is
    generic(
        havertg   : boolean := true;
        haveaudio : boolean := true;
        havec2p   : boolean := true
    );
    port(
        clk             : in     std_logic;
        nReset          : in     std_logic;
        clkena_in       : in     std_logic                     := '1'; -- sdram/enaWRreg
        -- Minimig interface
        IPL             : in     std_logic_vector(2 downto 0)  := "111"; -- minimig/cpu_ipl
        dtackn          : in     std_logic; -- minimig/cpu_dtack -> minimig/CPU1/dtack
        -- vpa             : in     std_logic                     := '1'; -- unused const 1
        -- ein             : in     std_logic                     := '1'; -- unused const 1
        addr            : out    std_logic_vector(31 downto 0); -- minimig/cpu_address
        data_read       : in     std_logic_vector(15 downto 0); -- From CPU bridge Amiga mimimig/cpu_data
        data_read2      : in     std_logic_vector(15 downto 0); -- From CPU bridge Amiga mimimig/cpu_data2
        data_write      : out    std_logic_vector(15 downto 0); -- To Amiga minimig/cpudata_in
        data_write2     : out    std_logic_vector(15 downto 0); -- To SDRAM sdram/chipWR2
        as              : out    std_logic; -- minimig/cpu_as -> minimig_m68k_bridge/as & cart/cpu_as
        uds             : out    std_logic; -- minimig/cpu_uds -> minimig_m68k_bridge/uds
        lds             : out    std_logic; -- mimimig/cpu_lds -> minimig_m68k_bridge/lds
        uds2            : out    std_logic; -- mimimig/cpu_uds2 -> minimig_m68k_bridge/uds2
        lds2            : out    std_logic; -- minimig/cpu_lds2 -> minimigig_m68k_bridge/lds2
        rw              : out    std_logic; -- minimig/cpu_rw -> minimigig_m68k_bridge/r_w
        -- vma             : buffer std_logic                     := '1'; -- not used
        -- wrd             : out    std_logic; -- not used
        ena7RDreg       : in     std_logic                     := '1'; -- 1 every 4 cycles enable
        ena7WRreg       : in     std_logic                     := '1'; -- 1 every 4 cycles enable
        fromram         : in     std_logic_vector(31 downto 0); -- From CPU cache sdram/cpuRD
        toram           : out    std_logic_vector(31 downto 0); -- To CPU cache sdram/cpuWR
        ramready        : in     std_logic                     := '0'; -- sdram/cpuena -> cpu_cache/cpu_ack
        -- cpu             : in     std_logic_vector(1 downto 0); -- Not going to use 68030 always
        -- autoconfig interface
        ziiram_active   : in     std_logic;
        ziiiram_active  : in     std_logic;
        ziiiram2_active : in     std_logic;
        ziiiram3_active : in     std_logic;
        eth_en          : in     std_logic                     := '0'; -- @suppress "Unused port: eth_en is not used in work.TG68K(logic)"
        sel_eth         : buffer std_logic;
        frometh         : in     std_logic_vector(15 downto 0);
        ethready        : in     std_logic;
        slow_config     : in     std_logic_vector(1 downto 0);
        aga             : in     std_logic;
        turbochipram    : in     std_logic;
        turbokick       : in     std_logic;
        -- SDRAM interface
        cache_inhibit   : out    std_logic;
        cacheline_clr   : out    std_logic;
        --    ovr           : in      std_logic;
        ramaddr         : out    std_logic_vector(31 downto 0); -- sdram/cpuAddr[25:1] & mycfide/amiga_addr 
        cpustate        : out    std_logic_vector(6 downto 0); -- sdram/cpustate[6:0] & mycfide/amiga_wr
        nResetOut       : out    std_logic;
        --    cpuDMA        : buffer  std_logic;
        ramlds          : out    std_logic; -- sdram/cpuL
        ramuds          : out    std_logic; -- sdram/cpuU
        CAAR_OUT        : out    std_logic_vector(31 downto 0); -- Cache Address Register
        CACR_OUT        : out    std_logic_vector(13 downto 0); -- CACR_out [clear cache, clear entry in cache, freeze cache, enable cache]
        VBR_OUT         : out    std_logic_vector(31 downto 0); -- Vector base register: mimimig/cpu_vbr -> CART1/cpu_vbr
        -- RTG interface
        rtg_addr        : out    std_logic_vector(25 downto 4);
        rtg_vbend       : out    std_logic_vector(6 downto 0);
        rtg_ext         : out    std_logic;
        rtg_pixelclock  : out    std_logic_vector(3 downto 0);
        rtg_clut        : out    std_logic;
        rtg_16bit       : out    std_logic;
        rtg_clut_idx    : in     std_logic_vector(7 downto 0)  := X"00";
        rtg_clut_r      : out    std_logic_vector(7 downto 0);
        rtg_clut_g      : out    std_logic_vector(7 downto 0);
        rtg_clut_b      : out    std_logic_vector(7 downto 0);
        -- Audio interface
        audio_buf       : in     std_logic;
        audio_ena       : out    std_logic;
        audio_int       : out    std_logic;
        -- Host interface
        host_req        : out    std_logic;
        host_ack        : in     std_logic                     := '0';
        host_q          : in     std_logic_vector(15 downto 0) := "----------------"
    );
end WF68K_interface;

ARCHITECTURE logic OF WF68K_interface IS

    SIGNAL cpuaddr     : std_logic_vector(31 downto 0);
    SIGNAL r_data      : std_logic_vector(15 downto 0);
    SIGNAL cpuIPL      : std_logic_vector(2 downto 0);
    -- SIGNAL vpad             : std_logic;
    SIGNAL waitm       : std_logic;
    SIGNAL clkena_e    : std_logic;
    SIGNAL clkena_f    : std_logic;
    SIGNAL S_state     : std_logic_vector(1 downto 0);
    -- SIGNAL decode           : std_logic;
    SIGNAL wr          : std_logic;
    SIGNAL uds_in      : std_logic;
    SIGNAL lds_in      : std_logic;
    SIGNAL state       : std_logic_vector(1 downto 0); -- 00-> fetch code 10->read data 11->write data 01->no memaccess
    signal longword    : std_logic;
    SIGNAL clkena      : std_logic;
    -- SIGNAL vmaena           : std_logic;
    -- SIGNAL eind        : std_logic;
    -- SIGNAL eindd       : std_logic;
    SIGNAL sel_ram     : std_logic;
    SIGNAL sel_chip    : std_logic;
    SIGNAL sel_chipram : std_logic;
    -- SIGNAL turbochip_ena    : std_logic := '0';
    SIGNAL turbochip_d : std_logic := '0';
    SIGNAL turbokick_d : std_logic := '0';
    SIGNAL turboslow_d : std_logic := '0';
    SIGNAL slower      : std_logic_vector(3 downto 0);

    TYPE sync_states IS (sync0, sync1, sync2, sync3, sync4, sync5, sync6, sync7, sync8, sync9);
    SIGNAL sync_state : sync_states;
    SIGNAL datatg68_c : std_logic_vector(31 downto 0);
    SIGNAL datatg68   : std_logic_vector(31 downto 0);
    SIGNAL w_datatg68 : std_logic_vector(31 downto 0);
    SIGNAL ramcs      : std_logic;
    SIGNAL dsackn     : std_logic_vector(1 downto 0);
    SIGNAL ipendn     : std_logic;

    SIGNAL z2ram_ena       : std_logic;
    SIGNAL z3ram_ena       : std_logic;
    SIGNAL z3ram2_ena      : std_logic;
    SIGNAL z3ram3_ena      : std_logic;
    -- SIGNAL eth_base         : std_logic_vector(7 downto 0);
    -- SIGNAL eth_cfgd         : std_logic;
    SIGNAL sel_z2ram       : std_logic;
    SIGNAL sel_z3ram       : std_logic;
    SIGNAL sel_z3ram2      : std_logic;
    SIGNAL sel_z3ram3      : std_logic;
    SIGNAL sel_kick        : std_logic;
    SIGNAL sel_kickram     : std_logic;
    --SIGNAL sel_eth          : std_logic;
    SIGNAL sel_slow        : std_logic;
    SIGNAL sel_slowram     : std_logic;
    -- SIGNAL sel_cart         : std_logic; 
    SIGNAL sel_32          : std_logic;
    signal sel_undecoded   : std_logic;
    signal sel_undecoded_d : std_logic;
    signal sel_akiko       : std_logic;
    signal sel_akiko_d     : std_logic;
    signal sel_audio       : std_logic;
    signal sel_ram_d       : std_logic;
    SIGNAL cpu_int         : std_logic;
    signal cpu_siz         : std_logic_vector(1 downto 0);

    -- Akiko registers
    signal akiko_d    : std_logic_vector(15 downto 0);
    signal akiko_q    : std_logic_vector(15 downto 0);
    signal akiko_wr   : std_logic;
    signal akiko_req  : std_logic;
    signal akiko_ack  : std_logic;
    signal host_req_r : std_logic;

    SIGNAL NMI_addr            : std_logic_vector(31 downto 0);
    SIGNAL sel_nmi_vector_addr : std_logic;
    SIGNAL sel_nmi_vector      : std_logic;

    -- signal chipset_cycle : std_logic;
    signal nResetOut_w : std_logic;
    signal VBR_OUT_w   : std_logic_vector(31 downto 0);
    signal OPCn        : std_logic;
    signal sizesel     : std_logic_vector(3 downto 0);
    signal statesel    : std_logic_vector(1 downto 0);
    signal dtacksel    : std_logic_vector(1 downto 0);
    signal sel         : std_logic_vector(4 downto 0);
    signal rw_d        : std_logic;

BEGIN

    nResetOut <= nResetOut_w;
    VBR_OUT   <= VBR_OUT_w;

    sel_eth <= '0';

    -- NMI
    PROCESS(nReset, clk)
    BEGIN
        IF nReset = '0' THEN
            NMI_addr            <= X"0000007c";
            sel_nmi_vector_addr <= '0';
        ELSIF rising_edge(clk) THEN
            NMI_addr            <= VBR_OUT_w + X"0000007c"; -- calculate NMI address
            sel_nmi_vector_addr <= '0';
            IF (cpuaddr(31 downto 2) = NMI_addr(31 downto 2)) THEN
                sel_nmi_vector_addr <= '1';
            END IF;
        END IF;
    END PROCESS;

    sel_nmi_vector <= '1' WHEN sel_nmi_vector_addr = '1' AND state = "10" ELSE '0';

    toram       <= w_datatg68;          -- Data directly to the SDRAM2
    data_write  <= w_datatg68(15 downto 0);
    data_write2 <= w_datatg68(31 downto 16);
    -- wrd     <= wr;
    cpu_int     <= '1' WHEN state = "01" else '0';
    PROCESS(clk)
    BEGIN
        IF rising_edge(clk) THEN
            z2ram_ena  <= ziiram_active;
            z3ram_ena  <= ziiiram_active;
            z3ram2_ena <= ziiiram2_active;
            z3ram3_ena <= ziiiram3_active;

            sel_akiko_d     <= sel_akiko;
            sel_undecoded_d <= sel_undecoded;
        END IF;
    END PROCESS;

    -- Switch between the different input sources for data towards the cpu
    datatg68 <= fromram WHEN cpu_int = '0' AND sel_ram_d = '1' AND sel_nmi_vector = '0' ELSE datatg68_c;

    -- Register incoming data
    process(clk)
    begin
        if rising_edge(clk) then
            if sel_undecoded = '1' then
                datatg68_c <= X"FFFFFFFF";
            elsif sel_akiko_d = '1' then
                datatg68_c <= akiko_q & akiko_q; -- TODO: Make sure the SIZE is set to 16-bit when selecting this
            elsif sel_eth = '1' then
                datatg68_c <= frometh & frometh;
            else
                datatg68_c <= data_read2 & data_read; -- TODO: Right way around? Corrent values in time?
            end if;
        end if;
    end process;

    sel_akiko     <= '1' when cpuaddr(31 downto 16) = X"00B8" else '0';
    sel_32        <= '1' when cpuaddr(31 downto 24) /= X"00" and cpuaddr(31 downto 24) /= X"ff" else '0'; -- Decode 32-bit space, but exclude interrupt vectors
    --  sel_z3ram       <= '1' WHEN (cpuaddr(31 downto 24)=z3ram_base) else '0'; -- AND z3ram_ena='1' ELSE '0';
    -- First block of ZIII RAM - 0x40000000 - 0x40ffffff
    sel_z3ram     <= '1' WHEN (cpuaddr(31 downto 30) = "01") and cpuaddr(26 downto 24) = "000" AND z3ram_ena = '1' ELSE '0';
    -- Second block of ZIII RAM - 32 meg from 0x42000000 - 0x43ffffff
    sel_z3ram2    <= '1' WHEN (cpuaddr(31 downto 30) = "01") and cpuaddr(25) = '1' AND z3ram2_ena = '1' ELSE '0';
    -- Third block of ZIII RAM - either 2 or 4 meg, starting at either 0x41000000 or 0x44000000
    sel_z3ram3    <= '1' WHEN (cpuaddr(31 downto 30) = "01") and cpuaddr(26) = z3ram2_ena and cpuaddr(24) = not z3ram2_ena and z3ram3_ena = '1' ELSE '0';
    sel_z2ram     <= '1' WHEN (cpuaddr(31 downto 24) = X"00") AND ((cpuaddr(23 downto 21) = "001") OR (cpuaddr(23 downto 21) = "010") OR (cpuaddr(23 downto 21) = "011") OR (cpuaddr(23 downto 21) = "100")) AND z2ram_ena = '1' ELSE '0';
    --sel_eth         <= '1' WHEN (cpuaddr(31 downto 24) = eth_base) AND eth_cfgd='1' ELSE '0';
    sel_chip      <= '1' WHEN (cpuaddr(31 downto 24) = X"00") AND (cpuaddr(23 downto 21) = "000") ELSE '0'; --$000000 - $1FFFFF
    sel_chipram   <= '1' WHEN sel_chip = '1' AND turbochip_d = '1' ELSE '0';
    sel_kick      <= '1' WHEN (cpuaddr(31 downto 24) = X"00") AND ((cpuaddr(23 downto 19) = "11111") OR (cpuaddr(23 downto 19) = "11100")) AND state /= "11" ELSE '0'; -- $F8xxxx, $E0xxxx, read only
    sel_kickram   <= '1' WHEN sel_kick = '1' AND turbokick_d = '1' ELSE '0';
    sel_slow      <= '1' WHEN (cpuaddr(31 downto 24) = X"00") AND ((cpuaddr(23 downto 20) = X"C" AND ((cpuaddr(19) = '0' AND slow_config /= "00") OR (cpuaddr(19) = '1' AND slow_config(1) = '1'))) OR (cpuaddr(23 downto 19) = X"D" & '0' AND slow_config = "11")) ELSE '0'; -- $C00000 - $D7FFFF
    sel_slowram   <= '1' WHEN sel_slow = '1' AND turboslow_d = '1' ELSE '0';
    -- sel_cart        <= '1' WHEN (cpuaddr(31 downto 24) = X"00") AND (cpuaddr(23 downto 20)="1010") ELSE '0'; -- $A00000 - $A7FFFF (actually matches up to $AFFFFF)
    sel_audio     <= '1' WHEN (cpuaddr(31 downto 24) = X"00") AND (cpuaddr(23 downto 18) = "111011") ELSE '0'; -- $EC0000 - $EFFFFF
    sel_undecoded <= '1' WHEN sel_32 = '1' and sel_z3ram = '0' and sel_z3ram2 = '0' and sel_z3ram3 = '0' else '0';
    sel_ram       <= '1' WHEN (sel_z2ram = '1' OR sel_z3ram = '1' OR sel_z3ram2 = '1' OR sel_z3ram3 = '1' OR sel_chipram = '1' OR sel_slowram = '1' OR sel_kickram = '1' OR sel_audio = '1') ELSE
                     '0';

    cache_inhibit <= '1' WHEN sel_kickram = '1' ELSE '0';

    ramcs <= NOT (NOT cpu_int AND sel_ram_d AND NOT sel_nmi_vector) OR slower(0);

    cpustate <= longword & clkena & slower(1 downto 0) & ramcs & state(1 downto 0);
    ramlds   <= lds_in;
    ramuds   <= uds_in;

    -- This is the mapping to the SDRAM
    -- map $00-$1F to $00-$1F (chipram), $A0-$FF to $20-$7F. All non-fastram goes into the first
    -- 8M block (i.e. SDRAM bank 0). This map should be the same as in minimig_sram_bridge.v
    -- 8M Zorro II RAM $20-9F goes to $80-$FF (SDRAM bank 1)

    -- Boolean logic can handle this mapping.  Furthermore, applying the same
    -- mapping to the other three banks is harmless, so there's no point expending logic
    -- to make it specific to the first bank.

    -- ABCD  B|C  A^(B|C)
    --
    -- 0000  0    0       0 -> 0
    --
    -- 0010  1    1       2 -> A
    -- 0100  1    1       4 -> C
    -- 0110  1    1       6 -> E
    -- 1000  0    1       8 -> 8
    --
    -- 1010  1    0       A -> 2
    -- 1100  1    0       C -> 4
    -- 1110  1    0       E -> 6 

    -- On 64-meg platforms we need an extra 32 meg merged into the memory map.
    -- If we configure that range second, it should end up in 42000000 - 43ffffff
    -- so the extra 2 or 4 meg will end up at either 41000000 or 4400000, depending
    -- on whether the extra 32 meg is configured.

    -- addr(25) will be high only when 32-meg block is active
    -- addr(24) will be high for the 16-meg block or the second half of the 32-meg block

    -- The extra ZIII mapping maps 41000000 -> 200000, (or 44000000 -> 200000)
    -- bits 23 downto 20 are mapped like so:
    -- 0000->0010 (1st 2 meg), 0010->0100 (2nd 2 meg),
    -- 0100->0010 (3rd 2 meg, aliases 1st), 0110->0100 (4th 2 meg, aliases 2nd), 
    -- addr(23) <= addr(23) and not sel_ziii_3;
    -- addr(22) <= (addr(22) and not sel_ziii_3) or (addr(21) and sel_ziii_3);
    -- addr(21) <= addr(21) xor sel_ziii_3;

    ramaddr(31 downto 26) <= "000000";
    ramaddr(25)           <= sel_z3ram2; -- Second block of 32 meg
    ramaddr(24)           <= (cpuaddr(24) and sel_z3ram2) or sel_z3ram; -- Remap the first block of Zorro III RAM to 0x1000000
    ramaddr(23)           <= (cpuaddr(23) xor (cpuaddr(22) or cpuaddr(21))) and not sel_z3ram3;
    ramaddr(22)           <= cpuaddr(21) when sel_z3ram3 = '1' else cpuaddr(22);
    ramaddr(21)           <= cpuaddr(21) xor sel_z3ram3;
    ramaddr(20 downto 0)  <= cpuaddr(20 downto 0);

    -- TODO: Replace with the WF68k030 core
    -- pf68K_Kernel_inst : entity work.TG68KdotC_Kernel
    --  GENERIC MAP(                    -- @suppress "Generic map uses default values. Missing optional actuals: BarrelShifter"
    --      SR_Read        => 2,        -- 0=>user,   1=>privileged,    2=>switchable with CPU(0)
    --      VBR_Stackframe => 2,        -- 0=>no,     1=>yes/extended,  2=>switchable with CPU(0)
    --      extAddr_Mode   => 2,        -- 0=>no,     1=>yes,           2=>switchable with CPU(1)
    --      MUL_Mode       => 2,        -- 0=>16Bit,  1=>32Bit,         2=>switchable with CPU(1),  3=>no MUL,
    --      DIV_Mode       => 2,        -- 0=>16Bit,  1=>32Bit,         2=>switchable with CPU(1),  3=>no DIV,
    --      BitField       => 2,        -- 0=>no,     1=>yes,           2=>switchable with CPU(1)
    --      MUL_Hardware   => 1         -- 0=>no,     1=>yes
    --  )
    --  PORT MAP(                       -- @suppress "The order of the associations is different from the declaration order"
    --      clk            => clk,      -- : in std_logic;
    --      nReset         => reset,    -- : in std_logic:='1';      --low active
    --      clkena_in      => clkena,   -- : in std_logic:='1';
    --      data_in        => datatg68, -- : in std_logic_vector(15 downto 0);
    --      IPL            => cpuIPL,   -- : in std_logic_vector(2 downto 0):="111"; Interrupt Priority Level
    --      addr_out       => addrtg68, -- : buffer std_logic_vector(31 downto 0);
    --      data_write     => w_datatg68, -- : out std_logic_vector(15 downto 0); Data OUT
    --      busstate       => state,    -- : buffer std_logic_vector(1 downto 0); -- 00-> fetch code 10->read data 11->write data 01->no memaccess
    --      longword       => longword, -- : long word active high
    --      nWr            => wr,       -- : out std_logic;
    --      nUDS           => uds_in,
    --      nLDS           => lds_in,   -- : out std_logic;
    --      nResetOut      => nResetOut_w,
    --      CACR_out       => CACR_out,  -- : Cache control register [3:0] = [clear cache, clear entry in cache, freeze cache, enable cache]
    --      VBR_out        => VBR_out_w, -- : Vector base register
    --  );

    -- Instanciate the WF68k030 core
    rw <= rw_d;
    WF68K30L_TOP_inst : entity work.WF68K30L_TOP
        GENERIC MAP(
            VERSION     => x"1904",     -- CPU version Number
            NO_PIPELINE => false,       -- If true the maincontroller workin scalarmode.
            NO_LOOP     => false        -- If true the DBcc loop mechanism is disabled.
        )
        PORT MAP(
            CLK       => clk,
            --Addressand data:
            ADR_OUT   => cpuaddr,
            DATA_IN   => datatg68,
            DATA_OUT  => w_datatg68,
            DATA_EN   => open,
            -- System control:
            BERRn     => '1',           -- In place of open, keep it inactivated
            RESET_INn => nReset,
            RESET_OUT => nResetOut_w,
            HALT_INn  => nReset,
            HALT_OUTn => open,
            -- Processor status:
            FC_OUT    => open,          -- Function code out, page 122 Address space encodings
            -- Interrupt control:
            AVECn     => '1',           -- autovector during an interrupt acknowledge cycle.
            IPLn      => IPL,
            IPENDn    => ipendn,        -- Interrupt pending
            -- Aynchronous bus control:
            DSACKn    => dsackn,        -- Asynchronous data transfer acknowledge
            SIZE      => cpu_siz,
            ASn       => as,            -- Valid address on bus
            RWn       => rw_d,
            RMCn      => open,          -- Indicate read modify write cycle
            DSn       => open,          -- Valid data on bus by cpu
            ECSn      => open,          -- External cycle start
            OCSn      => open,          -- First cycle of operand transfer start
            DBENn     => open,          -- Data buffer enable
            BUS_EN    => open,          -- not in datasheet
            -- Synchronous bus control:
            STERMn    => '1',           -- Synchronous termination (32-bit width only) (we use async)
            -- Status controls :
            STATUSn   => open,          -- Microsequencer status
            REFILLn   => open,          -- Start filling pipline
            -- Bus arbitration control:
            BRn       => '1',           -- Bus request
            BGn       => open,          -- Bus Grant
            BGACKn    => '1',           -- Bus Grant acknowledge
            -- Cache control:
            CAAR_OUT  => CAAR_OUT,
            CACR_OUT  => CACR_OUT,
            VBR_OUT   => VBR_OUT_w,
            OPCn      => OPCn
        );

    -- cpu address -> addr
    process(cpuaddr)
    begin
        addr <= cpuaddr;
    end process;

    -- generate cpu_int
    process(ipendn)
    begin
        cpu_int <= ipendn;
    end process;

    -- Create the state register from the cpu state
    -- state: 00-> fetch code 10->read data 11->write data 01->no memaccess
    -- rw = read high, write low
    -- ipendn = low when interrupt is pending
    statesel <= rw_d & OPCn;
    process(statesel)
    begin
        case (statesel) is
            when "00"   => state <= "01";
            when "01"   => state <= "11";
            when "10"   => state <= "00";
            when "11"   => state <= "10";
            when others => null;
        end case;
    end process;

    -- Translate SIZE register into uds,lds,uds2,lds2
    sizesel <= cpu_siz & cpuaddr(1 downto 0);
    process(sizesel)
    begin
        if nReset = '0' then
            sel <= (others => '0');
        else
            case sizesel is
                when "0000" => sel <= "11111";
                when "0001" => sel <= "10111";
                when "0010" => sel <= "00011";
                when "0011" => sel <= "00001";
                when "0100" => sel <= "11000";
                when "0101" => sel <= "10100";
                when "0110" => sel <= "00010";
                when "0111" => sel <= "00001";
                when "1000" => sel <= "11100";
                when "1001" => sel <= "10110";
                when "1010" => sel <= "00011";
                when "1011" => sel <= "00001";
                when "1100" => sel <= "11110";
                when "1101" => sel <= "10111";
                when "1110" => sel <= "00011";
                when "1111" => sel <= "00001";
                when others => null;
            end case;
        end if;
    end process;

    -- dtack to dsackn
    -- dsack hh-wait, hl-8bit, lh-16-bit, ll-32-bit
    dtacksel <= dtackn & sel_akiko_d;
    process(dtacksel)
    BEGIN
        case dtacksel is
            when "00"   => dsackn <= "00"; -- 32-bit wide
            when "01"   => dsackn <= "01"; -- Akiko selected, 16-bit
            when "10"   => dsackn <= "11"; -- dtack = 1, no ack wait
            -- when "11" => dsackn <= "11";
            when others => dsackn <= "11";
        end case;
    end process;

    -- Byte enable signals
    longword <= sel(4);
    uds      <= not sel(3);
    lds      <= not sel(2);
    uds2     <= not sel(1);
    lds2     <= not sel(0);

    -- Copy autoconfig state into module
    PROCESS(clk)
    BEGIN
        IF rising_edge(clk) THEN
            IF (nReset = '0' OR nResetOut_w = '0') THEN
                turbochip_d   <= '0';
                turbokick_d   <= '0';
                turboslow_d   <= '0';
                cacheline_clr <= '0';
                -- ELSIF state = "01" THEN     -- No mem access, so safe to switch chipram access mode
                --     turbochip_d   <= turbochipram;
                --     turbokick_d   <= turbokick;
                --     turboslow_d   <= turbochipram OR aga;
                --     cacheline_clr <= (turbochipram XOR turbochip_d);
            END IF;
            sel_ram_d <= sel_ram;
        END IF;
    END PROCESS;

    -- Akiko 'chunky pixel' implementation --
    host_req <= host_req_r;
    -- myakiko : entity work.akiko
    --     GENERIC MAP(
    --         havertg   => havertg,
    --         haveaudio => haveaudio,
    --         havec2p   => havec2p
    --     )
    --     PORT MAP(                       -- @suppress "The order of the associations is different from the declaration order"
    --         clk            => clk,
    --         reset_n        => reset,
    --         addr           => cpuaddr(10 downto 0),
    --         d              => akiko_d,
    --         q              => akiko_q,
    --         wr             => akiko_wr,
    --         req            => akiko_req,
    --         ack            => akiko_ack,
    --         host_req       => host_req_r,
    --         host_ack       => host_ack,
    --         host_q         => host_q,
    --         rtg_addr       => rtg_addr,
    --         rtg_vbend      => rtg_vbend,
    --         rtg_ext        => rtg_ext,
    --         rtg_pixelclock => rtg_pixelclock,
    --         rtg_16bit      => rtg_16bit,
    --         rtg_clut       => rtg_clut,
    --         rtg_clut_idx   => rtg_clut_idx,
    --         rtg_clut_r     => rtg_clut_r,
    --         rtg_clut_g     => rtg_clut_g,
    --         rtg_clut_b     => rtg_clut_b,
    --         audio_buf      => audio_buf,
    --         audio_ena      => audio_ena,
    --         audio_int      => audio_int
    --     );

    -- TODO: Enable Akiko later 
    -- akiko_d <= w_datatg68;
    -- PROCESS(clk)
    -- BEGIN
    --     if rising_edge(clk) then
    --         if sel_akiko = '0' then
    --             akiko_req <= '0';
    --             akiko_wr  <= '0';
    --         end if;
    --         if sel_akiko = '1' and state(1) = '1' and slower(2) = '0' then
    --             akiko_req <= not clkena;
    --             if state(0) = '1' then  -- write cycle
    --                 akiko_wr <= '1';
    --             end if;
    --         end if;
    --     end if;
    -- END PROCESS;

    -- Synchronize bus to write enable from ena7WRreg signal
    PROCESS(clk)
    BEGIN
        IF rising_edge(clk) THEN

            IF ena7WRreg = '1' THEN
                -- eind  <= ein;
                -- eindd <= eind;
                CASE sync_state IS
                    WHEN sync0  => sync_state <= sync1;
                    WHEN sync1  => sync_state <= sync2;
                    WHEN sync2  => sync_state <= sync3;
                    WHEN sync3  => sync_state <= sync4;
                    -- vma        <= vpa;
                    WHEN sync4  => sync_state <= sync5;
                    WHEN sync5  => sync_state <= sync6;
                    WHEN sync6  => sync_state <= sync7;
                    WHEN sync7  => sync_state <= sync8;
                    WHEN sync8  => sync_state <= sync9;
                    WHEN OTHERS => sync_state <= sync0;
                        -- vma        <= '1';
                END CASE;
                -- IF eind = '1' AND eindd = '0' THEN
                --     sync_state <= sync7;
                -- END IF;
            END IF;
        END IF;
    END PROCESS;

    clkena <= '1' WHEN (clkena_in = '1' AND (state = "01" OR (ena7RDreg = '1' AND clkena_e = '1') OR (ena7WRreg = '1' AND clkena_f = '1') OR ramready = '1' OR sel_undecoded_d = '1' OR akiko_ack = '1')) ELSE
              '0';

    -- Generate the slower bus signal
    PROCESS(clk)
    BEGIN
        IF rising_edge(clk) THEN
            IF clkena = '1' THEN
                slower <= "0111";       -- rokk
            ELSE
                slower(3 downto 0) <= '0' & slower(3 downto 1); -- enaWRreg&slower(3 downto 1);
            END IF;
        END IF;
    END PROCESS;

    -- chipset_cycle <= '1' when (sel_ram = '0' OR sel_nmi_vector = '1') AND sel_akiko = '0' and sel_undecoded = '0' else '0';

    -- Translate CPU output to RAM
    -- TG68K core to generic 68000 
    -- PROCESS(clk)
    -- BEGIN
    --     IF rising_edge(clk) THEN
    --         IF reset = '0' THEN
    --             S_state  <= "00";
    --             as       <= '1';
    --             rw       <= '1';
    --             uds      <= '1';
    --             lds      <= '1';
    --             uds2     <= '1';
    --             lds2     <= '1';
    --             clkena_e <= '0';
    --             clkena_f <= '0';
    --         ELSE
    --             IF S_state = "01" AND clkena_e = '1' THEN
    --                 uds2        <= uds_in;
    --                 lds2        <= lds_in;
    --                 data_write2 <= w_datatg68;
    --             END IF;

    --             -- CPU write cycle
    --             IF ena7WRreg = '1' THEN
    --                 CASE S_state IS
    --                     WHEN "00" =>
    --                         IF cpu_int = '0' AND chipset_cycle = '1' THEN
    --                             uds        <= uds_in;
    --                             lds        <= lds_in;
    --                             uds2       <= '1';
    --                             lds2       <= '1';
    --                             as         <= '0';
    --                             rw         <= wr;
    --                             data_write <= w_datatg68;
    --                             addr       <= cpuaddr;
    --                             IF aga = '1' AND longword = '1' AND state = "11" AND cpuaddr(1 downto 0) = "00" AND sel_chip = '1' THEN
    --                                 -- 32 bit write
    --                                 clkena_e <= '1';
    --                             END IF;
    --                             S_state <= "01";
    --                         END IF;
    --                     WHEN "01" =>
    --                         clkena_e <= '0';
    --                         S_state  <= "10";
    --                     WHEN "10" =>
    --                         IF waitm = '0' OR (vma = '0' AND sync_state = sync9) THEN
    --                             S_state <= "11";
    --                         END IF;
    --                     WHEN "11" =>
    --                         IF clkena_f = '1' THEN
    --                             clkena_f <= '0';
    --                             r_data   <= data_read2;
    --                         END IF;
    --                     WHEN OTHERS => null;
    --                 END CASE;
    --             -- CPU read cycle
    --             ELSIF ena7RDreg = '1' THEN
    --                 clkena_f <= '0';
    --                 CASE S_state IS
    --                     WHEN "00" =>
    --                         cpuIPL <= IPL;
    --                     WHEN "01" =>
    --                     WHEN "10" =>
    --                         cpuIPL <= IPL;
    --                         waitm  <= dtack;
    --                     WHEN "11" =>
    --                         as   <= '1';
    --                         rw   <= '1';
    --                         uds  <= '1';
    --                         lds  <= '1';
    --                         uds2 <= '1';
    --                         lds2 <= '1';
    --                         IF clkena_e = '0' THEN
    --                             r_data <= data_read;
    --                         END IF;

    --                         clkena_e <= '1';
    --                         IF aga = '1' AND longword = '1' AND state(0) = '0' AND cpuaddr(1 downto 0) = "00" AND (sel_chip = '1' OR sel_kick = '1') THEN
    --                             -- 32 bit read
    --                             clkena_f <= '1';
    --                         END IF;
    --                         IF clkena = '1' THEN
    --                             S_state  <= "00";
    --                             clkena_e <= '0';
    --                         END IF;
    --                     WHEN OTHERS => null;
    --                 END CASE;
    --             END IF;
    --         END IF;
    --     END IF;
    -- END PROCESS;

END;
