-- Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2018.3.1 (win64) Build 2489853 Tue Mar 26 04:20:25 MDT 2019
-- Date        : Tue Mar 10 14:04:57 2026
-- Host        : DESKTOP-CDKE44D running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode funcsim
--               C:/Users/khanm/AppData/Roaming/Xilinx/Vivado/proj/DDR_GOLDEN/DDR_GOLDEN.srcs/sources_1/bd/design_1/ip/design_1_bram_test_0_0/design_1_bram_test_0_0_sim_netlist.vhdl
-- Design      : design_1_bram_test_0_0
-- Purpose     : This VHDL netlist is a functional simulation representation of the design and should not be modified or
--               synthesized. This netlist cannot be used for SDF annotated simulation.
-- Device      : xc7a100tcsg324-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity design_1_bram_test_0_0_bram_test is
  port (
    clk : in STD_LOGIC;
    reset_n : in STD_LOGIC;
    bram_en : out STD_LOGIC;
    bram_addr : out STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_din : out STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_we : out STD_LOGIC_VECTOR ( 3 downto 0 );
    bram_dout : in STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_ready : in STD_LOGIC;
    tester : in STD_LOGIC;
    test_pass : out STD_LOGIC
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of design_1_bram_test_0_0_bram_test : entity is "bram_test";
end design_1_bram_test_0_0_bram_test;

architecture STRUCTURE of design_1_bram_test_0_0_bram_test is
  signal \<const0>\ : STD_LOGIC;
  signal \^bram_din\ : STD_LOGIC_VECTOR ( 31 to 31 );
  signal bram_en_i_1_n_0 : STD_LOGIC;
  signal \^test_pass\ : STD_LOGIC;
  attribute KEEP : string;
  attribute KEEP of \bram_addr_reg[0]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[10]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[11]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[12]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[13]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[14]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[15]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[16]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[17]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[18]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[19]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[1]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[20]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[21]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[22]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[23]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[24]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[25]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[26]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[27]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[28]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[29]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[2]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[30]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[31]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[3]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[4]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[5]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[6]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[7]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[8]\ : label is "yes";
  attribute KEEP of \bram_addr_reg[9]\ : label is "yes";
  attribute mark_debug : string;
  attribute mark_debug of bram_addr : signal is "true";
begin
  bram_din(31) <= \^bram_din\(31);
  bram_din(30) <= \^bram_din\(31);
  bram_din(29) <= \<const0>\;
  bram_din(28) <= \^bram_din\(31);
  bram_din(27) <= \^bram_din\(31);
  bram_din(26) <= \^bram_din\(31);
  bram_din(25) <= \^bram_din\(31);
  bram_din(24) <= \<const0>\;
  bram_din(23) <= \^bram_din\(31);
  bram_din(22) <= \<const0>\;
  bram_din(21) <= \^bram_din\(31);
  bram_din(20) <= \<const0>\;
  bram_din(19) <= \^bram_din\(31);
  bram_din(18) <= \^bram_din\(31);
  bram_din(17) <= \<const0>\;
  bram_din(16) <= \^bram_din\(31);
  bram_din(15) <= \^bram_din\(31);
  bram_din(14) <= \<const0>\;
  bram_din(13) <= \^bram_din\(31);
  bram_din(12) <= \^bram_din\(31);
  bram_din(11) <= \^bram_din\(31);
  bram_din(10) <= \^bram_din\(31);
  bram_din(9) <= \^bram_din\(31);
  bram_din(8) <= \<const0>\;
  bram_din(7) <= \^bram_din\(31);
  bram_din(6) <= \^bram_din\(31);
  bram_din(5) <= \^bram_din\(31);
  bram_din(4) <= \<const0>\;
  bram_din(3) <= \^bram_din\(31);
  bram_din(2) <= \^bram_din\(31);
  bram_din(1) <= \^bram_din\(31);
  bram_din(0) <= \^bram_din\(31);
  bram_en <= \^bram_din\(31);
  bram_we(3) <= \^bram_din\(31);
  bram_we(2) <= \^bram_din\(31);
  bram_we(1) <= \^bram_din\(31);
  bram_we(0) <= \^bram_din\(31);
  test_pass <= \^test_pass\;
GND: unisim.vcomponents.GND
     port map (
      G => \<const0>\
    );
\bram_addr_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(0)
    );
\bram_addr_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(10)
    );
\bram_addr_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(11)
    );
\bram_addr_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(12)
    );
\bram_addr_reg[13]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(13)
    );
\bram_addr_reg[14]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(14)
    );
\bram_addr_reg[15]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(15)
    );
\bram_addr_reg[16]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(16)
    );
\bram_addr_reg[17]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(17)
    );
\bram_addr_reg[18]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(18)
    );
\bram_addr_reg[19]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(19)
    );
\bram_addr_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(1)
    );
\bram_addr_reg[20]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(20)
    );
\bram_addr_reg[21]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(21)
    );
\bram_addr_reg[22]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(22)
    );
\bram_addr_reg[23]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(23)
    );
\bram_addr_reg[24]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(24)
    );
\bram_addr_reg[25]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(25)
    );
\bram_addr_reg[26]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(26)
    );
\bram_addr_reg[27]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(27)
    );
\bram_addr_reg[28]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(28)
    );
\bram_addr_reg[29]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(29)
    );
\bram_addr_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(2)
    );
\bram_addr_reg[30]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => \^test_pass\,
      Q => bram_addr(30)
    );
\bram_addr_reg[31]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => \^test_pass\,
      Q => bram_addr(31)
    );
\bram_addr_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(3)
    );
\bram_addr_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(4)
    );
\bram_addr_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(5)
    );
\bram_addr_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(6)
    );
\bram_addr_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(7)
    );
\bram_addr_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(8)
    );
\bram_addr_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => '0',
      Q => bram_addr(9)
    );
bram_en_i_1: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => reset_n,
      O => bram_en_i_1_n_0
    );
bram_en_reg: unisim.vcomponents.FDPE
     port map (
      C => clk,
      CE => '1',
      D => \^test_pass\,
      PRE => bram_en_i_1_n_0,
      Q => \^bram_din\(31)
    );
tester_latch_reg: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => bram_en_i_1_n_0,
      D => tester,
      Q => \^test_pass\
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity design_1_bram_test_0_0 is
  port (
    clk : in STD_LOGIC;
    reset_n : in STD_LOGIC;
    bram_en : out STD_LOGIC;
    bram_addr : out STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_din : out STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_we : out STD_LOGIC_VECTOR ( 3 downto 0 );
    bram_dout : in STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_ready : in STD_LOGIC;
    tester : in STD_LOGIC;
    test_pass : out STD_LOGIC
  );
  attribute NotValidForBitStream : boolean;
  attribute NotValidForBitStream of design_1_bram_test_0_0 : entity is true;
  attribute CHECK_LICENSE_TYPE : string;
  attribute CHECK_LICENSE_TYPE of design_1_bram_test_0_0 : entity is "design_1_bram_test_0_0,bram_test,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of design_1_bram_test_0_0 : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of design_1_bram_test_0_0 : entity is "module_ref";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of design_1_bram_test_0_0 : entity is "bram_test,Vivado 2018.3.1";
end design_1_bram_test_0_0;

architecture STRUCTURE of design_1_bram_test_0_0 is
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_INFO of clk : signal is "xilinx.com:signal:clock:1.0 clk CLK";
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_PARAMETER of clk : signal is "XIL_INTERFACENAME clk, FREQ_HZ 81247969, PHASE 0, CLK_DOMAIN design_1_mig_7series_0_0_ui_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of reset_n : signal is "xilinx.com:signal:reset:1.0 reset_n RST";
  attribute X_INTERFACE_PARAMETER of reset_n : signal is "XIL_INTERFACENAME reset_n, POLARITY ACTIVE_LOW, INSERT_VIP 0";
begin
inst: entity work.design_1_bram_test_0_0_bram_test
     port map (
      bram_addr(31 downto 0) => bram_addr(31 downto 0),
      bram_din(31 downto 0) => bram_din(31 downto 0),
      bram_dout(31 downto 0) => bram_dout(31 downto 0),
      bram_en => bram_en,
      bram_ready => bram_ready,
      bram_we(3 downto 0) => bram_we(3 downto 0),
      clk => clk,
      reset_n => reset_n,
      test_pass => test_pass,
      tester => tester
    );
end STRUCTURE;
