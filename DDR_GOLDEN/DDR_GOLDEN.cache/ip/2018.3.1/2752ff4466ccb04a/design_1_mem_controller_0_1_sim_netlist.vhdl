-- Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2018.3.1 (win64) Build 2489853 Tue Mar 26 04:20:25 MDT 2019
-- Date        : Tue Mar 10 22:45:20 2026
-- Host        : DESKTOP-CDKE44D running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
--               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ design_1_mem_controller_0_1_sim_netlist.vhdl
-- Design      : design_1_mem_controller_0_1
-- Purpose     : This VHDL netlist is a functional simulation representation of the design and should not be modified or
--               synthesized. This netlist cannot be used for SDF annotated simulation.
-- Device      : xc7a100tcsg324-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master is
  port (
    bram_en : in STD_LOGIC;
    bram_addr : in STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_din : in STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_we : in STD_LOGIC_VECTOR ( 3 downto 0 );
    bram_dout : out STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_ready : out STD_LOGIC;
    aclk : in STD_LOGIC;
    aresetn : in STD_LOGIC;
    M_AXI_AWADDR : out STD_LOGIC_VECTOR ( 31 downto 0 );
    M_AXI_AWVALID : out STD_LOGIC;
    M_AXI_AWREADY : in STD_LOGIC;
    M_AXI_WDATA : out STD_LOGIC_VECTOR ( 31 downto 0 );
    M_AXI_WSTRB : out STD_LOGIC_VECTOR ( 3 downto 0 );
    M_AXI_WVALID : out STD_LOGIC;
    M_AXI_WREADY : in STD_LOGIC;
    M_AXI_BRESP : in STD_LOGIC_VECTOR ( 1 downto 0 );
    M_AXI_BVALID : in STD_LOGIC;
    M_AXI_BREADY : out STD_LOGIC;
    value_out : in STD_LOGIC_VECTOR ( 31 downto 0 );
    init_write : in STD_LOGIC;
    addr_in : in STD_LOGIC_VECTOR ( 31 downto 0 );
    state_led : out STD_LOGIC_VECTOR ( 1 downto 0 )
  );
  attribute ST_AWBRAM : string;
  attribute ST_AWBRAM of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master : entity is "2'b11";
  attribute ST_AW_W : string;
  attribute ST_AW_W of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master : entity is "2'b00";
  attribute ST_B : string;
  attribute ST_B of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master : entity is "2'b01";
  attribute ST_DONE : string;
  attribute ST_DONE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master : entity is "2'b10";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master is
  signal \<const0>\ : STD_LOGIC;
  signal \<const1>\ : STD_LOGIC;
  signal \FSM_sequential_state[0]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state[1]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_AWADDR[31]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_AWADDR[31]_i_2_n_0\ : STD_LOGIC;
  signal \^m_axi_awvalid\ : STD_LOGIC;
  signal M_AXI_AWVALID_i_1_n_0 : STD_LOGIC;
  signal M_AXI_BREADY0 : STD_LOGIC;
  signal \M_AXI_WDATA[0]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[10]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[11]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[12]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[13]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[14]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[15]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[16]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[17]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[18]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[19]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[1]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[20]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[21]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[22]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[23]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[24]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[25]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[26]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[27]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[28]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[29]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[2]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[30]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[31]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[3]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[4]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[5]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[6]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[7]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[8]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WDATA[9]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WSTRB[0]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WSTRB[1]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WSTRB[2]_i_1_n_0\ : STD_LOGIC;
  signal \M_AXI_WSTRB[3]_i_1_n_0\ : STD_LOGIC;
  signal \^m_axi_wvalid\ : STD_LOGIC;
  signal M_AXI_WVALID_i_1_n_0 : STD_LOGIC;
  signal \^bram_ready\ : STD_LOGIC;
  signal bram_ready_i_1_n_0 : STD_LOGIC;
  signal \bram_we_latched[3]_i_1_n_0\ : STD_LOGIC;
  signal \bram_we_latched_reg_n_0_[0]\ : STD_LOGIC;
  signal \bram_we_latched_reg_n_0_[1]\ : STD_LOGIC;
  signal \bram_we_latched_reg_n_0_[2]\ : STD_LOGIC;
  signal \bram_we_latched_reg_n_0_[3]\ : STD_LOGIC;
  signal latch_addr : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal \latch_addr_reg_n_0_[0]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[10]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[11]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[12]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[13]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[14]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[15]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[16]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[17]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[18]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[19]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[1]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[20]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[21]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[22]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[23]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[24]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[25]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[26]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[27]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[28]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[29]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[2]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[30]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[31]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[3]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[4]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[5]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[6]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[7]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[8]\ : STD_LOGIC;
  signal \latch_addr_reg_n_0_[9]\ : STD_LOGIC;
  signal state : STD_LOGIC_VECTOR ( 1 downto 0 );
  signal \state_led[1]_i_1_n_0\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \FSM_sequential_state[0]_i_1\ : label is "soft_lutpair0";
  attribute FSM_ENCODED_STATES : string;
  attribute FSM_ENCODED_STATES of \FSM_sequential_state_reg[0]\ : label is "ST_B:11,ST_DONE:00,ST_AW_W:10,ST_AWBRAM:01";
  attribute FSM_ENCODED_STATES of \FSM_sequential_state_reg[1]\ : label is "ST_B:11,ST_DONE:00,ST_AW_W:10,ST_AWBRAM:01";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[0]_i_1\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[10]_i_1\ : label is "soft_lutpair23";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[11]_i_1\ : label is "soft_lutpair24";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[12]_i_1\ : label is "soft_lutpair24";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[13]_i_1\ : label is "soft_lutpair25";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[14]_i_1\ : label is "soft_lutpair25";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[15]_i_1\ : label is "soft_lutpair26";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[16]_i_1\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[17]_i_1\ : label is "soft_lutpair19";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[18]_i_1\ : label is "soft_lutpair20";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[19]_i_1\ : label is "soft_lutpair26";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[1]_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[20]_i_1\ : label is "soft_lutpair27";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[21]_i_1\ : label is "soft_lutpair28";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[22]_i_1\ : label is "soft_lutpair29";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[23]_i_1\ : label is "soft_lutpair29";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[24]_i_1\ : label is "soft_lutpair28";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[25]_i_1\ : label is "soft_lutpair30";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[26]_i_1\ : label is "soft_lutpair27";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[27]_i_1\ : label is "soft_lutpair30";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[28]_i_1\ : label is "soft_lutpair31";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[29]_i_1\ : label is "soft_lutpair31";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[2]_i_1\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[30]_i_1\ : label is "soft_lutpair32";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[31]_i_1\ : label is "soft_lutpair32";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[3]_i_1\ : label is "soft_lutpair19";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[4]_i_1\ : label is "soft_lutpair20";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[5]_i_1\ : label is "soft_lutpair21";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[6]_i_1\ : label is "soft_lutpair21";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[7]_i_1\ : label is "soft_lutpair22";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[8]_i_1\ : label is "soft_lutpair22";
  attribute SOFT_HLUTNM of \M_AXI_WDATA[9]_i_1\ : label is "soft_lutpair23";
  attribute SOFT_HLUTNM of \M_AXI_WSTRB[0]_i_1\ : label is "soft_lutpair33";
  attribute SOFT_HLUTNM of \M_AXI_WSTRB[1]_i_1\ : label is "soft_lutpair34";
  attribute SOFT_HLUTNM of \M_AXI_WSTRB[2]_i_1\ : label is "soft_lutpair33";
  attribute SOFT_HLUTNM of \M_AXI_WSTRB[3]_i_1\ : label is "soft_lutpair34";
  attribute SOFT_HLUTNM of \latch_addr[0]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \latch_addr[10]_i_1\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \latch_addr[11]_i_1\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \latch_addr[12]_i_1\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \latch_addr[13]_i_1\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \latch_addr[14]_i_1\ : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of \latch_addr[15]_i_1\ : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of \latch_addr[16]_i_1\ : label is "soft_lutpair10";
  attribute SOFT_HLUTNM of \latch_addr[17]_i_1\ : label is "soft_lutpair10";
  attribute SOFT_HLUTNM of \latch_addr[18]_i_1\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \latch_addr[19]_i_1\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \latch_addr[1]_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \latch_addr[20]_i_1\ : label is "soft_lutpair12";
  attribute SOFT_HLUTNM of \latch_addr[21]_i_1\ : label is "soft_lutpair12";
  attribute SOFT_HLUTNM of \latch_addr[22]_i_1\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \latch_addr[23]_i_1\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \latch_addr[24]_i_1\ : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of \latch_addr[25]_i_1\ : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of \latch_addr[26]_i_1\ : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of \latch_addr[27]_i_1\ : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of \latch_addr[28]_i_1\ : label is "soft_lutpair16";
  attribute SOFT_HLUTNM of \latch_addr[29]_i_1\ : label is "soft_lutpair16";
  attribute SOFT_HLUTNM of \latch_addr[2]_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \latch_addr[30]_i_1\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \latch_addr[31]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \latch_addr[3]_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \latch_addr[4]_i_1\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \latch_addr[5]_i_1\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \latch_addr[6]_i_1\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \latch_addr[7]_i_1\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \latch_addr[8]_i_1\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \latch_addr[9]_i_1\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \state_led[1]_i_1\ : label is "soft_lutpair0";
  attribute mark_debug : string;
  attribute mark_debug of M_AXI_BRESP : signal is "true";
begin
  M_AXI_AWVALID <= \^m_axi_awvalid\;
  M_AXI_BREADY <= \<const1>\;
  M_AXI_WVALID <= \^m_axi_wvalid\;
  bram_dout(31) <= \<const0>\;
  bram_dout(30) <= \<const0>\;
  bram_dout(29) <= \<const0>\;
  bram_dout(28) <= \<const0>\;
  bram_dout(27) <= \<const0>\;
  bram_dout(26) <= \<const0>\;
  bram_dout(25) <= \<const0>\;
  bram_dout(24) <= \<const0>\;
  bram_dout(23) <= \<const0>\;
  bram_dout(22) <= \<const0>\;
  bram_dout(21) <= \<const0>\;
  bram_dout(20) <= \<const0>\;
  bram_dout(19) <= \<const0>\;
  bram_dout(18) <= \<const0>\;
  bram_dout(17) <= \<const0>\;
  bram_dout(16) <= \<const0>\;
  bram_dout(15) <= \<const0>\;
  bram_dout(14) <= \<const0>\;
  bram_dout(13) <= \<const0>\;
  bram_dout(12) <= \<const0>\;
  bram_dout(11) <= \<const0>\;
  bram_dout(10) <= \<const0>\;
  bram_dout(9) <= \<const0>\;
  bram_dout(8) <= \<const0>\;
  bram_dout(7) <= \<const0>\;
  bram_dout(6) <= \<const0>\;
  bram_dout(5) <= \<const0>\;
  bram_dout(4) <= \<const0>\;
  bram_dout(3) <= \<const0>\;
  bram_dout(2) <= \<const0>\;
  bram_dout(1) <= \<const0>\;
  bram_dout(0) <= \<const0>\;
  bram_ready <= \^bram_ready\;
\FSM_sequential_state[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"5FFC5F0C"
    )
        port map (
      I0 => M_AXI_BVALID,
      I1 => bram_en,
      I2 => state(1),
      I3 => state(0),
      I4 => M_AXI_BREADY0,
      O => \FSM_sequential_state[0]_i_1_n_0\
    );
\FSM_sequential_state[1]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"55FFFF0C5500FF0C"
    )
        port map (
      I0 => M_AXI_BVALID,
      I1 => init_write,
      I2 => bram_en,
      I3 => state(1),
      I4 => state(0),
      I5 => M_AXI_BREADY0,
      O => \FSM_sequential_state[1]_i_1_n_0\
    );
\FSM_sequential_state[1]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8000"
    )
        port map (
      I0 => M_AXI_AWREADY,
      I1 => \^m_axi_awvalid\,
      I2 => M_AXI_WREADY,
      I3 => \^m_axi_wvalid\,
      O => M_AXI_BREADY0
    );
\FSM_sequential_state_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \FSM_sequential_state[0]_i_1_n_0\,
      Q => state(0)
    );
\FSM_sequential_state_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \FSM_sequential_state[1]_i_1_n_0\,
      Q => state(1)
    );
GND: unisim.vcomponents.GND
     port map (
      G => \<const0>\
    );
\M_AXI_AWADDR[31]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => state(0),
      I1 => state(1),
      O => \M_AXI_AWADDR[31]_i_1_n_0\
    );
\M_AXI_AWADDR[31]_i_2\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => aresetn,
      O => \M_AXI_AWADDR[31]_i_2_n_0\
    );
\M_AXI_AWADDR_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[0]\,
      Q => M_AXI_AWADDR(0)
    );
\M_AXI_AWADDR_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[10]\,
      Q => M_AXI_AWADDR(10)
    );
\M_AXI_AWADDR_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[11]\,
      Q => M_AXI_AWADDR(11)
    );
\M_AXI_AWADDR_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[12]\,
      Q => M_AXI_AWADDR(12)
    );
\M_AXI_AWADDR_reg[13]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[13]\,
      Q => M_AXI_AWADDR(13)
    );
\M_AXI_AWADDR_reg[14]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[14]\,
      Q => M_AXI_AWADDR(14)
    );
\M_AXI_AWADDR_reg[15]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[15]\,
      Q => M_AXI_AWADDR(15)
    );
\M_AXI_AWADDR_reg[16]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[16]\,
      Q => M_AXI_AWADDR(16)
    );
\M_AXI_AWADDR_reg[17]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[17]\,
      Q => M_AXI_AWADDR(17)
    );
\M_AXI_AWADDR_reg[18]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[18]\,
      Q => M_AXI_AWADDR(18)
    );
\M_AXI_AWADDR_reg[19]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[19]\,
      Q => M_AXI_AWADDR(19)
    );
\M_AXI_AWADDR_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[1]\,
      Q => M_AXI_AWADDR(1)
    );
\M_AXI_AWADDR_reg[20]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[20]\,
      Q => M_AXI_AWADDR(20)
    );
\M_AXI_AWADDR_reg[21]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[21]\,
      Q => M_AXI_AWADDR(21)
    );
\M_AXI_AWADDR_reg[22]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[22]\,
      Q => M_AXI_AWADDR(22)
    );
\M_AXI_AWADDR_reg[23]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[23]\,
      Q => M_AXI_AWADDR(23)
    );
\M_AXI_AWADDR_reg[24]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[24]\,
      Q => M_AXI_AWADDR(24)
    );
\M_AXI_AWADDR_reg[25]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[25]\,
      Q => M_AXI_AWADDR(25)
    );
\M_AXI_AWADDR_reg[26]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[26]\,
      Q => M_AXI_AWADDR(26)
    );
\M_AXI_AWADDR_reg[27]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[27]\,
      Q => M_AXI_AWADDR(27)
    );
\M_AXI_AWADDR_reg[28]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[28]\,
      Q => M_AXI_AWADDR(28)
    );
\M_AXI_AWADDR_reg[29]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[29]\,
      Q => M_AXI_AWADDR(29)
    );
\M_AXI_AWADDR_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[2]\,
      Q => M_AXI_AWADDR(2)
    );
\M_AXI_AWADDR_reg[30]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[30]\,
      Q => M_AXI_AWADDR(30)
    );
\M_AXI_AWADDR_reg[31]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[31]\,
      Q => M_AXI_AWADDR(31)
    );
\M_AXI_AWADDR_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[3]\,
      Q => M_AXI_AWADDR(3)
    );
\M_AXI_AWADDR_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[4]\,
      Q => M_AXI_AWADDR(4)
    );
\M_AXI_AWADDR_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[5]\,
      Q => M_AXI_AWADDR(5)
    );
\M_AXI_AWADDR_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[6]\,
      Q => M_AXI_AWADDR(6)
    );
\M_AXI_AWADDR_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[7]\,
      Q => M_AXI_AWADDR(7)
    );
\M_AXI_AWADDR_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[8]\,
      Q => M_AXI_AWADDR(8)
    );
\M_AXI_AWADDR_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \latch_addr_reg_n_0_[9]\,
      Q => M_AXI_AWADDR(9)
    );
M_AXI_AWVALID_i_1: unisim.vcomponents.LUT4
    generic map(
      INIT => X"A77A"
    )
        port map (
      I0 => \^m_axi_awvalid\,
      I1 => M_AXI_AWREADY,
      I2 => state(1),
      I3 => state(0),
      O => M_AXI_AWVALID_i_1_n_0
    );
M_AXI_AWVALID_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => M_AXI_AWVALID_i_1_n_0,
      Q => \^m_axi_awvalid\
    );
\M_AXI_WDATA[0]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(0),
      I1 => bram_din(0),
      I2 => state(1),
      O => \M_AXI_WDATA[0]_i_1_n_0\
    );
\M_AXI_WDATA[10]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(10),
      I1 => bram_din(10),
      I2 => state(1),
      O => \M_AXI_WDATA[10]_i_1_n_0\
    );
\M_AXI_WDATA[11]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(11),
      I1 => bram_din(11),
      I2 => state(1),
      O => \M_AXI_WDATA[11]_i_1_n_0\
    );
\M_AXI_WDATA[12]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(12),
      I1 => bram_din(12),
      I2 => state(1),
      O => \M_AXI_WDATA[12]_i_1_n_0\
    );
\M_AXI_WDATA[13]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(13),
      I1 => bram_din(13),
      I2 => state(1),
      O => \M_AXI_WDATA[13]_i_1_n_0\
    );
\M_AXI_WDATA[14]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(14),
      I1 => bram_din(14),
      I2 => state(1),
      O => \M_AXI_WDATA[14]_i_1_n_0\
    );
\M_AXI_WDATA[15]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(15),
      I1 => bram_din(15),
      I2 => state(1),
      O => \M_AXI_WDATA[15]_i_1_n_0\
    );
\M_AXI_WDATA[16]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(16),
      I1 => bram_din(16),
      I2 => state(1),
      O => \M_AXI_WDATA[16]_i_1_n_0\
    );
\M_AXI_WDATA[17]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(17),
      I1 => bram_din(17),
      I2 => state(1),
      O => \M_AXI_WDATA[17]_i_1_n_0\
    );
\M_AXI_WDATA[18]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(18),
      I1 => bram_din(18),
      I2 => state(1),
      O => \M_AXI_WDATA[18]_i_1_n_0\
    );
\M_AXI_WDATA[19]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(19),
      I1 => bram_din(19),
      I2 => state(1),
      O => \M_AXI_WDATA[19]_i_1_n_0\
    );
\M_AXI_WDATA[1]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(1),
      I1 => bram_din(1),
      I2 => state(1),
      O => \M_AXI_WDATA[1]_i_1_n_0\
    );
\M_AXI_WDATA[20]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(20),
      I1 => bram_din(20),
      I2 => state(1),
      O => \M_AXI_WDATA[20]_i_1_n_0\
    );
\M_AXI_WDATA[21]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(21),
      I1 => bram_din(21),
      I2 => state(1),
      O => \M_AXI_WDATA[21]_i_1_n_0\
    );
\M_AXI_WDATA[22]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(22),
      I1 => bram_din(22),
      I2 => state(1),
      O => \M_AXI_WDATA[22]_i_1_n_0\
    );
\M_AXI_WDATA[23]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(23),
      I1 => bram_din(23),
      I2 => state(1),
      O => \M_AXI_WDATA[23]_i_1_n_0\
    );
\M_AXI_WDATA[24]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(24),
      I1 => bram_din(24),
      I2 => state(1),
      O => \M_AXI_WDATA[24]_i_1_n_0\
    );
\M_AXI_WDATA[25]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(25),
      I1 => bram_din(25),
      I2 => state(1),
      O => \M_AXI_WDATA[25]_i_1_n_0\
    );
\M_AXI_WDATA[26]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(26),
      I1 => bram_din(26),
      I2 => state(1),
      O => \M_AXI_WDATA[26]_i_1_n_0\
    );
\M_AXI_WDATA[27]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(27),
      I1 => bram_din(27),
      I2 => state(1),
      O => \M_AXI_WDATA[27]_i_1_n_0\
    );
\M_AXI_WDATA[28]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(28),
      I1 => bram_din(28),
      I2 => state(1),
      O => \M_AXI_WDATA[28]_i_1_n_0\
    );
\M_AXI_WDATA[29]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(29),
      I1 => bram_din(29),
      I2 => state(1),
      O => \M_AXI_WDATA[29]_i_1_n_0\
    );
\M_AXI_WDATA[2]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(2),
      I1 => bram_din(2),
      I2 => state(1),
      O => \M_AXI_WDATA[2]_i_1_n_0\
    );
\M_AXI_WDATA[30]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(30),
      I1 => bram_din(30),
      I2 => state(1),
      O => \M_AXI_WDATA[30]_i_1_n_0\
    );
\M_AXI_WDATA[31]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(31),
      I1 => bram_din(31),
      I2 => state(1),
      O => \M_AXI_WDATA[31]_i_1_n_0\
    );
\M_AXI_WDATA[3]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(3),
      I1 => bram_din(3),
      I2 => state(1),
      O => \M_AXI_WDATA[3]_i_1_n_0\
    );
\M_AXI_WDATA[4]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(4),
      I1 => bram_din(4),
      I2 => state(1),
      O => \M_AXI_WDATA[4]_i_1_n_0\
    );
\M_AXI_WDATA[5]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(5),
      I1 => bram_din(5),
      I2 => state(1),
      O => \M_AXI_WDATA[5]_i_1_n_0\
    );
\M_AXI_WDATA[6]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(6),
      I1 => bram_din(6),
      I2 => state(1),
      O => \M_AXI_WDATA[6]_i_1_n_0\
    );
\M_AXI_WDATA[7]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(7),
      I1 => bram_din(7),
      I2 => state(1),
      O => \M_AXI_WDATA[7]_i_1_n_0\
    );
\M_AXI_WDATA[8]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(8),
      I1 => bram_din(8),
      I2 => state(1),
      O => \M_AXI_WDATA[8]_i_1_n_0\
    );
\M_AXI_WDATA[9]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => value_out(9),
      I1 => bram_din(9),
      I2 => state(1),
      O => \M_AXI_WDATA[9]_i_1_n_0\
    );
\M_AXI_WDATA_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[0]_i_1_n_0\,
      Q => M_AXI_WDATA(0)
    );
\M_AXI_WDATA_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[10]_i_1_n_0\,
      Q => M_AXI_WDATA(10)
    );
\M_AXI_WDATA_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[11]_i_1_n_0\,
      Q => M_AXI_WDATA(11)
    );
\M_AXI_WDATA_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[12]_i_1_n_0\,
      Q => M_AXI_WDATA(12)
    );
\M_AXI_WDATA_reg[13]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[13]_i_1_n_0\,
      Q => M_AXI_WDATA(13)
    );
\M_AXI_WDATA_reg[14]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[14]_i_1_n_0\,
      Q => M_AXI_WDATA(14)
    );
\M_AXI_WDATA_reg[15]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[15]_i_1_n_0\,
      Q => M_AXI_WDATA(15)
    );
\M_AXI_WDATA_reg[16]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[16]_i_1_n_0\,
      Q => M_AXI_WDATA(16)
    );
\M_AXI_WDATA_reg[17]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[17]_i_1_n_0\,
      Q => M_AXI_WDATA(17)
    );
\M_AXI_WDATA_reg[18]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[18]_i_1_n_0\,
      Q => M_AXI_WDATA(18)
    );
\M_AXI_WDATA_reg[19]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[19]_i_1_n_0\,
      Q => M_AXI_WDATA(19)
    );
\M_AXI_WDATA_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[1]_i_1_n_0\,
      Q => M_AXI_WDATA(1)
    );
\M_AXI_WDATA_reg[20]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[20]_i_1_n_0\,
      Q => M_AXI_WDATA(20)
    );
\M_AXI_WDATA_reg[21]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[21]_i_1_n_0\,
      Q => M_AXI_WDATA(21)
    );
\M_AXI_WDATA_reg[22]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[22]_i_1_n_0\,
      Q => M_AXI_WDATA(22)
    );
\M_AXI_WDATA_reg[23]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[23]_i_1_n_0\,
      Q => M_AXI_WDATA(23)
    );
\M_AXI_WDATA_reg[24]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[24]_i_1_n_0\,
      Q => M_AXI_WDATA(24)
    );
\M_AXI_WDATA_reg[25]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[25]_i_1_n_0\,
      Q => M_AXI_WDATA(25)
    );
\M_AXI_WDATA_reg[26]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[26]_i_1_n_0\,
      Q => M_AXI_WDATA(26)
    );
\M_AXI_WDATA_reg[27]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[27]_i_1_n_0\,
      Q => M_AXI_WDATA(27)
    );
\M_AXI_WDATA_reg[28]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[28]_i_1_n_0\,
      Q => M_AXI_WDATA(28)
    );
\M_AXI_WDATA_reg[29]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[29]_i_1_n_0\,
      Q => M_AXI_WDATA(29)
    );
\M_AXI_WDATA_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[2]_i_1_n_0\,
      Q => M_AXI_WDATA(2)
    );
\M_AXI_WDATA_reg[30]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[30]_i_1_n_0\,
      Q => M_AXI_WDATA(30)
    );
\M_AXI_WDATA_reg[31]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[31]_i_1_n_0\,
      Q => M_AXI_WDATA(31)
    );
\M_AXI_WDATA_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[3]_i_1_n_0\,
      Q => M_AXI_WDATA(3)
    );
\M_AXI_WDATA_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[4]_i_1_n_0\,
      Q => M_AXI_WDATA(4)
    );
\M_AXI_WDATA_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[5]_i_1_n_0\,
      Q => M_AXI_WDATA(5)
    );
\M_AXI_WDATA_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[6]_i_1_n_0\,
      Q => M_AXI_WDATA(6)
    );
\M_AXI_WDATA_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[7]_i_1_n_0\,
      Q => M_AXI_WDATA(7)
    );
\M_AXI_WDATA_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[8]_i_1_n_0\,
      Q => M_AXI_WDATA(8)
    );
\M_AXI_WDATA_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \M_AXI_WDATA[9]_i_1_n_0\,
      Q => M_AXI_WDATA(9)
    );
\M_AXI_WSTRB[0]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => state(1),
      I1 => \bram_we_latched_reg_n_0_[0]\,
      O => \M_AXI_WSTRB[0]_i_1_n_0\
    );
\M_AXI_WSTRB[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => state(1),
      I1 => \bram_we_latched_reg_n_0_[1]\,
      O => \M_AXI_WSTRB[1]_i_1_n_0\
    );
\M_AXI_WSTRB[2]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => state(1),
      I1 => \bram_we_latched_reg_n_0_[2]\,
      O => \M_AXI_WSTRB[2]_i_1_n_0\
    );
\M_AXI_WSTRB[3]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => state(1),
      I1 => \bram_we_latched_reg_n_0_[3]\,
      O => \M_AXI_WSTRB[3]_i_1_n_0\
    );
\M_AXI_WSTRB_reg[0]\: unisim.vcomponents.FDPE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      D => \M_AXI_WSTRB[0]_i_1_n_0\,
      PRE => \M_AXI_AWADDR[31]_i_2_n_0\,
      Q => M_AXI_WSTRB(0)
    );
\M_AXI_WSTRB_reg[1]\: unisim.vcomponents.FDPE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      D => \M_AXI_WSTRB[1]_i_1_n_0\,
      PRE => \M_AXI_AWADDR[31]_i_2_n_0\,
      Q => M_AXI_WSTRB(1)
    );
\M_AXI_WSTRB_reg[2]\: unisim.vcomponents.FDPE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      D => \M_AXI_WSTRB[2]_i_1_n_0\,
      PRE => \M_AXI_AWADDR[31]_i_2_n_0\,
      Q => M_AXI_WSTRB(2)
    );
\M_AXI_WSTRB_reg[3]\: unisim.vcomponents.FDPE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      D => \M_AXI_WSTRB[3]_i_1_n_0\,
      PRE => \M_AXI_AWADDR[31]_i_2_n_0\,
      Q => M_AXI_WSTRB(3)
    );
M_AXI_WVALID_i_1: unisim.vcomponents.LUT4
    generic map(
      INIT => X"A77A"
    )
        port map (
      I0 => \^m_axi_wvalid\,
      I1 => M_AXI_WREADY,
      I2 => state(1),
      I3 => state(0),
      O => M_AXI_WVALID_i_1_n_0
    );
M_AXI_WVALID_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => M_AXI_WVALID_i_1_n_0,
      Q => \^m_axi_wvalid\
    );
VCC: unisim.vcomponents.VCC
     port map (
      P => \<const1>\
    );
bram_ready_i_1: unisim.vcomponents.LUT4
    generic map(
      INIT => X"F704"
    )
        port map (
      I0 => state(0),
      I1 => aresetn,
      I2 => state(1),
      I3 => \^bram_ready\,
      O => bram_ready_i_1_n_0
    );
bram_ready_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => bram_ready_i_1_n_0,
      Q => \^bram_ready\,
      R => '0'
    );
\bram_we_latched[3]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"20"
    )
        port map (
      I0 => aresetn,
      I1 => state(1),
      I2 => state(0),
      O => \bram_we_latched[3]_i_1_n_0\
    );
\bram_we_latched_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => \bram_we_latched[3]_i_1_n_0\,
      D => bram_we(0),
      Q => \bram_we_latched_reg_n_0_[0]\,
      R => '0'
    );
\bram_we_latched_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => \bram_we_latched[3]_i_1_n_0\,
      D => bram_we(1),
      Q => \bram_we_latched_reg_n_0_[1]\,
      R => '0'
    );
\bram_we_latched_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => \bram_we_latched[3]_i_1_n_0\,
      D => bram_we(2),
      Q => \bram_we_latched_reg_n_0_[2]\,
      R => '0'
    );
\bram_we_latched_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => \bram_we_latched[3]_i_1_n_0\,
      D => bram_we(3),
      Q => \bram_we_latched_reg_n_0_[3]\,
      R => '0'
    );
\latch_addr[0]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(0),
      I1 => bram_addr(0),
      I2 => state(1),
      O => latch_addr(0)
    );
\latch_addr[10]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(10),
      I1 => bram_addr(10),
      I2 => state(1),
      O => latch_addr(10)
    );
\latch_addr[11]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(11),
      I1 => bram_addr(11),
      I2 => state(1),
      O => latch_addr(11)
    );
\latch_addr[12]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(12),
      I1 => bram_addr(12),
      I2 => state(1),
      O => latch_addr(12)
    );
\latch_addr[13]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(13),
      I1 => bram_addr(13),
      I2 => state(1),
      O => latch_addr(13)
    );
\latch_addr[14]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(14),
      I1 => bram_addr(14),
      I2 => state(1),
      O => latch_addr(14)
    );
\latch_addr[15]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(15),
      I1 => bram_addr(15),
      I2 => state(1),
      O => latch_addr(15)
    );
\latch_addr[16]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(16),
      I1 => bram_addr(16),
      I2 => state(1),
      O => latch_addr(16)
    );
\latch_addr[17]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(17),
      I1 => bram_addr(17),
      I2 => state(1),
      O => latch_addr(17)
    );
\latch_addr[18]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(18),
      I1 => bram_addr(18),
      I2 => state(1),
      O => latch_addr(18)
    );
\latch_addr[19]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(19),
      I1 => bram_addr(19),
      I2 => state(1),
      O => latch_addr(19)
    );
\latch_addr[1]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(1),
      I1 => bram_addr(1),
      I2 => state(1),
      O => latch_addr(1)
    );
\latch_addr[20]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(20),
      I1 => bram_addr(20),
      I2 => state(1),
      O => latch_addr(20)
    );
\latch_addr[21]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(21),
      I1 => bram_addr(21),
      I2 => state(1),
      O => latch_addr(21)
    );
\latch_addr[22]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(22),
      I1 => bram_addr(22),
      I2 => state(1),
      O => latch_addr(22)
    );
\latch_addr[23]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(23),
      I1 => bram_addr(23),
      I2 => state(1),
      O => latch_addr(23)
    );
\latch_addr[24]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(24),
      I1 => bram_addr(24),
      I2 => state(1),
      O => latch_addr(24)
    );
\latch_addr[25]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(25),
      I1 => bram_addr(25),
      I2 => state(1),
      O => latch_addr(25)
    );
\latch_addr[26]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(26),
      I1 => bram_addr(26),
      I2 => state(1),
      O => latch_addr(26)
    );
\latch_addr[27]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(27),
      I1 => bram_addr(27),
      I2 => state(1),
      O => latch_addr(27)
    );
\latch_addr[28]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(28),
      I1 => bram_addr(28),
      I2 => state(1),
      O => latch_addr(28)
    );
\latch_addr[29]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(29),
      I1 => bram_addr(29),
      I2 => state(1),
      O => latch_addr(29)
    );
\latch_addr[2]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(2),
      I1 => bram_addr(2),
      I2 => state(1),
      O => latch_addr(2)
    );
\latch_addr[30]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(30),
      I1 => bram_addr(30),
      I2 => state(1),
      O => latch_addr(30)
    );
\latch_addr[31]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(31),
      I1 => bram_addr(31),
      I2 => state(1),
      O => latch_addr(31)
    );
\latch_addr[3]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(3),
      I1 => bram_addr(3),
      I2 => state(1),
      O => latch_addr(3)
    );
\latch_addr[4]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(4),
      I1 => bram_addr(4),
      I2 => state(1),
      O => latch_addr(4)
    );
\latch_addr[5]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(5),
      I1 => bram_addr(5),
      I2 => state(1),
      O => latch_addr(5)
    );
\latch_addr[6]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(6),
      I1 => bram_addr(6),
      I2 => state(1),
      O => latch_addr(6)
    );
\latch_addr[7]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(7),
      I1 => bram_addr(7),
      I2 => state(1),
      O => latch_addr(7)
    );
\latch_addr[8]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(8),
      I1 => bram_addr(8),
      I2 => state(1),
      O => latch_addr(8)
    );
\latch_addr[9]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => addr_in(9),
      I1 => bram_addr(9),
      I2 => state(1),
      O => latch_addr(9)
    );
\latch_addr_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(0),
      Q => \latch_addr_reg_n_0_[0]\
    );
\latch_addr_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(10),
      Q => \latch_addr_reg_n_0_[10]\
    );
\latch_addr_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(11),
      Q => \latch_addr_reg_n_0_[11]\
    );
\latch_addr_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(12),
      Q => \latch_addr_reg_n_0_[12]\
    );
\latch_addr_reg[13]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(13),
      Q => \latch_addr_reg_n_0_[13]\
    );
\latch_addr_reg[14]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(14),
      Q => \latch_addr_reg_n_0_[14]\
    );
\latch_addr_reg[15]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(15),
      Q => \latch_addr_reg_n_0_[15]\
    );
\latch_addr_reg[16]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(16),
      Q => \latch_addr_reg_n_0_[16]\
    );
\latch_addr_reg[17]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(17),
      Q => \latch_addr_reg_n_0_[17]\
    );
\latch_addr_reg[18]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(18),
      Q => \latch_addr_reg_n_0_[18]\
    );
\latch_addr_reg[19]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(19),
      Q => \latch_addr_reg_n_0_[19]\
    );
\latch_addr_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(1),
      Q => \latch_addr_reg_n_0_[1]\
    );
\latch_addr_reg[20]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(20),
      Q => \latch_addr_reg_n_0_[20]\
    );
\latch_addr_reg[21]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(21),
      Q => \latch_addr_reg_n_0_[21]\
    );
\latch_addr_reg[22]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(22),
      Q => \latch_addr_reg_n_0_[22]\
    );
\latch_addr_reg[23]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(23),
      Q => \latch_addr_reg_n_0_[23]\
    );
\latch_addr_reg[24]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(24),
      Q => \latch_addr_reg_n_0_[24]\
    );
\latch_addr_reg[25]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(25),
      Q => \latch_addr_reg_n_0_[25]\
    );
\latch_addr_reg[26]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(26),
      Q => \latch_addr_reg_n_0_[26]\
    );
\latch_addr_reg[27]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(27),
      Q => \latch_addr_reg_n_0_[27]\
    );
\latch_addr_reg[28]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(28),
      Q => \latch_addr_reg_n_0_[28]\
    );
\latch_addr_reg[29]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(29),
      Q => \latch_addr_reg_n_0_[29]\
    );
\latch_addr_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(2),
      Q => \latch_addr_reg_n_0_[2]\
    );
\latch_addr_reg[30]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(30),
      Q => \latch_addr_reg_n_0_[30]\
    );
\latch_addr_reg[31]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(31),
      Q => \latch_addr_reg_n_0_[31]\
    );
\latch_addr_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(3),
      Q => \latch_addr_reg_n_0_[3]\
    );
\latch_addr_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(4),
      Q => \latch_addr_reg_n_0_[4]\
    );
\latch_addr_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(5),
      Q => \latch_addr_reg_n_0_[5]\
    );
\latch_addr_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(6),
      Q => \latch_addr_reg_n_0_[6]\
    );
\latch_addr_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(7),
      Q => \latch_addr_reg_n_0_[7]\
    );
\latch_addr_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(8),
      Q => \latch_addr_reg_n_0_[8]\
    );
\latch_addr_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \M_AXI_AWADDR[31]_i_1_n_0\,
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => latch_addr(9),
      Q => \latch_addr_reg_n_0_[9]\
    );
\state_led[1]_i_1\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => state(1),
      O => \state_led[1]_i_1_n_0\
    );
\state_led_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => state(0),
      Q => state_led(0)
    );
\state_led_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \M_AXI_AWADDR[31]_i_2_n_0\,
      D => \state_led[1]_i_1_n_0\,
      Q => state_led(1)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_slave is
  port (
    init_write : out STD_LOGIC;
    aresetn_0 : out STD_LOGIC;
    S_AXI_BVALID_reg_0 : out STD_LOGIC;
    S_AXI_WREADY_reg_0 : out STD_LOGIC;
    S_AXI_AWREADY_reg_0 : out STD_LOGIC;
    S_AXI_RVALID_reg_0 : out STD_LOGIC;
    S_AXI_ARREADY_reg_0 : out STD_LOGIC;
    S_AXI_RDATA : out STD_LOGIC_VECTOR ( 1 downto 0 );
    aclk : in STD_LOGIC;
    S_AXI_ARVALID : in STD_LOGIC;
    S_AXI_BREADY : in STD_LOGIC;
    S_AXI_WVALID : in STD_LOGIC;
    S_AXI_AWVALID : in STD_LOGIC;
    S_AXI_RREADY : in STD_LOGIC;
    aresetn : in STD_LOGIC;
    S_AXI_AWADDR : in STD_LOGIC_VECTOR ( 31 downto 0 );
    S_AXI_WDATA : in STD_LOGIC_VECTOR ( 1 downto 0 );
    S_AXI_ARADDR : in STD_LOGIC_VECTOR ( 31 downto 0 )
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_slave;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_slave is
  signal S_AXI_ARREADY_i_1_n_0 : STD_LOGIC;
  signal \^s_axi_arready_reg_0\ : STD_LOGIC;
  signal S_AXI_AWREADY_i_1_n_0 : STD_LOGIC;
  signal \^s_axi_awready_reg_0\ : STD_LOGIC;
  signal S_AXI_BVALID_i_1_n_0 : STD_LOGIC;
  signal \^s_axi_bvalid_reg_0\ : STD_LOGIC;
  signal \^s_axi_rdata\ : STD_LOGIC_VECTOR ( 1 downto 0 );
  signal \S_AXI_RDATA[0]_i_1_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[1]_i_10_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[1]_i_11_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[1]_i_1_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[1]_i_2_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[1]_i_3_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[1]_i_4_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[1]_i_6_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[1]_i_7_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[1]_i_8_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[1]_i_9_n_0\ : STD_LOGIC;
  signal S_AXI_RVALID0 : STD_LOGIC;
  signal S_AXI_RVALID_i_1_n_0 : STD_LOGIC;
  signal \^s_axi_rvalid_reg_0\ : STD_LOGIC;
  signal S_AXI_WREADY_i_1_n_0 : STD_LOGIC;
  signal \^s_axi_wready_reg_0\ : STD_LOGIC;
  signal ar_latched_i_1_n_0 : STD_LOGIC;
  signal ar_latched_reg_n_0 : STD_LOGIC;
  signal araddr_reg : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal \araddr_reg[31]_i_1_n_0\ : STD_LOGIC;
  signal \^aresetn_0\ : STD_LOGIC;
  signal aw_latched_i_1_n_0 : STD_LOGIC;
  signal aw_latched_reg_n_0 : STD_LOGIC;
  signal awaddr_reg : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal awaddr_reg_0 : STD_LOGIC;
  signal mem_fin_i_1_n_0 : STD_LOGIC;
  signal mode : STD_LOGIC_VECTOR ( 1 downto 0 );
  signal \mode[0]_i_1_n_0\ : STD_LOGIC;
  signal \mode[1]_i_1_n_0\ : STD_LOGIC;
  signal \mode[1]_i_2_n_0\ : STD_LOGIC;
  signal \mode[1]_i_3_n_0\ : STD_LOGIC;
  signal \mode[1]_i_4_n_0\ : STD_LOGIC;
  signal \mode[1]_i_5_n_0\ : STD_LOGIC;
  signal \mode[1]_i_6_n_0\ : STD_LOGIC;
  signal \mode[1]_i_7_n_0\ : STD_LOGIC;
  signal \mode[1]_i_8_n_0\ : STD_LOGIC;
  signal w_latched_i_1_n_0 : STD_LOGIC;
  signal w_latched_reg_n_0 : STD_LOGIC;
  signal \wdata_reg[0]_i_1_n_0\ : STD_LOGIC;
  signal \wdata_reg[1]_i_1_n_0\ : STD_LOGIC;
  signal \wdata_reg_reg_n_0_[0]\ : STD_LOGIC;
  signal \wdata_reg_reg_n_0_[1]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of S_AXI_ARREADY_i_1 : label is "soft_lutpair38";
  attribute SOFT_HLUTNM of S_AXI_AWREADY_i_1 : label is "soft_lutpair35";
  attribute SOFT_HLUTNM of \S_AXI_RDATA[1]_i_5\ : label is "soft_lutpair40";
  attribute SOFT_HLUTNM of S_AXI_RVALID_i_1 : label is "soft_lutpair40";
  attribute SOFT_HLUTNM of S_AXI_WREADY_i_1 : label is "soft_lutpair36";
  attribute SOFT_HLUTNM of ar_latched_i_1 : label is "soft_lutpair38";
  attribute SOFT_HLUTNM of aw_latched_i_1 : label is "soft_lutpair35";
  attribute SOFT_HLUTNM of mem_fin_i_1 : label is "soft_lutpair37";
  attribute SOFT_HLUTNM of \mode[0]_i_1\ : label is "soft_lutpair39";
  attribute SOFT_HLUTNM of \mode[1]_i_1\ : label is "soft_lutpair39";
  attribute SOFT_HLUTNM of \mode[1]_i_5\ : label is "soft_lutpair37";
  attribute SOFT_HLUTNM of w_latched_i_1 : label is "soft_lutpair36";
begin
  S_AXI_ARREADY_reg_0 <= \^s_axi_arready_reg_0\;
  S_AXI_AWREADY_reg_0 <= \^s_axi_awready_reg_0\;
  S_AXI_BVALID_reg_0 <= \^s_axi_bvalid_reg_0\;
  S_AXI_RDATA(1 downto 0) <= \^s_axi_rdata\(1 downto 0);
  S_AXI_RVALID_reg_0 <= \^s_axi_rvalid_reg_0\;
  S_AXI_WREADY_reg_0 <= \^s_axi_wready_reg_0\;
  aresetn_0 <= \^aresetn_0\;
S_AXI_ARREADY_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"15"
    )
        port map (
      I0 => ar_latched_reg_n_0,
      I1 => \^s_axi_arready_reg_0\,
      I2 => S_AXI_ARVALID,
      O => S_AXI_ARREADY_i_1_n_0
    );
S_AXI_ARREADY_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => S_AXI_ARREADY_i_1_n_0,
      Q => \^s_axi_arready_reg_0\
    );
S_AXI_AWREADY_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"07"
    )
        port map (
      I0 => \^s_axi_awready_reg_0\,
      I1 => S_AXI_AWVALID,
      I2 => aw_latched_reg_n_0,
      O => S_AXI_AWREADY_i_1_n_0
    );
S_AXI_AWREADY_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => S_AXI_AWREADY_i_1_n_0,
      Q => \^s_axi_awready_reg_0\
    );
S_AXI_BVALID_i_1: unisim.vcomponents.LUT4
    generic map(
      INIT => X"5C50"
    )
        port map (
      I0 => S_AXI_BREADY,
      I1 => aw_latched_reg_n_0,
      I2 => \^s_axi_bvalid_reg_0\,
      I3 => w_latched_reg_n_0,
      O => S_AXI_BVALID_i_1_n_0
    );
S_AXI_BVALID_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => S_AXI_BVALID_i_1_n_0,
      Q => \^s_axi_bvalid_reg_0\
    );
\S_AXI_RDATA[0]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0002FFFF00020000"
    )
        port map (
      I0 => mode(0),
      I1 => \S_AXI_RDATA[1]_i_2_n_0\,
      I2 => \S_AXI_RDATA[1]_i_3_n_0\,
      I3 => \S_AXI_RDATA[1]_i_4_n_0\,
      I4 => S_AXI_RVALID0,
      I5 => \^s_axi_rdata\(0),
      O => \S_AXI_RDATA[0]_i_1_n_0\
    );
\S_AXI_RDATA[1]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0002FFFF00020000"
    )
        port map (
      I0 => mode(1),
      I1 => \S_AXI_RDATA[1]_i_2_n_0\,
      I2 => \S_AXI_RDATA[1]_i_3_n_0\,
      I3 => \S_AXI_RDATA[1]_i_4_n_0\,
      I4 => S_AXI_RVALID0,
      I5 => \^s_axi_rdata\(1),
      O => \S_AXI_RDATA[1]_i_1_n_0\
    );
\S_AXI_RDATA[1]_i_10\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => araddr_reg(27),
      I1 => araddr_reg(28),
      I2 => araddr_reg(25),
      I3 => araddr_reg(26),
      O => \S_AXI_RDATA[1]_i_10_n_0\
    );
\S_AXI_RDATA[1]_i_11\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => araddr_reg(19),
      I1 => araddr_reg(20),
      I2 => araddr_reg(17),
      I3 => araddr_reg(18),
      O => \S_AXI_RDATA[1]_i_11_n_0\
    );
\S_AXI_RDATA[1]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => \S_AXI_RDATA[1]_i_6_n_0\,
      I1 => \S_AXI_RDATA[1]_i_7_n_0\,
      I2 => \S_AXI_RDATA[1]_i_8_n_0\,
      I3 => \S_AXI_RDATA[1]_i_9_n_0\,
      O => \S_AXI_RDATA[1]_i_2_n_0\
    );
\S_AXI_RDATA[1]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFFFFFE"
    )
        port map (
      I0 => araddr_reg(30),
      I1 => araddr_reg(29),
      I2 => araddr_reg(1),
      I3 => araddr_reg(0),
      I4 => \S_AXI_RDATA[1]_i_10_n_0\,
      O => \S_AXI_RDATA[1]_i_3_n_0\
    );
\S_AXI_RDATA[1]_i_4\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFFFFFE"
    )
        port map (
      I0 => araddr_reg(22),
      I1 => araddr_reg(21),
      I2 => araddr_reg(24),
      I3 => araddr_reg(23),
      I4 => \S_AXI_RDATA[1]_i_11_n_0\,
      O => \S_AXI_RDATA[1]_i_4_n_0\
    );
\S_AXI_RDATA[1]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => ar_latched_reg_n_0,
      I1 => \^s_axi_rvalid_reg_0\,
      O => S_AXI_RVALID0
    );
\S_AXI_RDATA[1]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => araddr_reg(11),
      I1 => araddr_reg(12),
      I2 => araddr_reg(9),
      I3 => araddr_reg(10),
      O => \S_AXI_RDATA[1]_i_6_n_0\
    );
\S_AXI_RDATA[1]_i_7\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => araddr_reg(15),
      I1 => araddr_reg(16),
      I2 => araddr_reg(13),
      I3 => araddr_reg(14),
      O => \S_AXI_RDATA[1]_i_7_n_0\
    );
\S_AXI_RDATA[1]_i_8\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFEF"
    )
        port map (
      I0 => araddr_reg(3),
      I1 => araddr_reg(4),
      I2 => araddr_reg(31),
      I3 => araddr_reg(2),
      O => \S_AXI_RDATA[1]_i_8_n_0\
    );
\S_AXI_RDATA[1]_i_9\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => araddr_reg(7),
      I1 => araddr_reg(8),
      I2 => araddr_reg(5),
      I3 => araddr_reg(6),
      O => \S_AXI_RDATA[1]_i_9_n_0\
    );
\S_AXI_RDATA_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => \S_AXI_RDATA[0]_i_1_n_0\,
      Q => \^s_axi_rdata\(0)
    );
\S_AXI_RDATA_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => \S_AXI_RDATA[1]_i_1_n_0\,
      Q => \^s_axi_rdata\(1)
    );
S_AXI_RVALID_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"74"
    )
        port map (
      I0 => S_AXI_RREADY,
      I1 => \^s_axi_rvalid_reg_0\,
      I2 => ar_latched_reg_n_0,
      O => S_AXI_RVALID_i_1_n_0
    );
S_AXI_RVALID_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => S_AXI_RVALID_i_1_n_0,
      Q => \^s_axi_rvalid_reg_0\
    );
S_AXI_WREADY_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"13"
    )
        port map (
      I0 => \^s_axi_wready_reg_0\,
      I1 => w_latched_reg_n_0,
      I2 => S_AXI_WVALID,
      O => S_AXI_WREADY_i_1_n_0
    );
S_AXI_WREADY_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => S_AXI_WREADY_i_1_n_0,
      Q => \^s_axi_wready_reg_0\
    );
a_en_i_2: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => aresetn,
      O => \^aresetn_0\
    );
ar_latched_i_1: unisim.vcomponents.LUT4
    generic map(
      INIT => X"B888"
    )
        port map (
      I0 => \^s_axi_rvalid_reg_0\,
      I1 => ar_latched_reg_n_0,
      I2 => \^s_axi_arready_reg_0\,
      I3 => S_AXI_ARVALID,
      O => ar_latched_i_1_n_0
    );
ar_latched_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => ar_latched_i_1_n_0,
      Q => ar_latched_reg_n_0
    );
\araddr_reg[31]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => S_AXI_ARVALID,
      I1 => \^s_axi_arready_reg_0\,
      O => \araddr_reg[31]_i_1_n_0\
    );
\araddr_reg_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(0),
      Q => araddr_reg(0)
    );
\araddr_reg_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(10),
      Q => araddr_reg(10)
    );
\araddr_reg_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(11),
      Q => araddr_reg(11)
    );
\araddr_reg_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(12),
      Q => araddr_reg(12)
    );
\araddr_reg_reg[13]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(13),
      Q => araddr_reg(13)
    );
\araddr_reg_reg[14]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(14),
      Q => araddr_reg(14)
    );
\araddr_reg_reg[15]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(15),
      Q => araddr_reg(15)
    );
\araddr_reg_reg[16]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(16),
      Q => araddr_reg(16)
    );
\araddr_reg_reg[17]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(17),
      Q => araddr_reg(17)
    );
\araddr_reg_reg[18]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(18),
      Q => araddr_reg(18)
    );
\araddr_reg_reg[19]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(19),
      Q => araddr_reg(19)
    );
\araddr_reg_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(1),
      Q => araddr_reg(1)
    );
\araddr_reg_reg[20]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(20),
      Q => araddr_reg(20)
    );
\araddr_reg_reg[21]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(21),
      Q => araddr_reg(21)
    );
\araddr_reg_reg[22]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(22),
      Q => araddr_reg(22)
    );
\araddr_reg_reg[23]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(23),
      Q => araddr_reg(23)
    );
\araddr_reg_reg[24]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(24),
      Q => araddr_reg(24)
    );
\araddr_reg_reg[25]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(25),
      Q => araddr_reg(25)
    );
\araddr_reg_reg[26]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(26),
      Q => araddr_reg(26)
    );
\araddr_reg_reg[27]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(27),
      Q => araddr_reg(27)
    );
\araddr_reg_reg[28]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(28),
      Q => araddr_reg(28)
    );
\araddr_reg_reg[29]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(29),
      Q => araddr_reg(29)
    );
\araddr_reg_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(2),
      Q => araddr_reg(2)
    );
\araddr_reg_reg[30]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(30),
      Q => araddr_reg(30)
    );
\araddr_reg_reg[31]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(31),
      Q => araddr_reg(31)
    );
\araddr_reg_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(3),
      Q => araddr_reg(3)
    );
\araddr_reg_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(4),
      Q => araddr_reg(4)
    );
\araddr_reg_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(5),
      Q => araddr_reg(5)
    );
\araddr_reg_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(6),
      Q => araddr_reg(6)
    );
\araddr_reg_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(7),
      Q => araddr_reg(7)
    );
\araddr_reg_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(8),
      Q => araddr_reg(8)
    );
\araddr_reg_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \araddr_reg[31]_i_1_n_0\,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(9),
      Q => araddr_reg(9)
    );
aw_latched_i_1: unisim.vcomponents.LUT5
    generic map(
      INIT => X"F808F8F8"
    )
        port map (
      I0 => S_AXI_AWVALID,
      I1 => \^s_axi_awready_reg_0\,
      I2 => aw_latched_reg_n_0,
      I3 => \^s_axi_bvalid_reg_0\,
      I4 => w_latched_reg_n_0,
      O => aw_latched_i_1_n_0
    );
aw_latched_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => aw_latched_i_1_n_0,
      Q => aw_latched_reg_n_0
    );
\awaddr_reg[31]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \^s_axi_awready_reg_0\,
      I1 => S_AXI_AWVALID,
      O => awaddr_reg_0
    );
\awaddr_reg_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(0),
      Q => awaddr_reg(0)
    );
\awaddr_reg_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(10),
      Q => awaddr_reg(10)
    );
\awaddr_reg_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(11),
      Q => awaddr_reg(11)
    );
\awaddr_reg_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(12),
      Q => awaddr_reg(12)
    );
\awaddr_reg_reg[13]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(13),
      Q => awaddr_reg(13)
    );
\awaddr_reg_reg[14]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(14),
      Q => awaddr_reg(14)
    );
\awaddr_reg_reg[15]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(15),
      Q => awaddr_reg(15)
    );
\awaddr_reg_reg[16]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(16),
      Q => awaddr_reg(16)
    );
\awaddr_reg_reg[17]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(17),
      Q => awaddr_reg(17)
    );
\awaddr_reg_reg[18]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(18),
      Q => awaddr_reg(18)
    );
\awaddr_reg_reg[19]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(19),
      Q => awaddr_reg(19)
    );
\awaddr_reg_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(1),
      Q => awaddr_reg(1)
    );
\awaddr_reg_reg[20]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(20),
      Q => awaddr_reg(20)
    );
\awaddr_reg_reg[21]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(21),
      Q => awaddr_reg(21)
    );
\awaddr_reg_reg[22]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(22),
      Q => awaddr_reg(22)
    );
\awaddr_reg_reg[23]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(23),
      Q => awaddr_reg(23)
    );
\awaddr_reg_reg[24]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(24),
      Q => awaddr_reg(24)
    );
\awaddr_reg_reg[25]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(25),
      Q => awaddr_reg(25)
    );
\awaddr_reg_reg[26]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(26),
      Q => awaddr_reg(26)
    );
\awaddr_reg_reg[27]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(27),
      Q => awaddr_reg(27)
    );
\awaddr_reg_reg[28]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(28),
      Q => awaddr_reg(28)
    );
\awaddr_reg_reg[29]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(29),
      Q => awaddr_reg(29)
    );
\awaddr_reg_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(2),
      Q => awaddr_reg(2)
    );
\awaddr_reg_reg[30]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(30),
      Q => awaddr_reg(30)
    );
\awaddr_reg_reg[31]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(31),
      Q => awaddr_reg(31)
    );
\awaddr_reg_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(3),
      Q => awaddr_reg(3)
    );
\awaddr_reg_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(4),
      Q => awaddr_reg(4)
    );
\awaddr_reg_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(5),
      Q => awaddr_reg(5)
    );
\awaddr_reg_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(6),
      Q => awaddr_reg(6)
    );
\awaddr_reg_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(7),
      Q => awaddr_reg(7)
    );
\awaddr_reg_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(8),
      Q => awaddr_reg(8)
    );
\awaddr_reg_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => awaddr_reg_0,
      CLR => \^aresetn_0\,
      D => S_AXI_AWADDR(9),
      Q => awaddr_reg(9)
    );
mem_fin_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"20"
    )
        port map (
      I0 => aw_latched_reg_n_0,
      I1 => \^s_axi_bvalid_reg_0\,
      I2 => w_latched_reg_n_0,
      O => mem_fin_i_1_n_0
    );
mem_fin_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => mem_fin_i_1_n_0,
      Q => init_write
    );
\mode[0]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \wdata_reg_reg_n_0_[0]\,
      I1 => \mode[1]_i_2_n_0\,
      I2 => mode(0),
      O => \mode[0]_i_1_n_0\
    );
\mode[1]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \wdata_reg_reg_n_0_[1]\,
      I1 => \mode[1]_i_2_n_0\,
      I2 => mode(1),
      O => \mode[1]_i_1_n_0\
    );
\mode[1]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"8000000000000000"
    )
        port map (
      I0 => \mode[1]_i_3_n_0\,
      I1 => \mode[1]_i_4_n_0\,
      I2 => \mode[1]_i_5_n_0\,
      I3 => \mode[1]_i_6_n_0\,
      I4 => \mode[1]_i_7_n_0\,
      I5 => \mode[1]_i_8_n_0\,
      O => \mode[1]_i_2_n_0\
    );
\mode[1]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000000001"
    )
        port map (
      I0 => awaddr_reg(22),
      I1 => awaddr_reg(23),
      I2 => awaddr_reg(20),
      I3 => awaddr_reg(21),
      I4 => awaddr_reg(25),
      I5 => awaddr_reg(24),
      O => \mode[1]_i_3_n_0\
    );
\mode[1]_i_4\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000100000000"
    )
        port map (
      I0 => awaddr_reg(28),
      I1 => awaddr_reg(29),
      I2 => awaddr_reg(26),
      I3 => awaddr_reg(27),
      I4 => awaddr_reg(30),
      I5 => awaddr_reg(31),
      O => \mode[1]_i_4_n_0\
    );
\mode[1]_i_5\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00100000"
    )
        port map (
      I0 => awaddr_reg(1),
      I1 => awaddr_reg(0),
      I2 => aw_latched_reg_n_0,
      I3 => \^s_axi_bvalid_reg_0\,
      I4 => w_latched_reg_n_0,
      O => \mode[1]_i_5_n_0\
    );
\mode[1]_i_6\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000000001"
    )
        port map (
      I0 => awaddr_reg(4),
      I1 => awaddr_reg(5),
      I2 => awaddr_reg(2),
      I3 => awaddr_reg(3),
      I4 => awaddr_reg(7),
      I5 => awaddr_reg(6),
      O => \mode[1]_i_6_n_0\
    );
\mode[1]_i_7\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000000001"
    )
        port map (
      I0 => awaddr_reg(16),
      I1 => awaddr_reg(17),
      I2 => awaddr_reg(14),
      I3 => awaddr_reg(15),
      I4 => awaddr_reg(19),
      I5 => awaddr_reg(18),
      O => \mode[1]_i_7_n_0\
    );
\mode[1]_i_8\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000000001"
    )
        port map (
      I0 => awaddr_reg(10),
      I1 => awaddr_reg(11),
      I2 => awaddr_reg(8),
      I3 => awaddr_reg(9),
      I4 => awaddr_reg(13),
      I5 => awaddr_reg(12),
      O => \mode[1]_i_8_n_0\
    );
\mode_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => \mode[0]_i_1_n_0\,
      Q => mode(0)
    );
\mode_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => \mode[1]_i_1_n_0\,
      Q => mode(1)
    );
w_latched_i_1: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FF0F8888"
    )
        port map (
      I0 => \^s_axi_wready_reg_0\,
      I1 => S_AXI_WVALID,
      I2 => aw_latched_reg_n_0,
      I3 => \^s_axi_bvalid_reg_0\,
      I4 => w_latched_reg_n_0,
      O => w_latched_i_1_n_0
    );
w_latched_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => w_latched_i_1_n_0,
      Q => w_latched_reg_n_0
    );
\wdata_reg[0]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"BF80"
    )
        port map (
      I0 => S_AXI_WDATA(0),
      I1 => S_AXI_WVALID,
      I2 => \^s_axi_wready_reg_0\,
      I3 => \wdata_reg_reg_n_0_[0]\,
      O => \wdata_reg[0]_i_1_n_0\
    );
\wdata_reg[1]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"BF80"
    )
        port map (
      I0 => S_AXI_WDATA(1),
      I1 => S_AXI_WVALID,
      I2 => \^s_axi_wready_reg_0\,
      I3 => \wdata_reg_reg_n_0_[1]\,
      O => \wdata_reg[1]_i_1_n_0\
    );
\wdata_reg_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => \wdata_reg[0]_i_1_n_0\,
      Q => \wdata_reg_reg_n_0_[0]\
    );
\wdata_reg_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => \wdata_reg[1]_i_1_n_0\,
      Q => \wdata_reg_reg_n_0_[1]\
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_bram_driver_stimulus is
  port (
    bram_web : out STD_LOGIC_VECTOR ( 0 to 0 );
    bram_addrb : out STD_LOGIC_VECTOR ( 0 to 0 );
    bram_dina : out STD_LOGIC_VECTOR ( 0 to 0 );
    aclk : in STD_LOGIC;
    a_en_reg_0 : in STD_LOGIC
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_bram_driver_stimulus;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_bram_driver_stimulus is
  signal \FSM_sequential_state[0]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_sequential_state[1]_i_1_n_0\ : STD_LOGIC;
  signal a_en_i_1_n_0 : STD_LOGIC;
  signal a_we_i_1_n_0 : STD_LOGIC;
  signal \b_addr[0]_i_1_n_0\ : STD_LOGIC;
  signal \^bram_addrb\ : STD_LOGIC_VECTOR ( 0 to 0 );
  signal \^bram_dina\ : STD_LOGIC_VECTOR ( 0 to 0 );
  signal \^bram_web\ : STD_LOGIC_VECTOR ( 0 to 0 );
  signal state : STD_LOGIC_VECTOR ( 1 downto 0 );
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \FSM_sequential_state[1]_i_1\ : label is "soft_lutpair42";
  attribute FSM_ENCODED_STATES : string;
  attribute FSM_ENCODED_STATES of \FSM_sequential_state_reg[0]\ : label is "IDLE:00,WRITE:01,READ:10,HOLD:11";
  attribute FSM_ENCODED_STATES of \FSM_sequential_state_reg[1]\ : label is "IDLE:00,WRITE:01,READ:10,HOLD:11";
  attribute SOFT_HLUTNM of a_en_i_1 : label is "soft_lutpair41";
  attribute SOFT_HLUTNM of a_we_i_1 : label is "soft_lutpair41";
  attribute SOFT_HLUTNM of \b_addr[0]_i_1\ : label is "soft_lutpair42";
begin
  bram_addrb(0) <= \^bram_addrb\(0);
  bram_dina(0) <= \^bram_dina\(0);
  bram_web(0) <= \^bram_web\(0);
\FSM_sequential_state[0]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"B"
    )
        port map (
      I0 => state(1),
      I1 => state(0),
      O => \FSM_sequential_state[0]_i_1_n_0\
    );
\FSM_sequential_state[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => state(0),
      I1 => state(1),
      O => \FSM_sequential_state[1]_i_1_n_0\
    );
\FSM_sequential_state_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => a_en_reg_0,
      D => \FSM_sequential_state[0]_i_1_n_0\,
      Q => state(0)
    );
\FSM_sequential_state_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => a_en_reg_0,
      D => \FSM_sequential_state[1]_i_1_n_0\,
      Q => state(1)
    );
a_en_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"F1"
    )
        port map (
      I0 => state(1),
      I1 => state(0),
      I2 => \^bram_dina\(0),
      O => a_en_i_1_n_0
    );
a_en_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => a_en_reg_0,
      D => a_en_i_1_n_0,
      Q => \^bram_dina\(0)
    );
a_we_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"8B"
    )
        port map (
      I0 => \^bram_web\(0),
      I1 => state(1),
      I2 => state(0),
      O => a_we_i_1_n_0
    );
a_we_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => a_en_reg_0,
      D => a_we_i_1_n_0,
      Q => \^bram_web\(0)
    );
\b_addr[0]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"B"
    )
        port map (
      I0 => \^bram_addrb\(0),
      I1 => state(0),
      O => \b_addr[0]_i_1_n_0\
    );
\b_addr_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => a_en_reg_0,
      D => \b_addr[0]_i_1_n_0\,
      Q => \^bram_addrb\(0)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mem_controller is
  port (
    bram_addrb : out STD_LOGIC_VECTOR ( 0 to 0 );
    S_AXI_ARREADY_reg : out STD_LOGIC;
    bram_dout : out STD_LOGIC_VECTOR ( 31 downto 0 );
    M_AXI_AWADDR : out STD_LOGIC_VECTOR ( 31 downto 0 );
    M_AXI_AWVALID : out STD_LOGIC;
    M_AXI_WDATA : out STD_LOGIC_VECTOR ( 31 downto 0 );
    M_AXI_WSTRB : out STD_LOGIC_VECTOR ( 3 downto 0 );
    M_AXI_WVALID : out STD_LOGIC;
    M_AXI_BREADY : out STD_LOGIC;
    S_AXI_WREADY_reg : out STD_LOGIC;
    S_AXI_RDATA : out STD_LOGIC_VECTOR ( 1 downto 0 );
    S_AXI_BVALID_reg : out STD_LOGIC;
    S_AXI_AWREADY_reg : out STD_LOGIC;
    S_AXI_RVALID_reg : out STD_LOGIC;
    bram_dina : out STD_LOGIC_VECTOR ( 0 to 0 );
    bram_web : out STD_LOGIC_VECTOR ( 0 to 0 );
    S_AXI_ARVALID : in STD_LOGIC;
    aclk : in STD_LOGIC;
    bram_en : in STD_LOGIC;
    bram_addr : in STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_din : in STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_we : in STD_LOGIC_VECTOR ( 3 downto 0 );
    aresetn : in STD_LOGIC;
    M_AXI_AWREADY : in STD_LOGIC;
    M_AXI_WREADY : in STD_LOGIC;
    M_AXI_BRESP : in STD_LOGIC_VECTOR ( 1 downto 0 );
    M_AXI_BVALID : in STD_LOGIC;
    S_AXI_AWADDR : in STD_LOGIC_VECTOR ( 31 downto 0 );
    S_AXI_WDATA : in STD_LOGIC_VECTOR ( 1 downto 0 );
    S_AXI_WVALID : in STD_LOGIC;
    S_AXI_ARADDR : in STD_LOGIC_VECTOR ( 31 downto 0 );
    S_AXI_AWVALID : in STD_LOGIC;
    S_AXI_BREADY : in STD_LOGIC;
    S_AXI_RREADY : in STD_LOGIC
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mem_controller;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mem_controller is
  signal init_out : STD_LOGIC;
  signal slave1_n_1 : STD_LOGIC;
  signal NLW_master1_bram_ready_UNCONNECTED : STD_LOGIC;
  signal NLW_master1_state_led_UNCONNECTED : STD_LOGIC_VECTOR ( 1 downto 0 );
  attribute ST_AWBRAM : string;
  attribute ST_AWBRAM of master1 : label is "2'b11";
  attribute ST_AW_W : string;
  attribute ST_AW_W of master1 : label is "2'b00";
  attribute ST_B : string;
  attribute ST_B of master1 : label is "2'b01";
  attribute ST_DONE : string;
  attribute ST_DONE of master1 : label is "2'b10";
begin
master1: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master
     port map (
      M_AXI_AWADDR(31 downto 0) => M_AXI_AWADDR(31 downto 0),
      M_AXI_AWREADY => M_AXI_AWREADY,
      M_AXI_AWVALID => M_AXI_AWVALID,
      M_AXI_BREADY => M_AXI_BREADY,
      M_AXI_BRESP(1 downto 0) => M_AXI_BRESP(1 downto 0),
      M_AXI_BVALID => M_AXI_BVALID,
      M_AXI_WDATA(31 downto 0) => M_AXI_WDATA(31 downto 0),
      M_AXI_WREADY => M_AXI_WREADY,
      M_AXI_WSTRB(3 downto 0) => M_AXI_WSTRB(3 downto 0),
      M_AXI_WVALID => M_AXI_WVALID,
      aclk => aclk,
      addr_in(31 downto 0) => B"00000000000000000000000000000000",
      aresetn => aresetn,
      bram_addr(31 downto 0) => bram_addr(31 downto 0),
      bram_din(31 downto 0) => bram_din(31 downto 0),
      bram_dout(31 downto 0) => bram_dout(31 downto 0),
      bram_en => bram_en,
      bram_ready => NLW_master1_bram_ready_UNCONNECTED,
      bram_we(3 downto 0) => bram_we(3 downto 0),
      init_write => init_out,
      state_led(1 downto 0) => NLW_master1_state_led_UNCONNECTED(1 downto 0),
      value_out(31 downto 0) => B"00000000000000000000000000000000"
    );
slave1: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_slave
     port map (
      S_AXI_ARADDR(31 downto 0) => S_AXI_ARADDR(31 downto 0),
      S_AXI_ARREADY_reg_0 => S_AXI_ARREADY_reg,
      S_AXI_ARVALID => S_AXI_ARVALID,
      S_AXI_AWADDR(31 downto 0) => S_AXI_AWADDR(31 downto 0),
      S_AXI_AWREADY_reg_0 => S_AXI_AWREADY_reg,
      S_AXI_AWVALID => S_AXI_AWVALID,
      S_AXI_BREADY => S_AXI_BREADY,
      S_AXI_BVALID_reg_0 => S_AXI_BVALID_reg,
      S_AXI_RDATA(1 downto 0) => S_AXI_RDATA(1 downto 0),
      S_AXI_RREADY => S_AXI_RREADY,
      S_AXI_RVALID_reg_0 => S_AXI_RVALID_reg,
      S_AXI_WDATA(1 downto 0) => S_AXI_WDATA(1 downto 0),
      S_AXI_WREADY_reg_0 => S_AXI_WREADY_reg,
      S_AXI_WVALID => S_AXI_WVALID,
      aclk => aclk,
      aresetn => aresetn,
      aresetn_0 => slave1_n_1,
      init_write => init_out
    );
stimmy: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_bram_driver_stimulus
     port map (
      a_en_reg_0 => slave1_n_1,
      aclk => aclk,
      bram_addrb(0) => bram_addrb(0),
      bram_dina(0) => bram_dina(0),
      bram_web(0) => bram_web(0)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  port (
    aclk : in STD_LOGIC;
    aresetn : in STD_LOGIC;
    a_en : in STD_LOGIC;
    a_we : in STD_LOGIC;
    a_addr : in STD_LOGIC_VECTOR ( 3 downto 0 );
    a_din : in STD_LOGIC_VECTOR ( 31 downto 0 );
    a_dout : out STD_LOGIC_VECTOR ( 31 downto 0 );
    b_en : in STD_LOGIC;
    b_we : in STD_LOGIC;
    b_addr : in STD_LOGIC_VECTOR ( 3 downto 0 );
    b_din : in STD_LOGIC_VECTOR ( 31 downto 0 );
    b_dout : out STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_ena : out STD_LOGIC;
    bram_wea : out STD_LOGIC_VECTOR ( 3 downto 0 );
    bram_addra : out STD_LOGIC_VECTOR ( 3 downto 0 );
    bram_dina : out STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_douta : in STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_rsta : out STD_LOGIC;
    bram_enb : out STD_LOGIC;
    bram_web : out STD_LOGIC_VECTOR ( 3 downto 0 );
    bram_addrb : out STD_LOGIC_VECTOR ( 3 downto 0 );
    bram_dinb : out STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_doutb : in STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_rstb : out STD_LOGIC;
    bram_en : in STD_LOGIC;
    bram_addr : in STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_din : in STD_LOGIC_VECTOR ( 31 downto 0 );
    bram_we : in STD_LOGIC_VECTOR ( 3 downto 0 );
    bram_dout : out STD_LOGIC_VECTOR ( 31 downto 0 );
    S_AXI_AWADDR : in STD_LOGIC_VECTOR ( 31 downto 0 );
    S_AXI_AWVALID : in STD_LOGIC;
    S_AXI_AWREADY : out STD_LOGIC;
    S_AXI_WDATA : in STD_LOGIC_VECTOR ( 31 downto 0 );
    S_AXI_WSTRB : in STD_LOGIC_VECTOR ( 3 downto 0 );
    S_AXI_WVALID : in STD_LOGIC;
    S_AXI_WREADY : out STD_LOGIC;
    S_AXI_BRESP : out STD_LOGIC_VECTOR ( 1 downto 0 );
    S_AXI_BVALID : out STD_LOGIC;
    S_AXI_BREADY : in STD_LOGIC;
    S_AXI_ARADDR : in STD_LOGIC_VECTOR ( 31 downto 0 );
    S_AXI_ARVALID : in STD_LOGIC;
    S_AXI_ARREADY : out STD_LOGIC;
    S_AXI_RDATA : out STD_LOGIC_VECTOR ( 31 downto 0 );
    S_AXI_RRESP : out STD_LOGIC_VECTOR ( 1 downto 0 );
    S_AXI_RVALID : out STD_LOGIC;
    S_AXI_RREADY : in STD_LOGIC;
    M_AXI_AWADDR : out STD_LOGIC_VECTOR ( 31 downto 0 );
    M_AXI_AWVALID : out STD_LOGIC;
    M_AXI_AWREADY : in STD_LOGIC;
    M_AXI_WDATA : out STD_LOGIC_VECTOR ( 31 downto 0 );
    M_AXI_WSTRB : out STD_LOGIC_VECTOR ( 3 downto 0 );
    M_AXI_WVALID : out STD_LOGIC;
    M_AXI_WREADY : in STD_LOGIC;
    M_AXI_BRESP : in STD_LOGIC_VECTOR ( 1 downto 0 );
    M_AXI_BVALID : in STD_LOGIC;
    M_AXI_BREADY : out STD_LOGIC
  );
  attribute NotValidForBitStream : boolean;
  attribute NotValidForBitStream of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is true;
  attribute CHECK_LICENSE_TYPE : string;
  attribute CHECK_LICENSE_TYPE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "design_1_mem_controller_0_1,mem_controller,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "package_project";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "mem_controller,Vivado 2018.3.1";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  signal \<const0>\ : STD_LOGIC;
  signal \^s_axi_rdata\ : STD_LOGIC_VECTOR ( 1 downto 0 );
  signal \^aresetn\ : STD_LOGIC;
  signal \^bram_addrb\ : STD_LOGIC_VECTOR ( 0 to 0 );
  signal \^bram_dina\ : STD_LOGIC_VECTOR ( 31 to 31 );
  signal \^bram_web\ : STD_LOGIC_VECTOR ( 0 to 0 );
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_INFO of M_AXI_AWREADY : signal is "xilinx.com:interface:aximm:1.0 M_AXI AWREADY";
  attribute X_INTERFACE_INFO of M_AXI_AWVALID : signal is "xilinx.com:interface:aximm:1.0 M_AXI AWVALID";
  attribute X_INTERFACE_INFO of M_AXI_BREADY : signal is "xilinx.com:interface:aximm:1.0 M_AXI BREADY";
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_PARAMETER of M_AXI_BREADY : signal is "XIL_INTERFACENAME M_AXI, DATA_WIDTH 32, PROTOCOL AXI4LITE, FREQ_HZ 81247969, ID_WIDTH 0, ADDR_WIDTH 32, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE WRITE_ONLY, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 0, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 0, SUPPORTS_NARROW_BURST 0, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1, MAX_BURST_LENGTH 1, PHASE 0, CLK_DOMAIN design_1_mig_7series_0_0_ui_clk, NUM_READ_THREADS 1, NUM_WRITE_THREADS 1, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of M_AXI_BVALID : signal is "xilinx.com:interface:aximm:1.0 M_AXI BVALID";
  attribute X_INTERFACE_INFO of M_AXI_WREADY : signal is "xilinx.com:interface:aximm:1.0 M_AXI WREADY";
  attribute X_INTERFACE_INFO of M_AXI_WVALID : signal is "xilinx.com:interface:aximm:1.0 M_AXI WVALID";
  attribute X_INTERFACE_INFO of S_AXI_ARREADY : signal is "xilinx.com:interface:aximm:1.0 S_AXI ARREADY";
  attribute X_INTERFACE_INFO of S_AXI_ARVALID : signal is "xilinx.com:interface:aximm:1.0 S_AXI ARVALID";
  attribute X_INTERFACE_INFO of S_AXI_AWREADY : signal is "xilinx.com:interface:aximm:1.0 S_AXI AWREADY";
  attribute X_INTERFACE_INFO of S_AXI_AWVALID : signal is "xilinx.com:interface:aximm:1.0 S_AXI AWVALID";
  attribute X_INTERFACE_INFO of S_AXI_BREADY : signal is "xilinx.com:interface:aximm:1.0 S_AXI BREADY";
  attribute X_INTERFACE_INFO of S_AXI_BVALID : signal is "xilinx.com:interface:aximm:1.0 S_AXI BVALID";
  attribute X_INTERFACE_INFO of S_AXI_RREADY : signal is "xilinx.com:interface:aximm:1.0 S_AXI RREADY";
  attribute X_INTERFACE_PARAMETER of S_AXI_RREADY : signal is "XIL_INTERFACENAME S_AXI, DATA_WIDTH 32, PROTOCOL AXI4LITE, FREQ_HZ 81247969, ID_WIDTH 0, ADDR_WIDTH 32, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE READ_WRITE, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 0, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1, SUPPORTS_NARROW_BURST 0, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1, MAX_BURST_LENGTH 1, PHASE 0, CLK_DOMAIN design_1_mig_7series_0_0_ui_clk, NUM_READ_THREADS 1, NUM_WRITE_THREADS 1, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of S_AXI_RVALID : signal is "xilinx.com:interface:aximm:1.0 S_AXI RVALID";
  attribute X_INTERFACE_INFO of S_AXI_WREADY : signal is "xilinx.com:interface:aximm:1.0 S_AXI WREADY";
  attribute X_INTERFACE_INFO of S_AXI_WVALID : signal is "xilinx.com:interface:aximm:1.0 S_AXI WVALID";
  attribute X_INTERFACE_INFO of aclk : signal is "xilinx.com:signal:clock:1.0 aclk CLK";
  attribute X_INTERFACE_PARAMETER of aclk : signal is "XIL_INTERFACENAME aclk, ASSOCIATED_BUSIF M_AXI:S_AXI, ASSOCIATED_RESET aresetn, FREQ_HZ 81247969, PHASE 0, CLK_DOMAIN design_1_mig_7series_0_0_ui_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of aresetn : signal is "xilinx.com:signal:reset:1.0 aresetn RST";
  attribute X_INTERFACE_PARAMETER of aresetn : signal is "XIL_INTERFACENAME aresetn, POLARITY ACTIVE_LOW, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of M_AXI_AWADDR : signal is "xilinx.com:interface:aximm:1.0 M_AXI AWADDR";
  attribute X_INTERFACE_INFO of M_AXI_BRESP : signal is "xilinx.com:interface:aximm:1.0 M_AXI BRESP";
  attribute X_INTERFACE_INFO of M_AXI_WDATA : signal is "xilinx.com:interface:aximm:1.0 M_AXI WDATA";
  attribute X_INTERFACE_INFO of M_AXI_WSTRB : signal is "xilinx.com:interface:aximm:1.0 M_AXI WSTRB";
  attribute X_INTERFACE_INFO of S_AXI_ARADDR : signal is "xilinx.com:interface:aximm:1.0 S_AXI ARADDR";
  attribute X_INTERFACE_INFO of S_AXI_AWADDR : signal is "xilinx.com:interface:aximm:1.0 S_AXI AWADDR";
  attribute X_INTERFACE_INFO of S_AXI_BRESP : signal is "xilinx.com:interface:aximm:1.0 S_AXI BRESP";
  attribute X_INTERFACE_INFO of S_AXI_RDATA : signal is "xilinx.com:interface:aximm:1.0 S_AXI RDATA";
  attribute X_INTERFACE_INFO of S_AXI_RRESP : signal is "xilinx.com:interface:aximm:1.0 S_AXI RRESP";
  attribute X_INTERFACE_INFO of S_AXI_WDATA : signal is "xilinx.com:interface:aximm:1.0 S_AXI WDATA";
  attribute X_INTERFACE_INFO of S_AXI_WSTRB : signal is "xilinx.com:interface:aximm:1.0 S_AXI WSTRB";
begin
  S_AXI_BRESP(1) <= \<const0>\;
  S_AXI_BRESP(0) <= \<const0>\;
  S_AXI_RDATA(31) <= \<const0>\;
  S_AXI_RDATA(30) <= \<const0>\;
  S_AXI_RDATA(29) <= \<const0>\;
  S_AXI_RDATA(28) <= \<const0>\;
  S_AXI_RDATA(27) <= \<const0>\;
  S_AXI_RDATA(26) <= \<const0>\;
  S_AXI_RDATA(25) <= \<const0>\;
  S_AXI_RDATA(24) <= \<const0>\;
  S_AXI_RDATA(23) <= \<const0>\;
  S_AXI_RDATA(22) <= \<const0>\;
  S_AXI_RDATA(21) <= \<const0>\;
  S_AXI_RDATA(20) <= \<const0>\;
  S_AXI_RDATA(19) <= \<const0>\;
  S_AXI_RDATA(18) <= \<const0>\;
  S_AXI_RDATA(17) <= \<const0>\;
  S_AXI_RDATA(16) <= \<const0>\;
  S_AXI_RDATA(15) <= \<const0>\;
  S_AXI_RDATA(14) <= \<const0>\;
  S_AXI_RDATA(13) <= \<const0>\;
  S_AXI_RDATA(12) <= \<const0>\;
  S_AXI_RDATA(11) <= \<const0>\;
  S_AXI_RDATA(10) <= \<const0>\;
  S_AXI_RDATA(9) <= \<const0>\;
  S_AXI_RDATA(8) <= \<const0>\;
  S_AXI_RDATA(7) <= \<const0>\;
  S_AXI_RDATA(6) <= \<const0>\;
  S_AXI_RDATA(5) <= \<const0>\;
  S_AXI_RDATA(4) <= \<const0>\;
  S_AXI_RDATA(3) <= \<const0>\;
  S_AXI_RDATA(2) <= \<const0>\;
  S_AXI_RDATA(1 downto 0) <= \^s_axi_rdata\(1 downto 0);
  S_AXI_RRESP(1) <= \<const0>\;
  S_AXI_RRESP(0) <= \<const0>\;
  \^aresetn\ <= aresetn;
  bram_addra(3) <= \<const0>\;
  bram_addra(2) <= \<const0>\;
  bram_addra(1) <= \<const0>\;
  bram_addra(0) <= \<const0>\;
  bram_addrb(3) <= \<const0>\;
  bram_addrb(2) <= \<const0>\;
  bram_addrb(1) <= \<const0>\;
  bram_addrb(0) <= \^bram_addrb\(0);
  bram_dina(31) <= \^bram_dina\(31);
  bram_dina(30) <= \^bram_dina\(31);
  bram_dina(29) <= \<const0>\;
  bram_dina(28) <= \^bram_dina\(31);
  bram_dina(27) <= \^bram_dina\(31);
  bram_dina(26) <= \^bram_dina\(31);
  bram_dina(25) <= \^bram_dina\(31);
  bram_dina(24) <= \<const0>\;
  bram_dina(23) <= \^bram_dina\(31);
  bram_dina(22) <= \<const0>\;
  bram_dina(21) <= \^bram_dina\(31);
  bram_dina(20) <= \<const0>\;
  bram_dina(19) <= \^bram_dina\(31);
  bram_dina(18) <= \^bram_dina\(31);
  bram_dina(17) <= \<const0>\;
  bram_dina(16) <= \^bram_dina\(31);
  bram_dina(15) <= \^bram_dina\(31);
  bram_dina(14) <= \<const0>\;
  bram_dina(13) <= \^bram_dina\(31);
  bram_dina(12) <= \^bram_dina\(31);
  bram_dina(11) <= \^bram_dina\(31);
  bram_dina(10) <= \^bram_dina\(31);
  bram_dina(9) <= \^bram_dina\(31);
  bram_dina(8) <= \<const0>\;
  bram_dina(7) <= \^bram_dina\(31);
  bram_dina(6) <= \^bram_dina\(31);
  bram_dina(5) <= \^bram_dina\(31);
  bram_dina(4) <= \<const0>\;
  bram_dina(3) <= \^bram_dina\(31);
  bram_dina(2) <= \^bram_dina\(31);
  bram_dina(1) <= \^bram_dina\(31);
  bram_dina(0) <= \^bram_dina\(31);
  bram_dinb(31) <= \^bram_dina\(31);
  bram_dinb(30) <= \^bram_dina\(31);
  bram_dinb(29) <= \<const0>\;
  bram_dinb(28) <= \^bram_dina\(31);
  bram_dinb(27) <= \^bram_dina\(31);
  bram_dinb(26) <= \^bram_dina\(31);
  bram_dinb(25) <= \^bram_dina\(31);
  bram_dinb(24) <= \<const0>\;
  bram_dinb(23) <= \^bram_dina\(31);
  bram_dinb(22) <= \<const0>\;
  bram_dinb(21) <= \^bram_dina\(31);
  bram_dinb(20) <= \<const0>\;
  bram_dinb(19) <= \^bram_dina\(31);
  bram_dinb(18) <= \^bram_dina\(31);
  bram_dinb(17) <= \<const0>\;
  bram_dinb(16) <= \^bram_dina\(31);
  bram_dinb(15) <= \^bram_dina\(31);
  bram_dinb(14) <= \<const0>\;
  bram_dinb(13) <= \^bram_dina\(31);
  bram_dinb(12) <= \^bram_dina\(31);
  bram_dinb(11) <= \^bram_dina\(31);
  bram_dinb(10) <= \^bram_dina\(31);
  bram_dinb(9) <= \^bram_dina\(31);
  bram_dinb(8) <= \<const0>\;
  bram_dinb(7) <= \^bram_dina\(31);
  bram_dinb(6) <= \^bram_dina\(31);
  bram_dinb(5) <= \^bram_dina\(31);
  bram_dinb(4) <= \<const0>\;
  bram_dinb(3) <= \^bram_dina\(31);
  bram_dinb(2) <= \^bram_dina\(31);
  bram_dinb(1) <= \^bram_dina\(31);
  bram_dinb(0) <= \^bram_dina\(31);
  bram_ena <= \^bram_dina\(31);
  bram_enb <= \^bram_dina\(31);
  bram_rsta <= \^aresetn\;
  bram_rstb <= \^aresetn\;
  bram_wea(3) <= \^bram_web\(0);
  bram_wea(2) <= \^bram_web\(0);
  bram_wea(1) <= \^bram_web\(0);
  bram_wea(0) <= \^bram_web\(0);
  bram_web(3) <= \^bram_web\(0);
  bram_web(2) <= \^bram_web\(0);
  bram_web(1) <= \^bram_web\(0);
  bram_web(0) <= \^bram_web\(0);
  a_dout(0) <= 'Z';
  a_dout(1) <= 'Z';
  a_dout(2) <= 'Z';
  a_dout(3) <= 'Z';
  a_dout(4) <= 'Z';
  a_dout(5) <= 'Z';
  a_dout(6) <= 'Z';
  a_dout(7) <= 'Z';
  a_dout(8) <= 'Z';
  a_dout(9) <= 'Z';
  a_dout(10) <= 'Z';
  a_dout(11) <= 'Z';
  a_dout(12) <= 'Z';
  a_dout(13) <= 'Z';
  a_dout(14) <= 'Z';
  a_dout(15) <= 'Z';
  a_dout(16) <= 'Z';
  a_dout(17) <= 'Z';
  a_dout(18) <= 'Z';
  a_dout(19) <= 'Z';
  a_dout(20) <= 'Z';
  a_dout(21) <= 'Z';
  a_dout(22) <= 'Z';
  a_dout(23) <= 'Z';
  a_dout(24) <= 'Z';
  a_dout(25) <= 'Z';
  a_dout(26) <= 'Z';
  a_dout(27) <= 'Z';
  a_dout(28) <= 'Z';
  a_dout(29) <= 'Z';
  a_dout(30) <= 'Z';
  a_dout(31) <= 'Z';
  b_dout(0) <= 'Z';
  b_dout(1) <= 'Z';
  b_dout(2) <= 'Z';
  b_dout(3) <= 'Z';
  b_dout(4) <= 'Z';
  b_dout(5) <= 'Z';
  b_dout(6) <= 'Z';
  b_dout(7) <= 'Z';
  b_dout(8) <= 'Z';
  b_dout(9) <= 'Z';
  b_dout(10) <= 'Z';
  b_dout(11) <= 'Z';
  b_dout(12) <= 'Z';
  b_dout(13) <= 'Z';
  b_dout(14) <= 'Z';
  b_dout(15) <= 'Z';
  b_dout(16) <= 'Z';
  b_dout(17) <= 'Z';
  b_dout(18) <= 'Z';
  b_dout(19) <= 'Z';
  b_dout(20) <= 'Z';
  b_dout(21) <= 'Z';
  b_dout(22) <= 'Z';
  b_dout(23) <= 'Z';
  b_dout(24) <= 'Z';
  b_dout(25) <= 'Z';
  b_dout(26) <= 'Z';
  b_dout(27) <= 'Z';
  b_dout(28) <= 'Z';
  b_dout(29) <= 'Z';
  b_dout(30) <= 'Z';
  b_dout(31) <= 'Z';
GND: unisim.vcomponents.GND
     port map (
      G => \<const0>\
    );
inst: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mem_controller
     port map (
      M_AXI_AWADDR(31 downto 0) => M_AXI_AWADDR(31 downto 0),
      M_AXI_AWREADY => M_AXI_AWREADY,
      M_AXI_AWVALID => M_AXI_AWVALID,
      M_AXI_BREADY => M_AXI_BREADY,
      M_AXI_BRESP(1 downto 0) => M_AXI_BRESP(1 downto 0),
      M_AXI_BVALID => M_AXI_BVALID,
      M_AXI_WDATA(31 downto 0) => M_AXI_WDATA(31 downto 0),
      M_AXI_WREADY => M_AXI_WREADY,
      M_AXI_WSTRB(3 downto 0) => M_AXI_WSTRB(3 downto 0),
      M_AXI_WVALID => M_AXI_WVALID,
      S_AXI_ARADDR(31 downto 0) => S_AXI_ARADDR(31 downto 0),
      S_AXI_ARREADY_reg => S_AXI_ARREADY,
      S_AXI_ARVALID => S_AXI_ARVALID,
      S_AXI_AWADDR(31 downto 0) => S_AXI_AWADDR(31 downto 0),
      S_AXI_AWREADY_reg => S_AXI_AWREADY,
      S_AXI_AWVALID => S_AXI_AWVALID,
      S_AXI_BREADY => S_AXI_BREADY,
      S_AXI_BVALID_reg => S_AXI_BVALID,
      S_AXI_RDATA(1 downto 0) => \^s_axi_rdata\(1 downto 0),
      S_AXI_RREADY => S_AXI_RREADY,
      S_AXI_RVALID_reg => S_AXI_RVALID,
      S_AXI_WDATA(1 downto 0) => S_AXI_WDATA(1 downto 0),
      S_AXI_WREADY_reg => S_AXI_WREADY,
      S_AXI_WVALID => S_AXI_WVALID,
      aclk => aclk,
      aresetn => \^aresetn\,
      bram_addr(31 downto 0) => bram_addr(31 downto 0),
      bram_addrb(0) => \^bram_addrb\(0),
      bram_din(31 downto 0) => bram_din(31 downto 0),
      bram_dina(0) => \^bram_dina\(31),
      bram_dout(31 downto 0) => bram_dout(31 downto 0),
      bram_en => bram_en,
      bram_we(3 downto 0) => bram_we(3 downto 0),
      bram_web(0) => \^bram_web\(0)
    );
end STRUCTURE;
