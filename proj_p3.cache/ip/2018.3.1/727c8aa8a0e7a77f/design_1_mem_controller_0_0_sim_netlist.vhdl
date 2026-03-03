-- Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2018.3.1 (win64) Build 2489853 Tue Mar 26 04:20:25 MDT 2019
-- Date        : Sun Mar  1 22:34:09 2026
-- Host        : DESKTOP-CDKE44D running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
--               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ design_1_mem_controller_0_0_sim_netlist.vhdl
-- Design      : design_1_mem_controller_0_0
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
    M_AXI_AWVALID_reg_0 : out STD_LOGIC;
    M_AXI_WVALID_reg_0 : out STD_LOGIC;
    M_AXI_BREADY : out STD_LOGIC;
    aclk : in STD_LOGIC;
    M_AXI_WVALID_reg_1 : in STD_LOGIC;
    M_AXI_AWREADY : in STD_LOGIC;
    M_AXI_WREADY : in STD_LOGIC;
    M_AXI_BVALID : in STD_LOGIC
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master is
  signal \FSM_onehot_state[0]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_state[1]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_state[1]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_onehot_state[2]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_state_reg_n_0_[0]\ : STD_LOGIC;
  signal \FSM_onehot_state_reg_n_0_[1]\ : STD_LOGIC;
  signal \FSM_onehot_state_reg_n_0_[2]\ : STD_LOGIC;
  signal M_AXI_AWVALID_i_1_n_0 : STD_LOGIC;
  signal \^m_axi_awvalid_reg_0\ : STD_LOGIC;
  signal \^m_axi_bready\ : STD_LOGIC;
  signal M_AXI_BREADY_i_1_n_0 : STD_LOGIC;
  signal M_AXI_BREADY_i_2_n_0 : STD_LOGIC;
  signal \M_AXI_WVALID1__0\ : STD_LOGIC;
  signal M_AXI_WVALID_i_1_n_0 : STD_LOGIC;
  signal \^m_axi_wvalid_reg_0\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \FSM_onehot_state[1]_i_1\ : label is "soft_lutpair1";
  attribute FSM_ENCODED_STATES : string;
  attribute FSM_ENCODED_STATES of \FSM_onehot_state_reg[0]\ : label is "ST_AW_W:010,ST_B:100,ST_DONE:001,";
  attribute FSM_ENCODED_STATES of \FSM_onehot_state_reg[1]\ : label is "ST_AW_W:010,ST_B:100,ST_DONE:001,";
  attribute FSM_ENCODED_STATES of \FSM_onehot_state_reg[2]\ : label is "ST_AW_W:010,ST_B:100,ST_DONE:001,";
  attribute SOFT_HLUTNM of M_AXI_AWVALID_i_1 : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of M_AXI_BREADY_i_2 : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of M_AXI_WVALID_i_1 : label is "soft_lutpair0";
begin
  M_AXI_AWVALID_reg_0 <= \^m_axi_awvalid_reg_0\;
  M_AXI_BREADY <= \^m_axi_bready\;
  M_AXI_WVALID_reg_0 <= \^m_axi_wvalid_reg_0\;
\FSM_onehot_state[0]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \FSM_onehot_state_reg_n_0_[2]\,
      I1 => \FSM_onehot_state[1]_i_2_n_0\,
      I2 => \FSM_onehot_state_reg_n_0_[0]\,
      O => \FSM_onehot_state[0]_i_1_n_0\
    );
\FSM_onehot_state[1]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \FSM_onehot_state_reg_n_0_[0]\,
      I1 => \FSM_onehot_state[1]_i_2_n_0\,
      I2 => \FSM_onehot_state_reg_n_0_[1]\,
      O => \FSM_onehot_state[1]_i_1_n_0\
    );
\FSM_onehot_state[1]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFF800080008000"
    )
        port map (
      I0 => \FSM_onehot_state_reg_n_0_[1]\,
      I1 => M_AXI_AWREADY,
      I2 => \^m_axi_awvalid_reg_0\,
      I3 => \M_AXI_WVALID1__0\,
      I4 => \FSM_onehot_state_reg_n_0_[2]\,
      I5 => M_AXI_BVALID,
      O => \FSM_onehot_state[1]_i_2_n_0\
    );
\FSM_onehot_state[2]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"AAAA8000FFFF8000"
    )
        port map (
      I0 => \FSM_onehot_state_reg_n_0_[1]\,
      I1 => M_AXI_AWREADY,
      I2 => \^m_axi_awvalid_reg_0\,
      I3 => \M_AXI_WVALID1__0\,
      I4 => \FSM_onehot_state_reg_n_0_[2]\,
      I5 => M_AXI_BVALID,
      O => \FSM_onehot_state[2]_i_1_n_0\
    );
\FSM_onehot_state[2]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \^m_axi_wvalid_reg_0\,
      I1 => M_AXI_WREADY,
      O => \M_AXI_WVALID1__0\
    );
\FSM_onehot_state_reg[0]\: unisim.vcomponents.FDPE
    generic map(
      INIT => '1'
    )
        port map (
      C => aclk,
      CE => '1',
      D => \FSM_onehot_state[0]_i_1_n_0\,
      PRE => M_AXI_WVALID_reg_1,
      Q => \FSM_onehot_state_reg_n_0_[0]\
    );
\FSM_onehot_state_reg[1]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => aclk,
      CE => '1',
      CLR => M_AXI_WVALID_reg_1,
      D => \FSM_onehot_state[1]_i_1_n_0\,
      Q => \FSM_onehot_state_reg_n_0_[1]\
    );
\FSM_onehot_state_reg[2]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => aclk,
      CE => '1',
      CLR => M_AXI_WVALID_reg_1,
      D => \FSM_onehot_state[2]_i_1_n_0\,
      Q => \FSM_onehot_state_reg_n_0_[2]\
    );
M_AXI_AWVALID_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"6E"
    )
        port map (
      I0 => \FSM_onehot_state_reg_n_0_[1]\,
      I1 => \^m_axi_awvalid_reg_0\,
      I2 => M_AXI_AWREADY,
      O => M_AXI_AWVALID_i_1_n_0
    );
M_AXI_AWVALID_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => M_AXI_WVALID_reg_1,
      D => M_AXI_AWVALID_i_1_n_0,
      Q => \^m_axi_awvalid_reg_0\
    );
M_AXI_BREADY_i_1: unisim.vcomponents.LUT4
    generic map(
      INIT => X"BFAA"
    )
        port map (
      I0 => M_AXI_BREADY_i_2_n_0,
      I1 => M_AXI_BVALID,
      I2 => \FSM_onehot_state_reg_n_0_[2]\,
      I3 => \^m_axi_bready\,
      O => M_AXI_BREADY_i_1_n_0
    );
M_AXI_BREADY_i_2: unisim.vcomponents.LUT5
    generic map(
      INIT => X"80000000"
    )
        port map (
      I0 => \FSM_onehot_state_reg_n_0_[1]\,
      I1 => \^m_axi_wvalid_reg_0\,
      I2 => M_AXI_WREADY,
      I3 => \^m_axi_awvalid_reg_0\,
      I4 => M_AXI_AWREADY,
      O => M_AXI_BREADY_i_2_n_0
    );
M_AXI_BREADY_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => M_AXI_WVALID_reg_1,
      D => M_AXI_BREADY_i_1_n_0,
      Q => \^m_axi_bready\
    );
M_AXI_WVALID_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"6E"
    )
        port map (
      I0 => \FSM_onehot_state_reg_n_0_[1]\,
      I1 => \^m_axi_wvalid_reg_0\,
      I2 => M_AXI_WREADY,
      O => M_AXI_WVALID_i_1_n_0
    );
M_AXI_WVALID_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => M_AXI_WVALID_reg_1,
      D => M_AXI_WVALID_i_1_n_0,
      Q => \^m_axi_wvalid_reg_0\
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_slave is
  port (
    S_AXI_AWREADY : out STD_LOGIC;
    aresetn_0 : out STD_LOGIC;
    S_AXI_ARREADY : out STD_LOGIC;
    S_AXI_WREADY_reg_0 : out STD_LOGIC;
    S_AXI_BVALID_reg_0 : out STD_LOGIC;
    S_AXI_RVALID : out STD_LOGIC;
    S_AXI_RDATA : out STD_LOGIC_VECTOR ( 0 to 0 );
    S_AXI_RRESP : out STD_LOGIC_VECTOR ( 0 to 0 );
    aclk : in STD_LOGIC;
    aresetn : in STD_LOGIC;
    S_AXI_WVALID : in STD_LOGIC;
    S_AXI_AWVALID : in STD_LOGIC;
    S_AXI_BREADY : in STD_LOGIC;
    S_AXI_ARVALID : in STD_LOGIC;
    S_AXI_RREADY : in STD_LOGIC;
    S_AXI_ARADDR : in STD_LOGIC_VECTOR ( 31 downto 0 )
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_slave;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_slave is
  signal \^s_axi_arready\ : STD_LOGIC;
  signal S_AXI_ARREADY_i_1_n_0 : STD_LOGIC;
  signal \^s_axi_awready\ : STD_LOGIC;
  signal S_AXI_BVALID_i_1_n_0 : STD_LOGIC;
  signal \^s_axi_bvalid_reg_0\ : STD_LOGIC;
  signal \^s_axi_rdata\ : STD_LOGIC_VECTOR ( 0 to 0 );
  signal \S_AXI_RDATA[31]_i_1_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[31]_i_2_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[31]_i_3_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[31]_i_4_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[31]_i_5_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[31]_i_6_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[31]_i_7_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[31]_i_8_n_0\ : STD_LOGIC;
  signal \S_AXI_RDATA[31]_i_9_n_0\ : STD_LOGIC;
  signal \^s_axi_rresp\ : STD_LOGIC_VECTOR ( 0 to 0 );
  signal \S_AXI_RRESP[1]_i_1_n_0\ : STD_LOGIC;
  signal \^s_axi_rvalid\ : STD_LOGIC;
  signal S_AXI_RVALID00_out : STD_LOGIC;
  signal S_AXI_RVALID_i_1_n_0 : STD_LOGIC;
  signal S_AXI_WREADY_i_1_n_0 : STD_LOGIC;
  signal \^s_axi_wready_reg_0\ : STD_LOGIC;
  signal araddr_latched : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal \^aresetn_0\ : STD_LOGIC;
  signal aw_hs_done : STD_LOGIC;
  signal aw_hs_done_i_1_n_0 : STD_LOGIC;
  signal p_0_in : STD_LOGIC;
  signal w_hs_done : STD_LOGIC;
  signal w_hs_done_i_1_n_0 : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \S_AXI_RVALID0_inferred__0/i_\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of S_AXI_RVALID_i_1 : label is "soft_lutpair2";
begin
  S_AXI_ARREADY <= \^s_axi_arready\;
  S_AXI_AWREADY <= \^s_axi_awready\;
  S_AXI_BVALID_reg_0 <= \^s_axi_bvalid_reg_0\;
  S_AXI_RDATA(0) <= \^s_axi_rdata\(0);
  S_AXI_RRESP(0) <= \^s_axi_rresp\(0);
  S_AXI_RVALID <= \^s_axi_rvalid\;
  S_AXI_WREADY_reg_0 <= \^s_axi_wready_reg_0\;
  aresetn_0 <= \^aresetn_0\;
S_AXI_ARREADY_i_1: unisim.vcomponents.LUT2
    generic map(
      INIT => X"7"
    )
        port map (
      I0 => \^s_axi_wready_reg_0\,
      I1 => S_AXI_RREADY,
      O => S_AXI_ARREADY_i_1_n_0
    );
S_AXI_ARREADY_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => S_AXI_ARREADY_i_1_n_0,
      Q => \^s_axi_arready\
    );
S_AXI_AWREADY_i_1: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => S_AXI_RREADY,
      I1 => \^s_axi_wready_reg_0\,
      O => p_0_in
    );
S_AXI_AWREADY_i_2: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => aresetn,
      O => \^aresetn_0\
    );
S_AXI_AWREADY_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => p_0_in,
      Q => \^s_axi_awready\
    );
S_AXI_BVALID_i_1: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00F08080"
    )
        port map (
      I0 => aw_hs_done,
      I1 => w_hs_done,
      I2 => aresetn,
      I3 => S_AXI_BREADY,
      I4 => \^s_axi_bvalid_reg_0\,
      O => S_AXI_BVALID_i_1_n_0
    );
S_AXI_BVALID_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => S_AXI_BVALID_i_1_n_0,
      Q => \^s_axi_bvalid_reg_0\,
      R => '0'
    );
\S_AXI_RDATA[31]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFEFFFFFFFE0000"
    )
        port map (
      I0 => \S_AXI_RDATA[31]_i_2_n_0\,
      I1 => araddr_latched(1),
      I2 => \S_AXI_RDATA[31]_i_3_n_0\,
      I3 => \S_AXI_RDATA[31]_i_4_n_0\,
      I4 => S_AXI_RVALID00_out,
      I5 => \^s_axi_rdata\(0),
      O => \S_AXI_RDATA[31]_i_1_n_0\
    );
\S_AXI_RDATA[31]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"F8"
    )
        port map (
      I0 => araddr_latched(2),
      I1 => araddr_latched(3),
      I2 => araddr_latched(0),
      O => \S_AXI_RDATA[31]_i_2_n_0\
    );
\S_AXI_RDATA[31]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFFFFFFFFFE"
    )
        port map (
      I0 => \S_AXI_RDATA[31]_i_5_n_0\,
      I1 => \S_AXI_RDATA[31]_i_6_n_0\,
      I2 => \S_AXI_RDATA[31]_i_7_n_0\,
      I3 => \S_AXI_RDATA[31]_i_8_n_0\,
      I4 => araddr_latched(19),
      I5 => araddr_latched(18),
      O => \S_AXI_RDATA[31]_i_3_n_0\
    );
\S_AXI_RDATA[31]_i_4\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFFFFFE"
    )
        port map (
      I0 => \S_AXI_RDATA[31]_i_9_n_0\,
      I1 => araddr_latched(9),
      I2 => araddr_latched(8),
      I3 => araddr_latched(7),
      I4 => araddr_latched(6),
      O => \S_AXI_RDATA[31]_i_4_n_0\
    );
\S_AXI_RDATA[31]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => araddr_latched(10),
      I1 => araddr_latched(11),
      I2 => araddr_latched(12),
      I3 => araddr_latched(13),
      O => \S_AXI_RDATA[31]_i_5_n_0\
    );
\S_AXI_RDATA[31]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFEF"
    )
        port map (
      I0 => araddr_latched(28),
      I1 => araddr_latched(29),
      I2 => araddr_latched(31),
      I3 => araddr_latched(30),
      O => \S_AXI_RDATA[31]_i_6_n_0\
    );
\S_AXI_RDATA[31]_i_7\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => araddr_latched(24),
      I1 => araddr_latched(25),
      I2 => araddr_latched(26),
      I3 => araddr_latched(27),
      O => \S_AXI_RDATA[31]_i_7_n_0\
    );
\S_AXI_RDATA[31]_i_8\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => araddr_latched(20),
      I1 => araddr_latched(21),
      I2 => araddr_latched(22),
      I3 => araddr_latched(23),
      O => \S_AXI_RDATA[31]_i_8_n_0\
    );
\S_AXI_RDATA[31]_i_9\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFFFFFFFFFE"
    )
        port map (
      I0 => araddr_latched(17),
      I1 => araddr_latched(16),
      I2 => araddr_latched(15),
      I3 => araddr_latched(14),
      I4 => araddr_latched(4),
      I5 => araddr_latched(5),
      O => \S_AXI_RDATA[31]_i_9_n_0\
    );
\S_AXI_RDATA_reg[31]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => \S_AXI_RDATA[31]_i_1_n_0\,
      Q => \^s_axi_rdata\(0)
    );
\S_AXI_RRESP[1]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFEFFFFFFFE0000"
    )
        port map (
      I0 => araddr_latched(1),
      I1 => \S_AXI_RDATA[31]_i_2_n_0\,
      I2 => \S_AXI_RDATA[31]_i_3_n_0\,
      I3 => \S_AXI_RDATA[31]_i_4_n_0\,
      I4 => S_AXI_RVALID00_out,
      I5 => \^s_axi_rresp\(0),
      O => \S_AXI_RRESP[1]_i_1_n_0\
    );
\S_AXI_RRESP_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => \S_AXI_RRESP[1]_i_1_n_0\,
      Q => \^s_axi_rresp\(0)
    );
\S_AXI_RVALID0_inferred__0/i_\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"08"
    )
        port map (
      I0 => \^s_axi_arready\,
      I1 => S_AXI_ARVALID,
      I2 => \^s_axi_rvalid\,
      O => S_AXI_RVALID00_out
    );
S_AXI_RVALID_i_1: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0F88"
    )
        port map (
      I0 => S_AXI_ARVALID,
      I1 => \^s_axi_arready\,
      I2 => S_AXI_RREADY,
      I3 => \^s_axi_rvalid\,
      O => S_AXI_RVALID_i_1_n_0
    );
S_AXI_RVALID_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \^aresetn_0\,
      D => S_AXI_RVALID_i_1_n_0,
      Q => \^s_axi_rvalid\
    );
S_AXI_WREADY_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"2A"
    )
        port map (
      I0 => aresetn,
      I1 => S_AXI_WVALID,
      I2 => \^s_axi_wready_reg_0\,
      O => S_AXI_WREADY_i_1_n_0
    );
S_AXI_WREADY_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => S_AXI_WREADY_i_1_n_0,
      Q => \^s_axi_wready_reg_0\,
      R => '0'
    );
\araddr_latched_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(0),
      Q => araddr_latched(0)
    );
\araddr_latched_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(10),
      Q => araddr_latched(10)
    );
\araddr_latched_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(11),
      Q => araddr_latched(11)
    );
\araddr_latched_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(12),
      Q => araddr_latched(12)
    );
\araddr_latched_reg[13]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(13),
      Q => araddr_latched(13)
    );
\araddr_latched_reg[14]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(14),
      Q => araddr_latched(14)
    );
\araddr_latched_reg[15]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(15),
      Q => araddr_latched(15)
    );
\araddr_latched_reg[16]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(16),
      Q => araddr_latched(16)
    );
\araddr_latched_reg[17]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(17),
      Q => araddr_latched(17)
    );
\araddr_latched_reg[18]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(18),
      Q => araddr_latched(18)
    );
\araddr_latched_reg[19]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(19),
      Q => araddr_latched(19)
    );
\araddr_latched_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(1),
      Q => araddr_latched(1)
    );
\araddr_latched_reg[20]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(20),
      Q => araddr_latched(20)
    );
\araddr_latched_reg[21]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(21),
      Q => araddr_latched(21)
    );
\araddr_latched_reg[22]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(22),
      Q => araddr_latched(22)
    );
\araddr_latched_reg[23]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(23),
      Q => araddr_latched(23)
    );
\araddr_latched_reg[24]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(24),
      Q => araddr_latched(24)
    );
\araddr_latched_reg[25]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(25),
      Q => araddr_latched(25)
    );
\araddr_latched_reg[26]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(26),
      Q => araddr_latched(26)
    );
\araddr_latched_reg[27]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(27),
      Q => araddr_latched(27)
    );
\araddr_latched_reg[28]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(28),
      Q => araddr_latched(28)
    );
\araddr_latched_reg[29]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(29),
      Q => araddr_latched(29)
    );
\araddr_latched_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(2),
      Q => araddr_latched(2)
    );
\araddr_latched_reg[30]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(30),
      Q => araddr_latched(30)
    );
\araddr_latched_reg[31]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(31),
      Q => araddr_latched(31)
    );
\araddr_latched_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(3),
      Q => araddr_latched(3)
    );
\araddr_latched_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(4),
      Q => araddr_latched(4)
    );
\araddr_latched_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(5),
      Q => araddr_latched(5)
    );
\araddr_latched_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(6),
      Q => araddr_latched(6)
    );
\araddr_latched_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(7),
      Q => araddr_latched(7)
    );
\araddr_latched_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(8),
      Q => araddr_latched(8)
    );
\araddr_latched_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => S_AXI_ARREADY_i_1_n_0,
      CLR => \^aresetn_0\,
      D => S_AXI_ARADDR(9),
      Q => araddr_latched(9)
    );
aw_hs_done_i_1: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000EA00EA00EA00"
    )
        port map (
      I0 => aw_hs_done,
      I1 => \^s_axi_awready\,
      I2 => S_AXI_AWVALID,
      I3 => aresetn,
      I4 => S_AXI_BREADY,
      I5 => \^s_axi_bvalid_reg_0\,
      O => aw_hs_done_i_1_n_0
    );
aw_hs_done_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => aw_hs_done_i_1_n_0,
      Q => aw_hs_done,
      R => '0'
    );
w_hs_done_i_1: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000EA00EA00EA00"
    )
        port map (
      I0 => w_hs_done,
      I1 => \^s_axi_wready_reg_0\,
      I2 => S_AXI_WVALID,
      I3 => aresetn,
      I4 => S_AXI_BREADY,
      I5 => \^s_axi_bvalid_reg_0\,
      O => w_hs_done_i_1_n_0
    );
w_hs_done_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => w_hs_done_i_1_n_0,
      Q => w_hs_done,
      R => '0'
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mem_controller is
  port (
    S_AXI_WREADY_reg : out STD_LOGIC;
    M_AXI_AWVALID_reg : out STD_LOGIC;
    S_AXI_AWREADY : out STD_LOGIC;
    S_AXI_ARREADY : out STD_LOGIC;
    S_AXI_RVALID : out STD_LOGIC;
    M_AXI_WVALID_reg : out STD_LOGIC;
    S_AXI_BVALID_reg : out STD_LOGIC;
    M_AXI_BREADY : out STD_LOGIC;
    S_AXI_RRESP : out STD_LOGIC_VECTOR ( 0 to 0 );
    S_AXI_RDATA : out STD_LOGIC_VECTOR ( 0 to 0 );
    aresetn : in STD_LOGIC;
    S_AXI_WVALID : in STD_LOGIC;
    M_AXI_AWREADY : in STD_LOGIC;
    M_AXI_BVALID : in STD_LOGIC;
    aclk : in STD_LOGIC;
    S_AXI_ARADDR : in STD_LOGIC_VECTOR ( 31 downto 0 );
    S_AXI_ARVALID : in STD_LOGIC;
    S_AXI_RREADY : in STD_LOGIC;
    M_AXI_WREADY : in STD_LOGIC;
    S_AXI_AWVALID : in STD_LOGIC;
    S_AXI_BREADY : in STD_LOGIC
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mem_controller;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mem_controller is
  signal slave1_n_1 : STD_LOGIC;
begin
master1: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_master
     port map (
      M_AXI_AWREADY => M_AXI_AWREADY,
      M_AXI_AWVALID_reg_0 => M_AXI_AWVALID_reg,
      M_AXI_BREADY => M_AXI_BREADY,
      M_AXI_BVALID => M_AXI_BVALID,
      M_AXI_WREADY => M_AXI_WREADY,
      M_AXI_WVALID_reg_0 => M_AXI_WVALID_reg,
      M_AXI_WVALID_reg_1 => slave1_n_1,
      aclk => aclk
    );
slave1: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_lite_slave
     port map (
      S_AXI_ARADDR(31 downto 0) => S_AXI_ARADDR(31 downto 0),
      S_AXI_ARREADY => S_AXI_ARREADY,
      S_AXI_ARVALID => S_AXI_ARVALID,
      S_AXI_AWREADY => S_AXI_AWREADY,
      S_AXI_AWVALID => S_AXI_AWVALID,
      S_AXI_BREADY => S_AXI_BREADY,
      S_AXI_BVALID_reg_0 => S_AXI_BVALID_reg,
      S_AXI_RDATA(0) => S_AXI_RDATA(0),
      S_AXI_RREADY => S_AXI_RREADY,
      S_AXI_RRESP(0) => S_AXI_RRESP(0),
      S_AXI_RVALID => S_AXI_RVALID,
      S_AXI_WREADY_reg_0 => S_AXI_WREADY_reg,
      S_AXI_WVALID => S_AXI_WVALID,
      aclk => aclk,
      aresetn => aresetn,
      aresetn_0 => slave1_n_1
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
  attribute CHECK_LICENSE_TYPE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "design_1_mem_controller_0_0,mem_controller,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "package_project";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "mem_controller,Vivado 2018.3.1";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  signal \<const0>\ : STD_LOGIC;
  signal \<const1>\ : STD_LOGIC;
  signal \^s_axi_rdata\ : STD_LOGIC_VECTOR ( 29 to 29 );
  signal \^s_axi_rresp\ : STD_LOGIC_VECTOR ( 1 to 1 );
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_INFO of M_AXI_AWREADY : signal is "xilinx.com:interface:aximm:1.0 M_AXI AWREADY";
  attribute X_INTERFACE_INFO of M_AXI_AWVALID : signal is "xilinx.com:interface:aximm:1.0 M_AXI AWVALID";
  attribute X_INTERFACE_INFO of M_AXI_BREADY : signal is "xilinx.com:interface:aximm:1.0 M_AXI BREADY";
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_PARAMETER of M_AXI_BREADY : signal is "XIL_INTERFACENAME M_AXI, DATA_WIDTH 32, PROTOCOL AXI4LITE, FREQ_HZ 100000000, ID_WIDTH 0, ADDR_WIDTH 32, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE WRITE_ONLY, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 0, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 0, SUPPORTS_NARROW_BURST 0, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1, MAX_BURST_LENGTH 1, PHASE 0.000, CLK_DOMAIN design_1_clk, NUM_READ_THREADS 1, NUM_WRITE_THREADS 1, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0, INSERT_VIP 0";
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
  attribute X_INTERFACE_PARAMETER of S_AXI_RREADY : signal is "XIL_INTERFACENAME S_AXI, DATA_WIDTH 32, PROTOCOL AXI4LITE, FREQ_HZ 100000000, ID_WIDTH 0, ADDR_WIDTH 32, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE READ_WRITE, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 0, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1, SUPPORTS_NARROW_BURST 0, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1, MAX_BURST_LENGTH 1, PHASE 0.000, CLK_DOMAIN design_1_clk, NUM_READ_THREADS 1, NUM_WRITE_THREADS 1, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of S_AXI_RVALID : signal is "xilinx.com:interface:aximm:1.0 S_AXI RVALID";
  attribute X_INTERFACE_INFO of S_AXI_WREADY : signal is "xilinx.com:interface:aximm:1.0 S_AXI WREADY";
  attribute X_INTERFACE_INFO of S_AXI_WVALID : signal is "xilinx.com:interface:aximm:1.0 S_AXI WVALID";
  attribute X_INTERFACE_INFO of aclk : signal is "xilinx.com:signal:clock:1.0 aclk CLK";
  attribute X_INTERFACE_PARAMETER of aclk : signal is "XIL_INTERFACENAME aclk, ASSOCIATED_BUSIF M_AXI:S_AXI, ASSOCIATED_RESET aresetn, FREQ_HZ 100000000, PHASE 0.000, CLK_DOMAIN design_1_clk, INSERT_VIP 0";
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
  M_AXI_AWADDR(31) <= \<const0>\;
  M_AXI_AWADDR(30) <= \<const0>\;
  M_AXI_AWADDR(29) <= \<const0>\;
  M_AXI_AWADDR(28) <= \<const0>\;
  M_AXI_AWADDR(27) <= \<const0>\;
  M_AXI_AWADDR(26) <= \<const0>\;
  M_AXI_AWADDR(25) <= \<const0>\;
  M_AXI_AWADDR(24) <= \<const0>\;
  M_AXI_AWADDR(23) <= \<const0>\;
  M_AXI_AWADDR(22) <= \<const0>\;
  M_AXI_AWADDR(21) <= \<const0>\;
  M_AXI_AWADDR(20) <= \<const0>\;
  M_AXI_AWADDR(19) <= \<const0>\;
  M_AXI_AWADDR(18) <= \<const0>\;
  M_AXI_AWADDR(17) <= \<const0>\;
  M_AXI_AWADDR(16) <= \<const0>\;
  M_AXI_AWADDR(15) <= \<const0>\;
  M_AXI_AWADDR(14) <= \<const0>\;
  M_AXI_AWADDR(13) <= \<const0>\;
  M_AXI_AWADDR(12) <= \<const0>\;
  M_AXI_AWADDR(11) <= \<const0>\;
  M_AXI_AWADDR(10) <= \<const0>\;
  M_AXI_AWADDR(9) <= \<const0>\;
  M_AXI_AWADDR(8) <= \<const0>\;
  M_AXI_AWADDR(7) <= \<const0>\;
  M_AXI_AWADDR(6) <= \<const0>\;
  M_AXI_AWADDR(5) <= \<const0>\;
  M_AXI_AWADDR(4) <= \<const0>\;
  M_AXI_AWADDR(3) <= \<const0>\;
  M_AXI_AWADDR(2) <= \<const0>\;
  M_AXI_AWADDR(1) <= \<const0>\;
  M_AXI_AWADDR(0) <= \<const0>\;
  M_AXI_WDATA(31) <= \<const0>\;
  M_AXI_WDATA(30) <= \<const0>\;
  M_AXI_WDATA(29) <= \<const0>\;
  M_AXI_WDATA(28) <= \<const0>\;
  M_AXI_WDATA(27) <= \<const0>\;
  M_AXI_WDATA(26) <= \<const0>\;
  M_AXI_WDATA(25) <= \<const0>\;
  M_AXI_WDATA(24) <= \<const0>\;
  M_AXI_WDATA(23) <= \<const0>\;
  M_AXI_WDATA(22) <= \<const0>\;
  M_AXI_WDATA(21) <= \<const0>\;
  M_AXI_WDATA(20) <= \<const0>\;
  M_AXI_WDATA(19) <= \<const0>\;
  M_AXI_WDATA(18) <= \<const0>\;
  M_AXI_WDATA(17) <= \<const0>\;
  M_AXI_WDATA(16) <= \<const0>\;
  M_AXI_WDATA(15) <= \<const0>\;
  M_AXI_WDATA(14) <= \<const0>\;
  M_AXI_WDATA(13) <= \<const0>\;
  M_AXI_WDATA(12) <= \<const0>\;
  M_AXI_WDATA(11) <= \<const0>\;
  M_AXI_WDATA(10) <= \<const0>\;
  M_AXI_WDATA(9) <= \<const0>\;
  M_AXI_WDATA(8) <= \<const0>\;
  M_AXI_WDATA(7) <= \<const0>\;
  M_AXI_WDATA(6) <= \<const0>\;
  M_AXI_WDATA(5) <= \<const0>\;
  M_AXI_WDATA(4) <= \<const0>\;
  M_AXI_WDATA(3) <= \<const0>\;
  M_AXI_WDATA(2) <= \<const0>\;
  M_AXI_WDATA(1) <= \<const0>\;
  M_AXI_WDATA(0) <= \<const0>\;
  M_AXI_WSTRB(3) <= \<const1>\;
  M_AXI_WSTRB(2) <= \<const1>\;
  M_AXI_WSTRB(1) <= \<const1>\;
  M_AXI_WSTRB(0) <= \<const1>\;
  S_AXI_BRESP(1) <= \<const0>\;
  S_AXI_BRESP(0) <= \<const0>\;
  S_AXI_RDATA(31) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(30) <= \<const0>\;
  S_AXI_RDATA(29) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(28) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(27) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(26) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(25) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(24) <= \<const0>\;
  S_AXI_RDATA(23) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(22) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(21) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(20) <= \<const0>\;
  S_AXI_RDATA(19) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(18) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(17) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(16) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(15) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(14) <= \<const0>\;
  S_AXI_RDATA(13) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(12) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(11) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(10) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(9) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(8) <= \<const0>\;
  S_AXI_RDATA(7) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(6) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(5) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(4) <= \<const0>\;
  S_AXI_RDATA(3) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(2) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(1) <= \^s_axi_rdata\(29);
  S_AXI_RDATA(0) <= \^s_axi_rdata\(29);
  S_AXI_RRESP(1) <= \^s_axi_rresp\(1);
  S_AXI_RRESP(0) <= \<const0>\;
GND: unisim.vcomponents.GND
     port map (
      G => \<const0>\
    );
VCC: unisim.vcomponents.VCC
     port map (
      P => \<const1>\
    );
inst: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mem_controller
     port map (
      M_AXI_AWREADY => M_AXI_AWREADY,
      M_AXI_AWVALID_reg => M_AXI_AWVALID,
      M_AXI_BREADY => M_AXI_BREADY,
      M_AXI_BVALID => M_AXI_BVALID,
      M_AXI_WREADY => M_AXI_WREADY,
      M_AXI_WVALID_reg => M_AXI_WVALID,
      S_AXI_ARADDR(31 downto 0) => S_AXI_ARADDR(31 downto 0),
      S_AXI_ARREADY => S_AXI_ARREADY,
      S_AXI_ARVALID => S_AXI_ARVALID,
      S_AXI_AWREADY => S_AXI_AWREADY,
      S_AXI_AWVALID => S_AXI_AWVALID,
      S_AXI_BREADY => S_AXI_BREADY,
      S_AXI_BVALID_reg => S_AXI_BVALID,
      S_AXI_RDATA(0) => \^s_axi_rdata\(29),
      S_AXI_RREADY => S_AXI_RREADY,
      S_AXI_RRESP(0) => \^s_axi_rresp\(1),
      S_AXI_RVALID => S_AXI_RVALID,
      S_AXI_WREADY_reg => S_AXI_WREADY,
      S_AXI_WVALID => S_AXI_WVALID,
      aclk => aclk,
      aresetn => aresetn
    );
end STRUCTURE;
