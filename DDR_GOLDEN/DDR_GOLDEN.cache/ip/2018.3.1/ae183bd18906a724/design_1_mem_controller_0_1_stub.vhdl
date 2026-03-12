-- Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2018.3.1 (win64) Build 2489853 Tue Mar 26 04:20:25 MDT 2019
-- Date        : Wed Mar 11 21:11:40 2026
-- Host        : DESKTOP-CDKE44D running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode synth_stub -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
--               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ design_1_mem_controller_0_1_stub.vhdl
-- Design      : design_1_mem_controller_0_1
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7a100tcsg324-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  Port ( 
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
    M_AXI_BREADY : out STD_LOGIC;
    a_read_val_sig : out STD_LOGIC_VECTOR ( 31 downto 0 );
    b_read_val_sig : out STD_LOGIC_VECTOR ( 31 downto 0 )
  );

end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix;

architecture stub of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "aclk,aresetn,a_en,a_we,a_addr[3:0],a_din[31:0],a_dout[31:0],b_en,b_we,b_addr[3:0],b_din[31:0],b_dout[31:0],bram_ena,bram_wea[3:0],bram_addra[3:0],bram_dina[31:0],bram_douta[31:0],bram_rsta,bram_enb,bram_web[3:0],bram_addrb[3:0],bram_dinb[31:0],bram_doutb[31:0],bram_rstb,bram_en,bram_addr[31:0],bram_din[31:0],bram_we[3:0],bram_dout[31:0],S_AXI_AWADDR[31:0],S_AXI_AWVALID,S_AXI_AWREADY,S_AXI_WDATA[31:0],S_AXI_WSTRB[3:0],S_AXI_WVALID,S_AXI_WREADY,S_AXI_BRESP[1:0],S_AXI_BVALID,S_AXI_BREADY,S_AXI_ARADDR[31:0],S_AXI_ARVALID,S_AXI_ARREADY,S_AXI_RDATA[31:0],S_AXI_RRESP[1:0],S_AXI_RVALID,S_AXI_RREADY,M_AXI_AWADDR[31:0],M_AXI_AWVALID,M_AXI_AWREADY,M_AXI_WDATA[31:0],M_AXI_WSTRB[3:0],M_AXI_WVALID,M_AXI_WREADY,M_AXI_BRESP[1:0],M_AXI_BVALID,M_AXI_BREADY,a_read_val_sig[31:0],b_read_val_sig[31:0]";
attribute X_CORE_INFO : string;
attribute X_CORE_INFO of stub : architecture is "mem_controller,Vivado 2018.3.1";
begin
end;
