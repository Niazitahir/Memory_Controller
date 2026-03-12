-- Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2018.3.1 (win64) Build 2489853 Tue Mar 26 04:20:25 MDT 2019
-- Date        : Tue Mar 10 14:04:57 2026
-- Host        : DESKTOP-CDKE44D running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode synth_stub
--               C:/Users/khanm/AppData/Roaming/Xilinx/Vivado/proj/DDR_GOLDEN/DDR_GOLDEN.srcs/sources_1/bd/design_1/ip/design_1_bram_test_0_0/design_1_bram_test_0_0_stub.vhdl
-- Design      : design_1_bram_test_0_0
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7a100tcsg324-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity design_1_bram_test_0_0 is
  Port ( 
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

end design_1_bram_test_0_0;

architecture stub of design_1_bram_test_0_0 is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "clk,reset_n,bram_en,bram_addr[31:0],bram_din[31:0],bram_we[3:0],bram_dout[31:0],bram_ready,tester,test_pass";
attribute X_CORE_INFO : string;
attribute X_CORE_INFO of stub : architecture is "bram_test,Vivado 2018.3.1";
begin
end;
