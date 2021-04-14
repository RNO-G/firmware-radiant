----------------------------------------------------------------------------------------------
-- 7-Series reset/reconfiguration via ICAPE2.

 

library work;
library IEEE;
use IEEE.std_logic_1164.all;
library UNISIM;
use UNISIM.vcomponents.all;

 

entity iprog_rst is
  port (
    WBSTAR  : in  std_logic_vector(31 downto 0);
    RST     : in  std_logic;
    CLK     : in  std_logic;
    RIP     : out std_logic
  );

end iprog_rst;

 

architecture arch_iprog_rst of iprog_rst is
   constant  CMAX    : integer := 32;
   signal  count     : integer range 0 to CMAX := 0;
   signal  run       : std_logic := '0';
   signal  cs_l      : std_logic := '1';
   signal  r_wx      : std_logic := '1';
   signal  data      : std_logic_vector(31 downto 0);
   signal  d_swapped : std_logic_vector(31 downto 0);

begin
process(CLK)

begin
   if(CLK'event and CLK='1') then
      if(RST = '1')  then
         run      <= '1';
      end if;

      if(run = '0')  then
         RIP      <= '0';
         cs_l     <= '1';
         r_wx     <= '1';
         count    <= 0;
         data     <= (others => '1');
      else
         RIP      <= '1';
 
         if(count /= CMAX)  then
            count <= count + 1;
         end if;
 
         case count is
            when  0 => data <= x"FFFFFFFF";   -- Dummy Word
            when  1 => data <= x"FFFFFFFF";   -- Dummy Word
            when  2 => data <= x"FFFFFFFF";   -- Dummy Word
            when  3 => data <= x"FFFFFFFF";   -- Dummy Word
            when  4 => data <= x"FFFFFFFF";   -- Dummy Word
            when  5 => data <= x"20000000";   -- Type 1 NO OP
               cs_l  <= '0';
               r_wx  <= '0';
            when  6 => data <= x"AA995566";   -- Sync Word
            when  7 => data <= x"20000000";   -- Type 1 NO OP
            when  8 => data <= x"20000000";   -- Type 1 NO OP
            when  9 => data <= x"30020001";   -- Type 1 Write 1 Word to WBSTAR
            when 10 => data <= WBSTAR;        -- Warm Boot Start Address
            when 11 => data <= x"20000000";   -- Type 1 NO OP
            when 12 => data <= x"20000000";   -- Type 1 NO OP
            when 13 => data <= x"30008001";   -- Type 1 Write 1 Words to CMD
            when 14 => data <= x"0000000F";   -- IPROG Command
            when 15 => data <= x"20000000";   -- Type 1 NO OP
            when 16 => data <= x"20000000";   -- Type 1 NO OP
               cs_l     <= '1';
               r_wx     <= '1';
            when others =>
               cs_l     <= '1';
               r_wx     <= '1';
         end case;
      end if;
   end if;
end process;

d_swapped(31)     <= data(24);
d_swapped(30)     <= data(25);
d_swapped(29)     <= data(26);
d_swapped(28)     <= data(27);
d_swapped(27)     <= data(28);
d_swapped(26)     <= data(29);
d_swapped(25)     <= data(30);
d_swapped(24)     <= data(31);
 
d_swapped(23)     <= data(16);
d_swapped(22)     <= data(17);
d_swapped(21)     <= data(18);
d_swapped(20)     <= data(19);
d_swapped(19)     <= data(20);
d_swapped(18)     <= data(21);
d_swapped(17)     <= data(22);
d_swapped(16)     <= data(23);
 
d_swapped(15)     <= data(8);
d_swapped(14)     <= data(9);
d_swapped(13)     <= data(10);
d_swapped(12)     <= data(11);
d_swapped(11)     <= data(12);
d_swapped(10)     <= data(13);
d_swapped(9)      <= data(14);
d_swapped(8)      <= data(15);
 
d_swapped(7)      <= data(0);
d_swapped(6)      <= data(1);
d_swapped(5)      <= data(2);
d_swapped(4)      <= data(3);
d_swapped(3)      <= data(4);
d_swapped(2)      <= data(5);
d_swapped(1)      <= data(6);
d_swapped(0)      <= data(7);

ICAPE2_inst: ICAPE2
generic map (
   ICAP_WIDTH => "X32"  -- Specifies the input and output data width.
)
port map (
   O     => open,       -- 32-bit output: Configuration data output bus
   CLK   => CLK,        -- 1-bit input: Clock Input
   CSIB  => cs_l,       -- 1-bit input: Active-Low ICAP Enable
   I     => d_swapped,  -- 32-bit input: Configuration data input bus
   RDWRB => r_wx        -- 1-bit input: Read/Write Select input
);
 
end arch_iprog_rst;