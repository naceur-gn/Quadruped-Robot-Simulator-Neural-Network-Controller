library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Standard 8-N-1 UART transmitter
entity uart_tx is
    generic (
        CLKS_PER_BIT : positive := 434 -- 50 MHz / 115200 baud
    );
    port (
        clk      : in  std_logic;
        rst      : in  std_logic;
        tx_start : in  std_logic;
        tx_byte  : in  std_logic_vector(7 downto 0);
        tx_pin   : out std_logic;
        tx_busy  : out std_logic;
        tx_done  : out std_logic
    );
end entity uart_tx;

architecture rtl of uart_tx is

    type state_t is (ST_IDLE, ST_START_BIT, ST_DATA_BITS, ST_STOP_BIT, ST_CLEANUP);
    signal state : state_t := ST_IDLE;

    signal clk_cnt  : integer range 0 to CLKS_PER_BIT-1 := 0;
    signal bit_idx  : integer range 0 to 7 := 0;
    signal tx_shift : std_logic_vector(7 downto 0) := (others => '0');

begin

    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                state    <= ST_IDLE;
                tx_pin   <= '1';
                tx_busy  <= '0';
                tx_done  <= '0';
                clk_cnt  <= 0;
                bit_idx  <= 0;
                tx_shift <= (others => '0');
            else
                case state is
                    when ST_IDLE =>
                        tx_pin  <= '1';
                        tx_done <= '0';
                        clk_cnt <= 0;
                        bit_idx <= 0;
                        if tx_start = '1' then
                            tx_busy  <= '1';
                            tx_shift <= tx_byte;
                            state    <= ST_START_BIT;
                        else
                            tx_busy  <= '0';
                        end if;

                    when ST_START_BIT =>
                        tx_pin <= '0'; -- Start bit
                        if clk_cnt < CLKS_PER_BIT - 1 then
                            clk_cnt <= clk_cnt + 1;
                        else
                            clk_cnt <= 0;
                            state   <= ST_DATA_BITS;
                        end if;

                    when ST_DATA_BITS =>
                        tx_pin <= tx_shift(bit_idx);
                        if clk_cnt < CLKS_PER_BIT - 1 then
                            clk_cnt <= clk_cnt + 1;
                        else
                            clk_cnt <= 0;
                            if bit_idx < 7 then
                                bit_idx <= bit_idx + 1;
                            else
                                bit_idx <= 0;
                                state   <= ST_STOP_BIT;
                            end if;
                        end if;

                    when ST_STOP_BIT =>
                        tx_pin <= '1'; -- Stop bit
                        if clk_cnt < CLKS_PER_BIT - 1 then
                            clk_cnt <= clk_cnt + 1;
                        else
                            clk_cnt <= 0;
                            tx_done <= '1';
                            state   <= ST_CLEANUP;
                        end if;

                    when ST_CLEANUP =>
                        tx_done <= '0';
                        tx_busy <= '0';
                        state   <= ST_IDLE;

                end case;
            end if;
        end if;
    end process;

end architecture rtl;
