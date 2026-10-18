library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Standard 8-N-1 UART receiver with 2-FF input synchronizer and mid-bit sampling
entity uart_rx is
    generic (
        CLKS_PER_BIT : positive := 434 -- 50 MHz / 115200 baud
    );
    port (
        clk     : in  std_logic;
        rst     : in  std_logic;
        rx_pin  : in  std_logic;
        rx_done : out std_logic;
        rx_byte : out std_logic_vector(7 downto 0)
    );
end entity uart_rx;

architecture rtl of uart_rx is

    type state_t is (ST_IDLE, ST_START_BIT, ST_DATA_BITS, ST_STOP_BIT, ST_CLEANUP);
    signal state : state_t := ST_IDLE;

    signal rx_sync_1 : std_logic := '1';
    signal rx_sync   : std_logic := '1';

    signal clk_cnt   : integer range 0 to CLKS_PER_BIT-1 := 0;
    signal bit_idx   : integer range 0 to 7 := 0;
    signal rx_shift  : std_logic_vector(7 downto 0) := (others => '0');

begin

    -- 2-stage synchronizer to avoid metastability on external asynchronous pin
    process(clk)
    begin
        if rising_edge(clk) then
            rx_sync_1 <= rx_pin;
            rx_sync   <= rx_sync_1;
        end if;
    end process;

    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                state    <= ST_IDLE;
                rx_done  <= '0';
                rx_byte  <= (others => '0');
                clk_cnt  <= 0;
                bit_idx  <= 0;
                rx_shift <= (others => '0');
            else
                case state is
                    when ST_IDLE =>
                        rx_done <= '0';
                        clk_cnt <= 0;
                        bit_idx <= 0;
                        if rx_sync = '0' then
                            state <= ST_START_BIT;
                        end if;

                    when ST_START_BIT =>
                        -- Sample middle of start bit to filter spurious glitches
                        if clk_cnt = (CLKS_PER_BIT / 2) then
                            if rx_sync = '0' then
                                clk_cnt <= 0;
                                state   <= ST_DATA_BITS;
                            else
                                state   <= ST_IDLE;
                            end if;
                        else
                            clk_cnt <= clk_cnt + 1;
                        end if;

                    when ST_DATA_BITS =>
                        if clk_cnt < CLKS_PER_BIT - 1 then
                            clk_cnt <= clk_cnt + 1;
                        else
                            clk_cnt <= 0;
                            rx_shift(bit_idx) <= rx_sync;
                            if bit_idx < 7 then
                                bit_idx <= bit_idx + 1;
                            else
                                bit_idx <= 0;
                                state   <= ST_STOP_BIT;
                            end if;
                        end if;

                    when ST_STOP_BIT =>
                        if clk_cnt < CLKS_PER_BIT - 1 then
                            clk_cnt <= clk_cnt + 1;
                        else
                            rx_done <= '1';
                            rx_byte <= rx_shift;
                            clk_cnt <= 0;
                            state   <= ST_CLEANUP;
                        end if;

                    when ST_CLEANUP =>
                        rx_done <= '0';
                        state   <= ST_IDLE;

                end case;
            end if;
        end if;
    end process;

end architecture rtl;
