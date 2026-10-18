library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.nn_types_pkg.all;

-- Top-level FPGA module: UART Cable Interface <-> Neural Network Controller
entity top is
    generic (
        CLK_FREQ_HZ  : integer  := 50_000_000;
        BAUD_RATE    : integer  := 115_200;
        CLKS_PER_BIT : positive := 434 -- 50_000_000 / 115_200
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;

        -- Serial cable pins
        uart_rx_pin : in  std_logic;
        uart_tx_pin : out std_logic;

        -- Status LEDs: [3: TX, 2: NN, 1: RX, 0: Heartbeat]
        leds        : out std_logic_vector(3 downto 0)
    );
end entity top;

architecture rtl of top is

    constant SYNC1 : std_logic_vector(7 downto 0) := x"AA";
    constant SYNC2 : std_logic_vector(7 downto 0) := x"55";

    signal rx_done  : std_logic;
    signal rx_byte  : std_logic_vector(7 downto 0);

    signal tx_start : std_logic := '0';
    signal tx_byte  : std_logic_vector(7 downto 0) := (others => '0');
    signal tx_busy  : std_logic;
    signal tx_done  : std_logic;

    signal nn_valid_in  : std_logic := '0';
    signal nn_obs_in    : input_vector_t := (others => (others => '0'));
    signal nn_valid_out : std_logic;
    signal nn_acts_out  : output_vector_t;

    signal obs_buffer   : input_vector_t := (others => (others => '0'));
    signal acts_buffer  : output_vector_t := (others => (others => '0'));

    -- Master Controller FSM
    type state_t is (
        ST_WAIT_HEADER_1, -- Phase 1: Wait for 0xAA
        ST_WAIT_HEADER_2, -- Phase 1: Wait for 0x55
        ST_RX_PAYLOAD,    -- Phase 2: Read 64 bytes (32 observations)
        ST_TRIGGER_NN,    -- Phase 3: Launch inference
        ST_WAIT_NN,       -- Phase 4: Await neural network output
        ST_TX_HEADER_1,   -- Phase 5: Transmit 0xAA
        ST_TX_HEADER_2,   -- Phase 5: Transmit 0x55
        ST_TX_PAYLOAD,    -- Phase 5: Transmit 20 bytes (10 actions)
        ST_TX_WAIT_DONE
    );
    signal state : state_t := ST_WAIT_HEADER_1;

    signal rx_cnt : integer range 0 to 63 := 0;
    signal tx_cnt : integer range 0 to 19 := 0;
    signal msb_buf : std_logic_vector(7 downto 0) := (others => '0');

    signal hb_cnt : integer range 0 to CLK_FREQ_HZ/2 := 0;
    signal hb_led : std_logic := '0';
    signal rx_led : std_logic := '0';
    signal nn_led : std_logic := '0';
    signal tx_led : std_logic := '0';

begin

    -- Heartbeat counter for visual verification on board
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                hb_cnt <= 0;
                hb_led <= '0';
            else
                if hb_cnt < (CLK_FREQ_HZ / 2) - 1 then
                    hb_cnt <= hb_cnt + 1;
                else
                    hb_cnt <= 0;
                    hb_led <= not hb_led;
                end if;
            end if;
        end if;
    end process;

    leds(0) <= hb_led;
    leds(1) <= rx_led;
    leds(2) <= nn_led;
    leds(3) <= tx_led;

    -- UART RX
    u_rx : entity work.uart_rx
        generic map ( CLKS_PER_BIT => CLKS_PER_BIT )
        port map (
            clk     => clk,
            rst     => rst,
            rx_pin  => uart_rx_pin,
            rx_done => rx_done,
            rx_byte => rx_byte
        );

    -- UART TX
    u_tx : entity work.uart_tx
        generic map ( CLKS_PER_BIT => CLKS_PER_BIT )
        port map (
            clk      => clk,
            rst      => rst,
            tx_start => tx_start,
            tx_byte  => tx_byte,
            tx_pin   => uart_tx_pin,
            tx_busy  => tx_busy,
            tx_done  => tx_done
        );

    -- Neural Network Core
    u_nn : entity work.policy_network_top
        port map (
            clk         => clk,
            rst         => rst,
            enable      => '1',
            valid_in    => nn_valid_in,
            obs_in      => nn_obs_in,
            valid_out   => nn_valid_out,
            actions_out => nn_acts_out
        );

    -- Master Controller FSM
    process(clk)
        variable obs_i : integer range 0 to 31;
        variable act_i : integer range 0 to 9;
        variable is_low_byte : boolean;
    begin
        if rising_edge(clk) then
            if rst = '1' then
                state       <= ST_WAIT_HEADER_1;
                tx_start    <= '0';
                tx_byte     <= (others => '0');
                nn_valid_in <= '0';
                rx_cnt      <= 0;
                tx_cnt      <= 0;
                rx_led      <= '0';
                nn_led      <= '0';
                tx_led      <= '0';
            else
                case state is

                    -- Phase 1: Wait for sync header (0xAA, 0x55)
                    when ST_WAIT_HEADER_1 =>
                        tx_start    <= '0';
                        nn_valid_in <= '0';
                        rx_cnt      <= 0;
                        tx_cnt      <= 0;
                        rx_led      <= '0';
                        nn_led      <= '0';
                        tx_led      <= '0';

                        if rx_done = '1' and rx_byte = SYNC1 then
                            rx_led <= '1';
                            state  <= ST_WAIT_HEADER_2;
                        end if;

                    when ST_WAIT_HEADER_2 =>
                        if rx_done = '1' then
                            if rx_byte = SYNC2 then
                                rx_cnt <= 0;
                                state  <= ST_RX_PAYLOAD;
                            else
                                state <= ST_WAIT_HEADER_1;
                            end if;
                        end if;

                    -- Phase 2: Receive 64 payload bytes (32 16-bit observations)
                    when ST_RX_PAYLOAD =>
                        if rx_done = '1' then
                            obs_i       := rx_cnt / 2;
                            is_low_byte := (rx_cnt mod 2) = 1;

                            if not is_low_byte then
                                msb_buf <= rx_byte;
                            else
                                obs_buffer(obs_i) <= signed(msb_buf & rx_byte);
                            end if;

                            if rx_cnt = 63 then
                                rx_led <= '0';
                                state  <= ST_TRIGGER_NN;
                            else
                                rx_cnt <= rx_cnt + 1;
                            end if;
                        end if;

                    -- Phase 3: Trigger Neural Network forward inference
                    when ST_TRIGGER_NN =>
                        nn_obs_in   <= obs_buffer;
                        nn_valid_in <= '1';
                        nn_led      <= '1';
                        state       <= ST_WAIT_NN;

                    -- Phase 4: Await inference completion
                    when ST_WAIT_NN =>
                        nn_valid_in <= '0';
                        if nn_valid_out = '1' then
                            acts_buffer <= nn_acts_out;
                            nn_led      <= '0';
                            state       <= ST_TX_HEADER_1;
                        end if;

                    -- Phase 5: Transmit response packet (0xAA, 0x55 followed by 20 bytes)
                    when ST_TX_HEADER_1 =>
                        tx_led <= '1';
                        if tx_busy = '0' then
                            tx_byte  <= SYNC1;
                            tx_start <= '1';
                            state    <= ST_TX_HEADER_2;
                        end if;

                    when ST_TX_HEADER_2 =>
                        if tx_busy = '0' and tx_start = '0' then
                            tx_byte  <= SYNC2;
                            tx_start <= '1';
                            tx_cnt   <= 0;
                            state    <= ST_TX_PAYLOAD;
                        else
                            tx_start <= '0';
                        end if;

                    when ST_TX_PAYLOAD =>
                        if tx_busy = '0' and tx_start = '0' then
                            act_i       := tx_cnt / 2;
                            is_low_byte := (tx_cnt mod 2) = 1;

                            if not is_low_byte then
                                tx_byte <= std_logic_vector(acts_buffer(act_i)(15 downto 8));
                            else
                                tx_byte <= std_logic_vector(acts_buffer(act_i)(7 downto 0));
                            end if;

                            tx_start <= '1';

                            if tx_cnt = 19 then
                                state <= ST_TX_WAIT_DONE;
                            else
                                tx_cnt <= tx_cnt + 1;
                            end if;
                        else
                            tx_start <= '0';
                        end if;

                    when ST_TX_WAIT_DONE =>
                        tx_start <= '0';
                        if tx_busy = '0' then
                            tx_led <= '0';
                            state  <= ST_WAIT_HEADER_1;
                        end if;

                end case;
            end if;
        end if;
    end process;

end architecture rtl;
