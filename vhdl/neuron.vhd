library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.nn_types_pkg.all;

-- Artificial neuron with pipelined Multiply-Accumulate and Tanh activation
entity neuron is
    generic (
        NUM_INPUTS : positive := INPUT_DIM;
        DATA_WIDTH : positive := 16;
        FRAC_WIDTH : positive := 12;
        ACC_WIDTH  : positive := 32
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;
        enable      : in  std_logic;
        valid_in    : in  std_logic;
        x_in        : in  data_vector_t(0 to NUM_INPUTS-1);
        w_in        : in  data_vector_t(0 to NUM_INPUTS-1);
        b_in        : in  data_t;
        valid_out   : out std_logic;
        y_out       : out data_t;
        sum_raw_out : out data_t
    );
end entity neuron;

architecture rtl of neuron is

    type prod_array_t is array (0 to NUM_INPUTS-1) of signed(2*DATA_WIDTH-1 downto 0);
    signal prod_reg  : prod_array_t := (others => (others => '0'));
    signal b_aligned : signed(ACC_WIDTH-1 downto 0) := (others => '0');

    signal acc_sum   : signed(ACC_WIDTH-1 downto 0) := (others => '0');
    signal z_sat     : data_t := (others => '0');

    signal v_p1      : std_logic := '0';
    signal v_p2      : std_logic := '0';
    signal v_p3      : std_logic := '0';

    signal tanh_v    : std_logic;
    signal tanh_y    : data_t;

begin

    -- Phase 1: Synaptic multiplication (x_i * w_i) in parallel
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                v_p1      <= '0';
                b_aligned <= (others => '0');
                for i in 0 to NUM_INPUTS-1 loop
                    prod_reg(i) <= (others => '0');
                end loop;
            elsif enable = '1' then
                v_p1 <= valid_in;
                if valid_in = '1' then
                    for i in 0 to NUM_INPUTS-1 loop
                        prod_reg(i) <= x_in(i) * w_in(i);
                    end loop;
                    -- Align Q4.12 bias to Q8.24 accumulator scale
                    b_aligned <= shift_left(resize(b_in, ACC_WIDTH), FRAC_WIDTH);
                end if;
            end if;
        end if;
    end process;

    -- Phase 2: Accumulate products and add bias
    process(clk)
        variable total : signed(ACC_WIDTH-1 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                v_p2    <= '0';
                acc_sum <= (others => '0');
            elsif enable = '1' then
                v_p2 <= v_p1;
                if v_p1 = '1' then
                    total := b_aligned;
                    for i in 0 to NUM_INPUTS-1 loop
                        total := total + prod_reg(i);
                    end loop;
                    acc_sum <= total;
                end if;
            end if;
        end if;
    end process;

    -- Phase 3: Quantize from Q8.24 to Q4.12 and saturate
    process(clk)
        variable shifted : signed(ACC_WIDTH-1 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                v_p3  <= '0';
                z_sat <= (others => '0');
            elsif enable = '1' then
                v_p3 <= v_p2;
                if v_p2 = '1' then
                    shifted := shift_right(acc_sum, FRAC_WIDTH);
                    if shifted > 32767 then
                        z_sat <= MAX_DATA_VAL;
                    elsif shifted < -32768 then
                        z_sat <= MIN_DATA_VAL;
                    else
                        z_sat <= resize(shifted, DATA_WIDTH);
                    end if;
                end if;
            end if;
        end if;
    end process;

    sum_raw_out <= z_sat;

    -- Phase 4: Non-linear activation (Tanh)
    u_tanh : entity work.activation_tanh
        port map (
            clk       => clk,
            rst       => rst,
            enable    => enable,
            valid_in  => v_p3,
            x_in      => z_sat,
            valid_out => tanh_v,
            y_out     => tanh_y
        );

    valid_out <= tanh_v;
    y_out     <= tanh_y;

end architecture rtl;
