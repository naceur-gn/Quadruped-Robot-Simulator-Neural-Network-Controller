library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.nn_types_pkg.all;

-- Tanh activation unit using 65-point symmetric LUT with linear interpolation
entity activation_tanh is
    port (
        clk       : in  std_logic;
        rst       : in  std_logic;
        enable    : in  std_logic;
        valid_in  : in  std_logic;
        x_in      : in  data_t;
        valid_out : out std_logic;
        y_out     : out data_t
    );
end entity activation_tanh;

architecture rtl of activation_tanh is

    -- LUT sampled at step = 0.0625 for x in [0, 4.0] (4096 = 1.0)
    type tanh_lut_t is array (0 to 64) of integer range 0 to 4096;
    constant TANH_LUT : tanh_lut_t := (
           0,  256,  509,  759, 1003, 1240, 1468, 1686,
        1893, 2088, 2272, 2443, 2602, 2748, 2883, 3007,
        3119, 3222, 3315, 3399, 3475, 3543, 3604, 3659,
        3707, 3751, 3790, 3825, 3856, 3883, 3908, 3929,
        3949, 3966, 3981, 3994, 4006, 4016, 4026, 4034,
        4041, 4048, 4053, 4058, 4063, 4067, 4070, 4073,
        4076, 4078, 4080, 4082, 4084, 4085, 4086, 4088,
        4089, 4089, 4090, 4091, 4091, 4092, 4092, 4093,
        4093
    );

    -- Phase 1 registers: Absolute value and indexing
    signal sign_p1     : std_logic := '0';
    signal sat_p1      : boolean   := false;
    signal idx_p1      : integer range 0 to 64 := 0;
    signal rem_p1      : unsigned(7 downto 0) := (others => '0');
    signal valid_p1    : std_logic := '0';

    -- Phase 2 registers: Table lookup
    signal sign_p2     : std_logic := '0';
    signal sat_p2      : boolean   := false;
    signal y0_p2       : integer range 0 to 4096 := 0;
    signal dy_p2       : integer range -4096 to 4096 := 0;
    signal rem_p2      : unsigned(7 downto 0) := (others => '0');
    signal valid_p2    : std_logic := '0';

begin

    -- Phase 1: Input decomposition (sign, absolute value, and table index)
    process(clk)
        variable abs_val : unsigned(DATA_WIDTH-1 downto 0);
        variable idx     : integer;
    begin
        if rising_edge(clk) then
            if rst = '1' then
                valid_p1 <= '0';
                sign_p1  <= '0';
                sat_p1   <= false;
                idx_p1   <= 0;
                rem_p1   <= (others => '0');
            elsif enable = '1' then
                valid_p1 <= valid_in;
                if valid_in = '1' then
                    sign_p1 <= x_in(DATA_WIDTH-1);

                    if x_in(DATA_WIDTH-1) = '1' then
                        abs_val := unsigned(-x_in);
                    else
                        abs_val := unsigned(x_in);
                    end if;

                    -- Saturation boundary at x >= 4.0 (16384 in Q4.12)
                    if abs_val >= to_unsigned(16384, DATA_WIDTH) then
                        sat_p1 <= true;
                        idx_p1 <= 64;
                        rem_p1 <= (others => '0');
                    else
                        sat_p1 <= false;
                        idx := to_integer(abs_val(13 downto 8));
                        if idx > 63 then
                            idx := 63;
                        end if;
                        idx_p1 <= idx;
                        rem_p1 <= abs_val(7 downto 0);
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- Phase 2: Read table points (y0 and y1) and compute step delta
    process(clk)
        variable val0, val1 : integer range 0 to 4096;
    begin
        if rising_edge(clk) then
            if rst = '1' then
                valid_p2 <= '0';
                sign_p2  <= '0';
                sat_p2   <= false;
                y0_p2    <= 0;
                dy_p2    <= 0;
                rem_p2   <= (others => '0');
            elsif enable = '1' then
                valid_p2 <= valid_p1;
                sign_p2  <= sign_p1;
                sat_p2   <= sat_p1;
                rem_p2   <= rem_p1;

                if sat_p1 then
                    y0_p2 <= 4096;
                    dy_p2 <= 0;
                else
                    val0  := TANH_LUT(idx_p1);
                    val1  := TANH_LUT(idx_p1 + 1);
                    y0_p2 <= val0;
                    dy_p2 <= val1 - val0;
                end if;
            end if;
        end if;
    end process;

    -- Phase 3: Linear interpolation and sign restoration
    process(clk)
        variable interp_val : integer;
        variable mag        : integer;
    begin
        if rising_edge(clk) then
            if rst = '1' then
                valid_out <= '0';
                y_out     <= (others => '0');
            elsif enable = '1' then
                valid_out <= valid_p2;
                if valid_p2 = '1' then
                    if sat_p2 then
                        mag := 4096;
                    else
                        interp_val := y0_p2 + (dy_p2 * to_integer(rem_p2)) / 256;
                        if interp_val > 4096 then
                            mag := 4096;
                        else
                            mag := interp_val;
                        end if;
                    end if;

                    if sign_p2 = '1' then
                        y_out <= to_signed(-mag, DATA_WIDTH);
                    else
                        y_out <= to_signed(mag, DATA_WIDTH);
                    end if;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
