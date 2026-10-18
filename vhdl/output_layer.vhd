library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.nn_types_pkg.all;
use work.weights_pkg.all;

-- Layer 2: 64 hidden activations -> 10 motor actions
entity output_layer is
    port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        enable     : in  std_logic;
        valid_in   : in  std_logic;
        h_in       : in  hidden_vector_t;
        valid_out  : out std_logic;
        action_out : out output_vector_t
    );
end entity output_layer;

architecture rtl of output_layer is
    signal neuron_valids : std_logic_vector(0 to OUTPUT_DIM-1);
begin

    -- Instantiate 10 parallel neurons mapped to pre-trained W2 and b2
    gen_neurons : for i in 0 to OUTPUT_DIM-1 generate
        u_neuron : entity work.neuron
            generic map (
                NUM_INPUTS => HIDDEN_DIM,
                DATA_WIDTH => DATA_WIDTH,
                FRAC_WIDTH => FRAC_WIDTH,
                ACC_WIDTH  => ACC_WIDTH
            )
            port map (
                clk         => clk,
                rst         => rst,
                enable      => enable,
                valid_in    => valid_in,
                x_in        => h_in,
                w_in        => W2_WEIGHTS(i),
                b_in        => B2_BIAS(i),
                valid_out   => neuron_valids(i),
                y_out       => action_out(i),
                sum_raw_out => open
            );
    end generate gen_neurons;

    valid_out <= neuron_valids(0);

end architecture rtl;
