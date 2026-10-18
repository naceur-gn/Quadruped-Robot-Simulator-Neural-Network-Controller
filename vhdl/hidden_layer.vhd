library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.nn_types_pkg.all;
use work.weights_pkg.all;

-- Layer 1: 32 observations -> 64 hidden neurons
entity hidden_layer is
    port (
        clk       : in  std_logic;
        rst       : in  std_logic;
        enable    : in  std_logic;
        valid_in  : in  std_logic;
        x_obs     : in  input_vector_t;
        valid_out : out std_logic;
        h_out     : out hidden_vector_t
    );
end entity hidden_layer;

architecture rtl of hidden_layer is
    signal neuron_valids : std_logic_vector(0 to HIDDEN_DIM-1);
begin

    -- Instantiate 64 parallel neurons mapped to pre-trained W1 and b1
    gen_neurons : for i in 0 to HIDDEN_DIM-1 generate
        u_neuron : entity work.neuron
            generic map (
                NUM_INPUTS => INPUT_DIM,
                DATA_WIDTH => DATA_WIDTH,
                FRAC_WIDTH => FRAC_WIDTH,
                ACC_WIDTH  => ACC_WIDTH
            )
            port map (
                clk         => clk,
                rst         => rst,
                enable      => enable,
                valid_in    => valid_in,
                x_in        => x_obs,
                w_in        => W1_WEIGHTS(i),
                b_in        => B1_BIAS(i),
                valid_out   => neuron_valids(i),
                y_out       => h_out(i),
                sum_raw_out => open
            );
    end generate gen_neurons;

    valid_out <= neuron_valids(0);

end architecture rtl;
