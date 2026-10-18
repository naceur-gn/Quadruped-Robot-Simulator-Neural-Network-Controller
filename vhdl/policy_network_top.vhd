library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.nn_types_pkg.all;

-- Cascaded neural network accelerator (32 inputs -> 64 hidden -> 10 actions)
entity policy_network_top is
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;
        enable      : in  std_logic;
        valid_in    : in  std_logic;
        obs_in      : in  input_vector_t;
        valid_out   : out std_logic;
        actions_out : out output_vector_t
    );
end entity policy_network_top;

architecture rtl of policy_network_top is

    signal hidden_valid : std_logic := '0';
    signal hidden_acts  : hidden_vector_t := (others => (others => '0'));

begin

    -- Layer 1: 32 observations to 64 hidden neurons
    u_hidden : entity work.hidden_layer
        port map (
            clk       => clk,
            rst       => rst,
            enable    => enable,
            valid_in  => valid_in,
            x_obs     => obs_in,
            valid_out => hidden_valid,
            h_out     => hidden_acts
        );

    -- Layer 2: 64 hidden features to 10 action outputs
    u_output : entity work.output_layer
        port map (
            clk        => clk,
            rst        => rst,
            enable     => enable,
            valid_in   => hidden_valid,
            h_in       => hidden_acts,
            valid_out  => valid_out,
            action_out => actions_out
        );

end architecture rtl;
