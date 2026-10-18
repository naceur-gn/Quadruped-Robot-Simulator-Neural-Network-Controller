library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package nn_types_pkg is

    -- Network architecture: 32 inputs -> 64 hidden -> 10 outputs
    constant INPUT_DIM      : integer := 32;
    constant HIDDEN_DIM     : integer := 64;
    constant OUTPUT_DIM     : integer := 10;

    -- Fixed-point arithmetic parameters (signed Q4.12)
    constant DATA_WIDTH     : integer := 16;
    constant FRAC_WIDTH     : integer := 12;
    constant ACC_WIDTH      : integer := 32;

    constant MAX_DATA_VAL   : signed(DATA_WIDTH-1 downto 0) := to_signed(32767, DATA_WIDTH);
    constant MIN_DATA_VAL   : signed(DATA_WIDTH-1 downto 0) := to_signed(-32768, DATA_WIDTH);

    subtype data_t is signed(DATA_WIDTH-1 downto 0);
    subtype acc_t  is signed(ACC_WIDTH-1 downto 0);

    type data_vector_t is array (natural range <>) of data_t;

    -- Layer vectors
    subtype input_vector_t  is data_vector_t(0 to INPUT_DIM-1);
    subtype hidden_vector_t is data_vector_t(0 to HIDDEN_DIM-1);
    subtype output_vector_t is data_vector_t(0 to OUTPUT_DIM-1);

    type w1_matrix_t is array (0 to HIDDEN_DIM-1) of input_vector_t;
    type w2_matrix_t is array (0 to OUTPUT_DIM-1) of hidden_vector_t;

end package nn_types_pkg;
