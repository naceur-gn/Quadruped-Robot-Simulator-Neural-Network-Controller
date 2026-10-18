# Quadruped Robot Neural Network - VHDL Hardware Implementation

This folder contains the complete, synthesizable **VHDL-93 / VHDL-2008** hardware accelerator implementation of the quadruped robot neural network policy.

```
       32 Sensory Inputs (via USB-UART Cable RX)
             │
             ▼
  ┌──────────────────────┐
  │  UART Receiver (RX)  │  ◄── 115200 Baud, 8-N-1 (64 bytes received)
  └──────────┬───────────┘
             │
             ▼
  ┌──────────────────────┐
  │  Hidden Layer (64)   │  ◄── W1 [64x32] & b1 [64]  (work.weights_pkg)
  │  (Tanh Activation)   │
  └──────────┬───────────┘
             │  64 Hidden Activations
             ▼
  ┌──────────────────────┐
  │  Output Layer (10)   │  ◄── W2 [10x64] & b2 [10]  (work.weights_pkg)
  │  (Tanh Activation)   │
  └──────────┬───────────┘
             │
             ▼
  ┌──────────────────────┐
  │ UART Transmitter(TX) │  ──► 115200 Baud, 8-N-1 (20 bytes sent)
  └──────────────────────┘
             │
             ▼
      10 Motor Actions (via USB-UART Cable TX)
```

## 🌟 The Core Principle: "A Neural Network is Just Addition and Multiplication"

In digital hardware (VHDL / FPGA), an artificial neural network is fundamentally built by mastering a **simple 1-synapse neuron** (Multiply + Add) and then scaling it to **N dimensions** using VHDL's `generate` statement:

```
Step 1: Elementary Primitive
  ┌───────────────┐
  │  multiplier   │──► prod = x * w
  └───────────────┘
  ┌───────────────┐
  │     adder     │──► sum  = a + b
  └───────────────┘

Step 2: Simple 1-Dimension Neuron (simple_neuron.vhd)
  y = (x * w) + b

Step 3: Generated N-Dimension Neuron (neuron_n_dim.vhd)
  VHDL `for i in 0 to N-1 generate`:
    chain(0) = (x(0) * w(0)) + b
    chain(i) = (x(i) * w(i)) + chain(i-1)
  ──► Result: y = b + sum_{i=0}^{N-1} (x_i * w_i)

Step 4: Generated M-Neuron Neural Network Layer (layer_n_dim.vhd)
  VHDL `for j in 0 to M-1 generate`:
    instantiates M parallel N-dimensional neurons!
```

---

## 1. Fixed-Point Arithmetic Format (Q4.12)

Floating-point units (FPUs) in FPGAs require significant logic and introduce multi-cycle latency. This architecture employs **16-bit Signed Fixed-Point Q4.12**:

- **Total Width**: 16 bits (`signed(15 downto 0)`)
- **Sign Bit**: 1 bit
- **Integer Bits**: 3 bits (dynamic range: $[-8.0, +7.999755859375]$)
- **Fractional Bits**: 12 bits ($1.0 = 2^{12} = 4096$)
- **Resolution**: $2^{-12} \approx 0.00024414$
- **Multiplication**: $Q4.12 \times Q4.12 = Q8.24$ (32-bit intermediate product).
- **Accumulator Guard**: 32-bit internal register preventing overflow during 32/64 input accumulations.

---

## 2. Structure of the Artificial Neuron (`neuron.vhd`)

Each neuron computes the standard biological/artificial perceptron equation:

$$y = \tanh\left( \sum_{i=0}^{N-1} x_i \cdot w_i + b \right)$$

### Pipelined Dataflow Architecture:

```
 x_i (Q4.12) ──┐
               ├──► [ DSP Mult ] ──► P_i (Q8.24) ──┐
 w_i (Q4.12) ──┘                                   │
                                                   ▼
 b (Q4.12) ────► [ << 12 Aligner ] ─────────► [ Adder Tree ]
                                                   │
                                                   ▼
                                            acc_sum (Q8.24)
                                                   │
                                                   ▼
                                           [ >> 12 Scaler ]
                                                   │
                                                   ▼
                                           [ Saturation Unit ]
                                            [-32768, +32767]
                                                   │
                                                   ▼ z (Q4.12)
                                        [ activation_tanh ]
                                                   │
                                                   ▼
                                            y_out (Q4.12)
```

### Pipeline Stages:
1. **Stage 1 (Synaptic Multiplication)**: $N$ parallel signed multiplications ($x_i \cdot w_i$) in $Q8.24$ format. Bias $b$ is aligned by left-shifting by 12 bits.
2. **Stage 2 (Adder Tree & Accumulation)**: Computes the sum of all $N$ products plus the aligned bias:
   $$\text{acc\_sum} = \sum_{i=0}^{N-1} P_i + (b \ll 12)$$
3. **Stage 3 (Quantization & Saturation)**: Converts 32-bit $Q8.24$ back to 16-bit $Q4.12$ via arithmetic right-shift by 12 bits, with hard saturation to $[-32768, +32767]$ ($[-8.0, +7.999]$).
4. **Stage 4 (Non-Linear Activation)**: Feeds pre-activation $z$ into the hyperbolic tangent unit (`activation_tanh.vhd`).

---

## 3. Synaptic Weights and Biases (`weights_pkg.vhd`)

The trained neural network policy parameters from PyTorch RL training (`training/model_weights.json`) are quantized to fixed-point $Q4.12$ and compiled as VHDL constants:

- **$W_1$ Matrix** (`W1_WEIGHTS`): `64` hidden neurons $\times$ `32` inputs = $2,048$ synaptic weights.
- **$b_1$ Vector** (`B1_BIAS`): `64` bias constants for the hidden layer.
- **$W_2$ Matrix** (`W2_WEIGHTS`): `10` output neurons $\times$ `64` inputs = $640$ synaptic weights.
- **$b_2$ Vector** (`B2_BIAS`): `10` bias constants for the output layer.

---

## 4. Activation Function (`activation_tanh.vhd`)

Hyperbolic tangent $\tanh(x)$ is implemented via a **symmetric 65-point Look-Up Table (LUT)** with **on-chip linear interpolation**:

- Exploit odd symmetry: $\tanh(-x) = -\tanh(x)$ and $\tanh(0) = 0$.
- For $|x| \ge 4.0$ ($16384$ in $Q4.12$), output saturates to $\pm 1.0$ ($4096$).
- For $|x| < 4.0$, LUT provides base value $y_0$ and step delta $\Delta y$.
- Hardware linear interpolation:
  $$y = y_0 + \frac{\Delta y \times \text{fractional\_remainder}}{256}$$
- **Accuracy**: Maximum error compared to 64-bit IEEE double float $\tanh(x)$ is **$< 0.0007$** across the entire $[-8.0, +8.0]$ domain.
- **Latency**: 2 clock cycles.

---

## 5. File Manifest

| File | Type | Description |
|---|---|---|
| [`nn_types_pkg.vhd`](file:///c:/Users/gnich/Desktop/robot_nural_network/vhdl/nn_types_pkg.vhd) | Package | Fixed-point Q4.12 types, dimension constants, and arithmetic utility functions. |
| [`weights_pkg.vhd`](file:///c:/Users/gnich/Desktop/robot_nural_network/vhdl/weights_pkg.vhd) | Package | Pre-trained quadruped weights ($W_1$, $W_2$) and biases ($b_1$, $b_2$) in Q4.12. |
| [`activation_tanh.vhd`](file:///c:/Users/gnich/Desktop/robot_nural_network/vhdl/activation_tanh.vhd) | Entity | Pipelined 65-point symmetric LUT + linear interpolation Tanh activation. |
| [`neuron.vhd`](file:///c:/Users/gnich/Desktop/robot_nural_network/vhdl/neuron.vhd) | Entity | Parallel artificial neuron: Synaptic multiplication, addition, saturation, and $\tanh$. |
| [`hidden_layer.vhd`](file:///c:/Users/gnich/Desktop/robot_nural_network/vhdl/hidden_layer.vhd) | Entity | Dense layer of 64 parallel neurons mapped to $W_1$ and $b_1$. |
| [`output_layer.vhd`](file:///c:/Users/gnich/Desktop/robot_nural_network/vhdl/output_layer.vhd) | Entity | Dense layer of 10 parallel neurons mapped to $W_2$ and $b_2$. |
| [`policy_network_top.vhd`](file:///c:/Users/gnich/Desktop/robot_nural_network/vhdl/policy_network_top.vhd) | Entity | Cascaded neural network accelerator ($32 \to 64 \to 10$). |
| [`uart_rx.vhd`](file:///c:/Users/gnich/Desktop/robot_nural_network/vhdl/uart_rx.vhd) | Entity | 8-N-1 UART receiver with input synchronizer and mid-bit sampling. |
| [`uart_tx.vhd`](file:///c:/Users/gnich/Desktop/robot_nural_network/vhdl/uart_tx.vhd) | Entity | 8-N-1 UART transmitter with busy/done handshake. |
| [`top.vhd`](file:///c:/Users/gnich/Desktop/robot_nural_network/vhdl/top.vhd) | Entity | **Top-Level Design**: Master FSM receiving 32 inputs, executing inference, and transmitting 10 actions. |
| [`top.ucf`](file:///c:/Users/gnich/Desktop/robot_nural_network/vhdl/top.ucf) | Constraints | **User Constraints File (UCF)**: Clock, reset, UART RX/TX pins, and diagnostic status LEDs. |

---

## 6. Synthesis in Xilinx ISE / Vivado

1. Add the design files to your project:
   - `nn_types_pkg.vhd`
   - `weights_pkg.vhd`
   - `activation_tanh.vhd`
   - `neuron.vhd`
   - `hidden_layer.vhd`
   - `output_layer.vhd`
   - `policy_network_top.vhd`
   - `uart_rx.vhd`
   - `uart_tx.vhd`
   - `top.vhd` (Set as Top Module)
2. Add the constraints file:
   - `top.ucf`
3. Run Synthesis and Implementation to generate the FPGA bitstream.
