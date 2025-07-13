#  IITK-Mini-MIPS Processor

This project presents a 32-bit, single-cycle Mini-MIPS processor implemented in Verilog. It features a modular design with separate instruction and data memory, supporting a core set of MIPS instructions. The architecture is designed for clarity and educational purposes, demonstrating fundamental concepts of computer architecture.

---

## Key Features

* **32-Bit Single-Cycle Architecture:** All instructions are executed in a single clock cycle.
* **Harvard Architecture:** Utilizes separate memory modules for instructions and data, implemented using Vivado's Distributed Dual-Port RAM IP.
* **Comprehensive Register Set:** Includes 32 general-purpose registers and 32 floating-point registers.
* **Instruction Support:** Implements R-type, I-type, and J-type instruction formats.
* **Modular Design:** The processor is broken down into distinct, reusable modules such as the ALU, Controller, Register File, and memory units.
* **Floating-Point Capability:** Includes dedicated units for single-precision floating-point addition, subtraction, and comparison.

---

## Instruction Set Architecture (ISA)

The processor supports a custom-encoded instruction set based on the MIPS standard, categorized into three primary formats:

### R-Type Instructions
Used for register-to-register operations.
* **Format:** `[31:26] opcode | [25:21] rd | [20:16] rt | [15:11] rs | [10:6] shamt | [5:0] func`
* **Supported Instructions:** `add`, `sub`, `and`, `sll`, `xor`

### I-Type Instructions
Used for operations involving an immediate value.
* **Format:** `[31:26] opcode | [25:21] rd | [20:16] rt | [15:0] immediate`
* **Supported Instructions:** `addi`, `lw`, `sw`, `beq`, `slti`

### J-Type Instructions
Used for unconditional jumps.
* **Format:** `[31:26] opcode | [25:0] jaddress`
* **Supported Instructions:** `j`, `jal`

Instruction decoding is managed by the `Splitter` module, which parses the 32-bit instruction word into its constituent fields.

---

## Core Components

The processor's design is highly modular, with each component responsible for a specific function.

* **DistributedMemory:** A 1024-location dual-port RAM used for both Instruction Memory and Data Memory. Reads are performed on the positive clock edge, and writes occur on the negative clock edge.

* **RegisterFile:** Manages the 32 general-purpose and 32 floating-point registers. It supports asynchronous reads and synchronous writes on the negative clock edge. All registers are initialized to zero on reset.

* **Splitter:** Decodes a 32-bit instruction into its functional parts: `opcode`, `rs`, `rt`, `rd`, `shamt`, `func`, `address_constant`, and `jaddress`.

* **ALU (Arithmetic Logic Unit):** Executes a range of operations, including:
    * **Arithmetic:** `add`, `sub`, `addu`, `subu`
    * **Logical:** `and`, `or`, `xor`, `not`
    * **Shift:** `sll`, `srl`, `sla`, `sra`
    * **Comparison:** `slt`, `seq`
    * Also supports 64-bit multiplication.

* **Floating Point Units (FPU):**
    * **floating_adder:** A dedicated module for floating-point addition and subtraction, correctly handling sign, exponent, and mantissa.
    * **FPU:** The main floating-point unit that executes `add.s`, `sub.s`, and `c.eq.s`.

* **Controllers:**
    * **PC\_Controller:** Manages the Program Counter (PC), updating it sequentially or based on branch and jump instructions. The PC is set to `0` on reset.
    * **ALU\_controller:** Generates the appropriate ALU control signals based on the instruction's `opcode` and `func` fields.
    * **Controller:** The main control unit that generates primary control signals like `write_reg`, `data_write`, `jump`, and `branch`.
    * **FPR\_controller:** Manages control signals specific to the floating-point register file.

* **Auxiliary Modules:**
    * **SignExtender:** Performs sign extension of 16-bit immediate values to 32 bits.
    * **mux2\_1:** A standard 2-to-1 multiplexer used for signal selection throughout the design.

* **CPU (Top-Level Module):** Integrates all components to form the complete processor, managing the instruction lifecycle from fetch to writeback.

---

## Processor Workflow

The processor follows a single-cycle execution model:

1.  **Fetch:** The PC provides the address to the Instruction Memory, which fetches the 32-bit instruction.
2.  **Decode:** The instruction is passed to the `Splitter` and `Controller`. The `Splitter` extracts the various fields, while the `Controller` generates the necessary control signals for the datapath.
3.  **Execute:**
    * For R-type and I-type arithmetic/logic instructions, the ALU performs the specified operation.
    * For floating-point instructions, the FPU is used.
    * For memory access instructions (`lw`, `sw`), the ALU calculates the effective address.
4.  **Memory Access:** For `lw` or `sw` instructions, the Data Memory is read from or written to.
5.  **Writeback:** The result of the operation (from the ALU, FPU, or Data Memory) is written back to the destination register.

Branch and jump logic concurrently determines the address of the next instruction, which is loaded into the PC for the next cycle.

---

## Usage Guide

### System Initialization
1.  **Reset:** Assert the `rst` signal (`rst = 1`) to initialize the Program Counter to `0` and clear all registers.
2.  **Load Program:** Use the `write_instruction`, `address`, and `inst_data` inputs to load instructions into the Instruction Memory.
3.  **Pre-load Data:** Use the `write_data`, `memory_in`, and `memory_write` inputs to populate the Data Memory if required.

### Execution
1.  **De-assert Reset:** Set `rst = 0`.
2.  **Provide Clock:** Supply a `clk` signal to begin execution.
3.  **Monitor Outputs:** Observe the processor state through the `OutputOfR1` to `OutputOfR5` signals. A `done` signal is asserted when the PC reaches a predefined address (50), indicating the completion of the program.

---

## Design Notes

* **Memory/Register Behavior:** Reads from the register file and memory modules are asynchronous, providing immediate output. Writes are synchronous and occur on the negative edge of the clock.
* **Floating-Point Implementation:** The floating-point logic is simplified for single-precision operations. It can be extended to achieve full IEEE-754 compliance.
* **Modularity:** The design is intentionally modular and commented to facilitate understanding, modification, and extension.