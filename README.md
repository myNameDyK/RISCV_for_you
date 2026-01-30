# RISC-V 5-Stage Pipelined CPU (Verilog)

This project implements a **32-bit RISC-V (RV32I) 5-stage pipelined CPU**
written in **Verilog**, designed for **simulation and FPGA implementation**.
The CPU follows the classic pipeline structure:
**IF → ID → EX → MEM → WB**, with hazard handling and forwarding.

---
## FPGA Implementation Note (BRAM & Synchronous Read)

This CPU is designed to be synthesizable on FPGA.
When targeting FPGA, **instruction memory and data memory are implemented
using Block RAM (BRAM)** instead of LUT-based memory.
As a result:
- Memory read operations are **synchronous**
- Read data becomes valid **one clock cycle later**
- This introduces an inherent **1-cycle latency** for instruction fetch
  and load operations

The pipeline design and hazard handling logic take this behavior into account.
Therefore, the timing behavior on FPGA may differ from an ideal
asynchronous-memory simulation model.
This design choice reflects realistic FPGA hardware constraints.

## 1. Features

- RV32I base instruction set
- 5-stage pipeline:
  - Instruction Fetch (IF)
  - Instruction Decode (ID)
  - Execute (EX)
  - Memory (MEM)
  - Write Back (WB)
- Data forwarding (EX/MEM, MEM/WB)
- Load-use hazard detection & stalling
- Control hazard handling (branch / jump flush)
- Separate instruction and data memory
- Designed for Vivado simulation and FPGA synthesis

---
## 2. Supported Instructions

### Arithmetic / Logic
- `add`, `sub`
- `and`, `or`, `xor`
- `sll`, `srl`, `sra`
- `slt`, `sltu`
- `addi`, `andi`, `ori`, ...

### Memory
- `lw`
- `sw`

### Control Flow
- `beq`, `bne`
- `jal`
- `jalr`

### Upper Immediate
- `lui`
- `auipc`

---

## 3. Simulation (Vivado)

### Step 1: Open Vivado
- Create a new project
- Add `rtl/` as **Design Sources**
- Add `sim/` as **Simulation Sources**

### Step 2: Set Simulation Top
- Top module: `tb_RISC_V_Control`

### Step 3: Run Simulation
- `Run Behavioral Simulation`

> The instruction memory loads program code from `sim/mem.hex`
> using `$readmemh`.

---

## 4. Instruction Memory
Instruction memory is initialized using: verilog
$readmemh("mem.hex", imem);
The file mem.hex must be placed in the simulation directory (sim/).

Notes
Register x0 is hard-wired to zero.
Branch decision is resolved in the Execute stage.
Load-use hazards introduce one pipeline stall.
Forwarding priority: EX/MEM > MEM/WB.

Future Improvements
Support for more RISC-V instructions
Exception and interrupt handling
CSR support
Instruction cache / data cache
Performance counters

Author
Name: Duy Khoa Nguyễn

Language: Verilog HDL

Tool: Vivado

License
This project is for educational and research purposes.
