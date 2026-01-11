# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Spike is the official RISC-V ISA Simulator implementing a functional model of RISC-V harts. It supports RV32/RV64 base ISAs with numerous extensions (M, A, F, D, C, V, B, K, H, etc.), multiple privilege modes (Machine, Supervisor, User, Hypervisor), and virtual memory.

## Build Commands

```bash
# Dependencies (Debian/Ubuntu)
apt-get install device-tree-compiler libboost-regex-dev libboost-system-dev

# Build (out-of-tree recommended)
mkdir build && cd build
../configure --prefix=$RISCV
make

# Install
make install

# Run tests
make check

# Clean build
make clean
```

## Architecture

### Directory Structure

- `riscv/` - Core processor simulation (processor, MMU, CSRs, devices)
- `riscv/insns/` - Individual instruction implementations (920+ header files, one per instruction)
- `spike_main/` - CLI entry point and main simulation driver
- `disasm/` - Instruction disassembler
- `fesvr/` - Frontend Server for host interaction
- `softfloat/` - IEEE 754 floating-point library
- `customext/` - Custom extension framework
- `fdt/` - Flattened Device Tree support

### Key Components

**Processor (`processor.h/cc`)**: Main processor class managing PC, integer registers (XPR), floating-point registers (FPR), vector registers (VPR), and CSRs. Supports privilege levels U/S/M/V.

**MMU (`mmu.h/cc`)**: Memory management with TLB, page table walking (Sv39/48/57), and I-cache.

**Simulator (`sim.h/cc`)**: Top-level simulation orchestrating processors, memory, device bus, and debug module. Default 5000 instruction interleave between harts.

**Devices**: CLINT (timer/IPI), PLIC (interrupts), NS16550 (UART), VirtIO (block, net, rng).

### Instruction System

Instructions are defined in `riscv/insns/*.h` with one file per instruction. Each instruction file contains the execution logic using macros for register access:
- `RS1`, `RS2`, `RD` - Register operands
- `insn.i_imm()`, `insn.s_imm()`, etc. - Immediate extraction
- `MMU.load_*()`, `MMU.store_*()` - Memory operations

Instruction encoding is in `riscv/encoding.h` (170KB of bit patterns and masks).

### Adding New Instructions

1. Create `riscv/insns/<instruction_name>.h` with execution logic
2. Add opcode/mask to `riscv/encoding.h` or use riscv-opcodes
3. Add to appropriate extension list in `riscv/riscv.mk.in`
4. Rebuild

### Build System

Uses MCPPBS (Modular C++ Build System) with autoconf. Key files:
- `configure.ac` - Main configure script
- `riscv/riscv.mk.in` - Defines instruction extensions and source files
- `spike_main/spike_main.mk.in` - Main executable rules

Build generates instruction source files from `insns/*.h` templates into individual `.cc` files.

## Running Spike

```bash
# Run with proxy kernel
spike pk hello

# Interactive debug mode
spike -d pk hello

# Debug commands: reg, mem, until, while, r (run), q (quit)

# GDB debugging via OpenOCD
spike --rbb-port=9824 -m0x10100000:0x20000 program
```

## Key Configuration

- `cfg_t` - Machine configuration (core count, memory map, ISA string)
- ISA string parsed by `isa_parser.cc` (e.g., "rv64imafdc")
- Device tree generated in `dts.cc` for guest OS discovery
