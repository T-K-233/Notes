# Setting up RISC-V Toolchain - Mac

## 1. Install RISC-V Toolchain

First, we need to install the following dependencies.

```bash
brew install python3 gawk gnu-sed gmp mpfr libmpc isl zlib expat texinfo flock
```

Clone the RISC-V GNU Toolchain repo.

```bash
cd ~/Downloads
git clone https://github.com/riscv-collab/riscv-gnu-toolchain.git
cd ~/Downloads/riscv-gnu-toolchain/
```

Run configuration. The prefix is where we want to install the toolchain. Here, we will be installing under the `riscv64-unknown-toolchain` directory.

```bash
./configure --prefix=/home/tk/Documents/RISCV/riscv64-unknown-toolchain/ --with-multilib-generator="rv32i-ilp32--;rv32im-ilp32--;rv32ima-ilp32--;rv32imac-ilp32--;rv32imafc-ilp32f--;rv64i-lp64--;rv64im-lp64--;rv64ima-lp64--;rv64imac-lp64--;rv64imaf-lp64f--;rv64imafd-lp64d--;rv64imafdc-lp64d--"
```

Build the toolchain

```bash
make -j8
```

## 2. Install OpenOCD

```bash
brew install [--HEAD] openocd
```





Reference

[https://github.com/riscv/riscv-openocd/blob/riscv/README.macOS](https://github.com/riscv/riscv-openocd/blob/riscv/README.macOS)







