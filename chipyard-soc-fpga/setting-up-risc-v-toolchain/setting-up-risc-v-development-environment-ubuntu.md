# Setting up RISC-V Development Environment - Ubuntu

```bash
sudo apt install texinfo bison flex libgmp-dev
```



```bash
cd /home/tk/Documents/RISCV/riscv-gnu-toolchain
git clone git@github.com:riscv-collab/riscv-gnu-toolchain.git
```



```bash
cd riscv-gnu-toolchain/
```



{% code overflow="wrap" %}
```bash
./configure --prefix=/home/tk/Documents/RISCV --with-multilib-generator="rv32i-ilp32--;rv32im-ilp32--;rv32ima-ilp32--;rv32imac-ilp32--;rv32imafc-ilp32f--;rv64i-lp64--;rv64im-lp64--;rv64ima-lp64--;rv64imac-lp64--;rv64imaf-lp64f--;rv64imafd-lp64d--;rv64imafdc-lp64d--"
```
{% endcode %}



