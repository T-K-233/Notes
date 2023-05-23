# Setting up RV32 Toolchain on BWRC

The default RISCV toolchain that comes with Chipyard does not support RV32 / software floating-point compilation.&#x20;

We need to set up a new toolchain.

```bash
git clone git@github.com:riscv-collab/riscv-gnu-toolchain.git
cd riscv-gnu-toolchain/
```



Change "--prefix" to your desired installation directory.

This takes eternity to compile...

{% code overflow="wrap" %}
```bash
./configure --prefix=/tools/C/chiyufeng/documents/RISCV --with-multilib-generator="rv32i-ilp32--;rv32im-ilp32--;rv32ima-ilp32--;rv32imac-ilp32--;rv32imafc-ilp32f--;rv64i-lp64--;rv64im-lp64--;rv64ima-lp64--;rv64imac-lp64--;rv64imaf-lp64f--;rv64imafd-lp64d--;rv64imafdc-lp64d--"
```
{% endcode %}

```bash
make
```



Or, can just source this environment

```bash
source /tools/C/chiyufeng/documents/riscv_env.sh
```





