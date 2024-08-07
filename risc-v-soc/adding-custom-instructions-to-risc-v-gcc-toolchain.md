# Adding Custom Instructions to RISC-V GCC Toolchain



```bash
git clone git@github.com:riscv-collab/riscv-gnu-toolchain.git
git submodule update --init --recursive
```



```bash
git clone git@github.com:riscv/riscv-opcodes.git
```





Create a new file for the new instruction. We will use `rv_zbme`

```bash
testinst    rd rs1 rs2 31..25=1  14..12=0 6..2=0x1A 1..0=3
```



```bash
./parse.py -c ./unratified/rv_zbme
```



In the resulting encoding.out.h file, we can see the custom instruction:

{% code title="encoding.out.h" %}
```c
/* Automatically generated by parse_opcodes. */
#ifndef RISCV_ENCODING_H
#define RISCV_ENCODING_H
#define MATCH_TESTINST 0x200006b
#define MASK_TESTINST 0xfe00707f

```
{% endcode %}





Then, we go to riscv-gnu-toolchain/binutils.

We need to edit the following files:





{% code title="riscv-op.h" overflow="wrap" %}
```c
#define MATCH_TESTINST 0x200006b
#define MASK_TESTINST 0xfe00707f

DECLARE_INSN(testinst, MATCH_TESTINST, MASK_TESTINST)

```
{% endcode %}



{% code title="riscv-op.c" overflow="wrap" %}
```c
{"testinst", 0, INSN_CLASS_I,       "d,s,t",         MATCH_TESTINST, MASK_TESTINST, match_opcode, 0 },

```
{% endcode %}



Make the same edit in

riscv-gnu-toolchain/gdb/opcodes/riscv-opc.c

riscv-gnu-toolchain/gdb/include/opcode/riscv-opc.h







Build

```bash
./configure --prefix=/scratch/tk/Desktop/riscv-matrix/riscv-unknown-elf/
make
```

{% code overflow="wrap" %}
```bash
./configure --prefix=/scratch/tk/Desktop/riscv-matrix/riscv-unknown-elf/ --with-cmodel=medany --enable-multilib
```
{% endcode %}



Example program

{% code title="main.c" %}
```c
#include <stdint.h>
#include <stdio.h>


int main(void) {

  asm volatile("testinst x1, x0, x4\n");
  
  printf("hello\n");

  return 0;
}

```
{% endcode %}



```bash
riscv64-unknown-elf-gcc main.c -o main.elf
```



```bash
riscv64-unknown-elf-objdump -d main.elf > disassemble.S
```







{% embed url="https://medium.com/@viveksgt/adding-custom-instructions-compilation-support-to-riscv-toolchain-78ce1b6efcf4" %}

{% embed url="https://hsandid.github.io/posts/risc-v-custom-instruction/#installing-the-risc-v-gnugcc-toolchain" %}
