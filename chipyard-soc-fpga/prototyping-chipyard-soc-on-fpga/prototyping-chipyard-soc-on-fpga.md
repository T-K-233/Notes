---
description: Chipyard Arty Flow
---

# Prototyping Chipyard SoC on FPGA - Ubuntu

## Setting up Chipyard

See the following tutorial to install Chipyard.

{% content-ref url="../installing-chipyard.md" %}
[installing-chipyard.md](../installing-chipyard.md)
{% endcontent-ref %}



## Setting up Vivado

See the following tutorial to install Vivado.

{% content-ref url="installing-xilinx-vivado-on-ubuntu-22.04.md" %}
[installing-xilinx-vivado-on-ubuntu-22.04.md](installing-xilinx-vivado-on-ubuntu-22.04.md)
{% endcontent-ref %}

add vivado to env.sh script

```bash
PATH=/home/tk/Documents/Xilinx/Vivado/2023.2/bin:$PATH
```



## Building Bitstream

```bash
cd $chipyard/
source ./env.sh
```

```bash
cd $chipyard/fpga/
```

to make default arty config

```bash
make SUB_PROJECT=arty bitstream
```

to make example config

```bash
make SUB_PROJECT=ExampleChipArty35TConfig bitstream
```

```bash
make SUB_PROJECT=ExampleChipArty100TConfig bitstream
```





```bash
make LD_SCRIPT=./bsp/examplechip/examplechip_scratch.ld ARCH=rv32imaczicsr ABI=ilp32
```





## JTAG Debugging

See the following tutorial to set up FT-LINK.



