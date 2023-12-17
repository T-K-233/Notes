# Prototyping Chipyard SoC on FPGA - BWRC

## Setting up Chipyard

See the following tutorial to install Chipyard.

{% content-ref url="broken-reference" %}
[Broken link](broken-reference)
{% endcontent-ref %}



## Setting up Vivado

```bash
source /tools/xilinx/Vivado/2022.1/settings64.sh
```



## Building Bitstream

```bash
cd $chipyard
source ./env.sh
```

```bash
cd $chipyard/fpga/
```

```bash
bsub -q ee194 -Is -XF make SUB_PROJECT=arty bitstream
```

