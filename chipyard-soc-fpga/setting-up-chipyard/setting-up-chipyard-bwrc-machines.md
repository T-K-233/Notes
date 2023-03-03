# Setting Up Chipyard - BWRC Machines

## System Environment

BWRC servers



## 1. Set up conda environment

```bash
conda activate /tools/C/raghavgupta/intech22/sp23/chipyard-lab-sp23/.conda-env
```

## 2. Clone Chipyard

```bash
git clone https://github.com/ucb-bar/chipyard.git

export chipyard=/tools/C/chiyufeng/tapeout/chipyard-demo
cd $chipyard

git checkout stable
```

## 3. Configure Chipyard

```bash
$chipyard/scripts/init-submodules-no-riscv-tools.sh
```



Finally, for every new terminal, run

```bash
source $chipyard/env.sh
```

