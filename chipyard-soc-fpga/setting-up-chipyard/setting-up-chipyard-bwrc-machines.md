# Setting Up Chipyard - BWRC Machines

## System Environment

BWRC servers



## 1. Set up conda environment

```bash
source /tools/C/ee290-sp23/miniconda3/bin/activate
conda activate base
//conda activate /tools/C/raghavgupta/intech22/sp23/chipyard-lab-sp23/.conda-env
```

## 2. Clone Chipyard

```bash
git clone https://github.com/ucb-bar/chipyard.git

export chipyard=/tools/C/chiyufeng/tapeout/chipyard-demo
cd $chipyard

git checkout stable
```

## 3. Configure Chipyard

ssh -XY bwrcr740-8

ssh -XY bwrcr740-9



```bash
$chipyard/build-setup.sh riscv-tools # or esp-tools
// bsub -q ee194 -Is -XF $chipyard/build-setup.sh riscv-tools -s 6 -s 7 -s 8 -s 9
//$chipyard/scripts/init-submodules-no-riscv-tools.sh
```



Finally, for every new terminal, run

```bash
source $chipyard/env.sh
```

