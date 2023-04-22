# Setting Up Chipyard - BWRC Machines

## System Environment

On BWRC login servers



## 1. Set up conda environment

```bash
source /tools/C/chiyufeng/documents/miniconda3/bin/activate
conda activate base
```

If running for the first time, we need to first install conda-lock before running `conda activate`.

```bash
conda install -n base conda-lock
```



## 2. Clone Chipyard

```bash
git clone https://github.com/ucb-bar/chipyard.git
```

Set chipyard path to an environment variable. We will be referencing this path for our script locations from now on.

```bash
export chipyard=/tools/C/chiyufeng/tapeout/chipyard
```

```bash
cd $chipyard
```



## 3. Configure Chipyard

Optionally, ssh to any of the following server for faster setup.

```bash
ssh -XY bwrcr740-8
ssh -XY bwrcr740-9
ssh -XY bwrcix-1
```



~~\[Depricated] On old branches:~~

```
$chipyard/build-setup.sh riscv-tools
```



On the newer release branch:

```bash
$chipyard/build-setup.sh riscv-tools -s 6 -s 7 -s 8 -s 9
```

When it prompts whether to continue setup Chipyard, enter `y`.



## 4. On New Terminal

For every new terminal, run the following script.

```bash
source $chipyard/env.sh
```

