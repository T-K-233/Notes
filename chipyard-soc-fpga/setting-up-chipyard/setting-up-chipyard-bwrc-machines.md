# Setting Up Chipyard - BWRC Machines

## System Environment

On BWRC login servers



## 1. Set up conda environment

Run the following commands to activate the base conda environment

```bash
source /tools/C/chiyufeng/documents/miniconda3/bin/activate
conda activate base
```



## 2. Install conda-lock

If this is the first time conda is activated on the user environment, we need to install `conda-lock`.

```bash
conda install -n base conda-lock
```



## 3. Clone Chipyard

Open terminal in the working directory. Here, we will use the directory `/tools/C/chiyufeng/tapeout/` as an example. In the terminal, execute the following command.

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

By default, chipyard setup script initializes/installs things in the following order:&#x20;

1. Conda environment
2. Chipyard submodules
3. Toolchain collateral (Spike, PK, tests, libgloss)
4. Ctags
5. Chipyard pre-compile sources
6. FireSim
7. FireSim pre-compile sources
8. FireMarshal
9. FireMarshal pre-compile default buildroot Linux sources
10. Runs repository clean-up



To execute the setup script, run the following command.

```bash
$chipyard/build-setup.sh riscv-tools -s 6 -s 7 -s 8 -s 9
```

The script will prompt the following message. Enter "y" and press Enter key to continue.

Alternatively, the release check prompt can be skipped by passing the "--force" flag.

```bash
$chipyard/build-setup.sh riscv-tools -s 6 -s 7 -s 8 -s 9 --force
```



## 4. On New Terminal

Finally, for every new terminal, run the following script to set up all the environment variables required by Chipyard.

```bash
source $chipyard/env.sh
```



## 5. Dealing with Failed Setups

If the setup script fails to compile, try running

```bash
git clean -fxd
```



Worst case, run this to re-link the submodules

```bash
git submodule deinit -f .
./scripts/init-submodules-no-riscv-tools.sh 
```



##



