# Setting up Chipyard - Ubuntu

## System Environment

Hardware: Framework Laptop i7-1165G7

Operating System: Ubuntu 22.04.1 LTS

## 1. Install conda

Download conda from the miniforge release page.

{% embed url="https://github.com/conda-forge/miniforge/releases" %}

Select the corresponding version and download the file ending with ".sh".

<figure><img src="../.gitbook/assets/image (161).png" alt=""><figcaption></figcaption></figure>

After download, we need to mark the script as executable. Right click the file, and select "Properties..." option. In the Properties window, go to "Permissions" tab. Check the "Allow executing file as program" selection.

<figure><img src="../.gitbook/assets/image (96) (1).png" alt=""><figcaption></figcaption></figure>

Open terminal in the download folder and execute the ".sh" script.

```bash
./Mambaforge-4.14.0-0-Linux-x86_64.sh 
```

Follow the installation prompt. The program will prompt you to input the installation location. Here, we are using `/home/tk/Documents/mambaforge`.

After installation, it will ask whether to execute `conda init`. Enter "yes" to the prompt.

<figure><img src="../.gitbook/assets/image (4) (3) (1).png" alt=""><figcaption></figcaption></figure>

## 2. Install conda-lock

Chipyard also requires the `conda-lock` module. Install `conda-lock` by executing the following commands.

> **Note**: after installing conda, the `conda` path is not added to the PATH environment variable of the current terminal. If the `conda: command not found` error occured, open a new terminal (or source `~./bashrc`).

```bash
conda install -n base conda-lock==1.4.0
conda activate base
```

## 3. Clone Chipyard

Open terminal in a known location. Here, we will use the directory `/home/tk/Desktop/`. In the terminal, execute the following command.

```bash
git clone https://github.com/ucb-bar/chipyard.git

export chipyard=/home/tk/Desktop/chipyard
cd $chipyard

git checkout stable
```

## 4. Configure Chipyard

By default, chipyard setup script initializes/installs things in the following order:

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

<figure><img src="../.gitbook/assets/image (3) (2) (1).png" alt=""><figcaption></figcaption></figure>

Alternatively, the release check prompt can be skipped by passing the "--force" flag.

```bash
$chipyard/build-setup.sh riscv-tools -s 6 -s 7 -s 8 -s 9 --force
```

## 4. On New Terminal

Finally, for every new terminal, run the following script to set up all the environment variables required by Chipyard.

```bash
source $chipyard/env.sh
```
