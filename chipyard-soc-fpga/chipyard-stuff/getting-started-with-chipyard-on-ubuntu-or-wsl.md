# Getting Started With Chipyard on Ubuntu or WSL

{% hint style="info" %}
**Note:**

The following tutorial content is tested on Ubuntu 20.04, Ubuntu 22.04, and WSL 2.0 on Windows 10 systems with Chipyard release 1.10.1
{% endhint %}



## Installation

### Step 1. Install conda

Chipyard uses [Conda](https://docs.conda.io/en/latest/) to manage the development environment and packages.

Download Conda from the miniforge [release page](https://github.com/conda-forge/miniforge/releases).



Select the corresponding version and download the file ending with ".sh".

<figure><img src="../../.gitbook/assets/conda.png" alt=""><figcaption></figcaption></figure>

After download, we need to mark the script as executable.&#x20;

```bash
chmod +x ~/Downloads/Mambaforge-23.3.1-1-Linux-x86_64.sh
```

Then, execute the script.

```bash
~/Downloads/Mambaforge-23.3.1-1-Linux-x86_64.sh
```

Follow the installation prompt. The program will prompt you to input the installation location. Here, we are using `/home/tk/Documents/mambaforge`.

After installation, it will ask whether to execute `conda init`. Enter "yes" to the prompt.

<figure><img src="../../.gitbook/assets/conda (1).png" alt=""><figcaption></figcaption></figure>

Conda is now installed on the system.

{% hint style="info" %}
**Note**:&#x20;

After installing conda, the `conda` path is not added to the PATH environment variable of the current terminal.&#x20;

If the `conda: command not found` error occured, open a new terminal (or source `~./bashrc`).
{% endhint %}



### Step 2. Install conda-lock

Chipyard also requires the `conda-lock` module. Install `conda-lock` by executing the following commands.

```bash
conda install -n base conda-lock==1.4.0
conda activate base
```



### Step 3. Clone Chipyard

Open terminal in a known location. Here, we will use the directory `/home/tk/Desktop/`. In the terminal, execute the following command.

```bash
git clone git@github.com:ucb-bar/chipyard.git
```



### Step 4. Configure Chipyard

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
./build-setup.sh riscv-tools -s 6 -s 7 -s 8 -s 9 --force
```

The `--force` flag is used to skip the confirmation prompt, which is buggy on some terminals.



The set up process will take around 10-30 minutes, depending on the system configuration. After the script is finished, Chipyard is initialized and is ready to be used.



## Running RTL Simulation





























