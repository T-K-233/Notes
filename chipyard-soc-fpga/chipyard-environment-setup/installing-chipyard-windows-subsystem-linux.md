# Setting up Chipyard - Windows Subsystem Linux

## System Environment

Hardware: Framework Laptop i7-1165G7

Operating System: Windows 10 Pro 21H2 Build 19044.2486

Subsystem Linux: Ubuntu 22.04.1 LTS



## 1. Install conda

Download conda from the miniforge release page.

{% embed url="https://github.com/conda-forge/miniforge/releases" %}

After download, we need to mark the script as executable. Then run the script.

```
```

## 2. Clone Chipyard

```bash
git clone https://github.com/ucb-bar/chipyard.git

export chipyard=/Documents/chipyard
cd $chipyard

git checkout stable
```

<figure><img src="../../.gitbook/assets/image (1) (2) (3).png" alt=""><figcaption></figcaption></figure>

## 3. Configure Chipyard

```bash
$chipyard/build-setup.sh riscv-tools
```



Finally, for every new terminal, run

```bash
source $chipyard/env.sh
```
