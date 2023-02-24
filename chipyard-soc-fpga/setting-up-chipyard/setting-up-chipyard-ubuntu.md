# Setting up Chipyard - Ubuntu

## System Environment

Hardware: Framework Laptop i7-1165G7

Operating System: Ubuntu 22.04.1 LTS





## 1. Install conda

Download conda from the miniforge release page.

{% embed url="https://github.com/conda-forge/miniforge/releases" %}

After download, we need to mark the script as executable. Then run the script.

<figure><img src="../../.gitbook/assets/image (96) (1).png" alt=""><figcaption></figcaption></figure>

```bash
./Mambaforge-4.14.0-0-Linux-x86_64.sh 
```



During installation, the program will prompt you to input the installation location. Here, we are using `/home/tk/Document/mambaforge`.&#x20;

<figure><img src="../../.gitbook/assets/image (4) (3) (1).png" alt=""><figcaption></figcaption></figure>

Chipyard also requires the `conda-lock` module.

```bash
conda install -n base conda-lock
conda activate base
```



## 2. Clone Chipyard

```bash
git clone https://github.com/ucb-bar/chipyard.git

export chipyard=/Documents/chipyard
cd $chipyard

git checkout stable
```



## 3. Configure Chipyard

```bash
$chipyard/build-setup.sh riscv-tools
```

<figure><img src="../../.gitbook/assets/image (3) (2) (1).png" alt=""><figcaption></figcaption></figure>

Finally, for every new terminal, run

```bash
source $chipyard/env.sh
```





