# Setting up Common Software on Jetson AGX Orin

## TL;DR

In general, we need to install the `aarch64` distro of the software, since this device is using the ARM64 ISA.





## Install CUDA

CUDA should be included with the Jetpack 6.1 SDK.&#x20;

To test installation, do

```bash
nvcc --version
```





## Install VSCode

When downloading from the [official website](https://code.visualstudio.com/Download), select ".deb - Arm64" button.

<figure><img src="../../../.gitbook/assets/image (237).png" alt=""><figcaption></figcaption></figure>



After download, use the following command to install.

```bash
sudo apt install ~/Downloads/code_1.95.1-1730354713.deb
```



## Install MiniForge

Download "[Mambaforge-24.9.0-0-Linux-aarch64.sh](https://github.com/conda-forge/miniforge/releases/download/24.9.0-0/Mambaforge-24.9.0-0-Linux-aarch64.sh)" from the official [release page](https://github.com/conda-forge/miniforge/releases).

```
chmod +x ~/Downloads/Mambaforge-24.9.0-0-Linux-aarch64.sh
~/Downloads/Mambaforge-24.9.0-0-Linux-aarch64.sh
```



## Install PyTorch

PyTorch with CUDA support is not available from the normal pip installation method.



### Method 1: install from whl file

{% embed url="https://developer.nvidia.com/embedded/downloads" %}

Older version of PyTorch can be found [here](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048).



First, we need to install dependencies for sparse matrix operation support

Follow instructions [here](https://developer.nvidia.com/cusparselt-downloads?target\_os=Linux\&target\_arch=aarch64-jetson\&Compilation=Native\&Distribution=Ubuntu\&target\_version=22.04\&target\_type=deb\_local).

{% code overflow="wrap" %}
```bash
wget https://developer.download.nvidia.com/compute/cusparselt/0.6.3/local_installers/cusparselt-local-tegra-repo-ubuntu2204-0.6.3_1.0-1_arm64.deb
sudo dpkg -i cusparselt-local-tegra-repo-ubuntu2204-0.6.3_1.0-1_arm64.deb
sudo cp /var/cusparselt-local-tegra-repo-ubuntu2204-0.6.3/cusparselt-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install libcusparselt0 libcusparselt-dev
```
{% endcode %}



also numpy is recommended

```bash
pip install numpy
```



Download the PyTorch wheel for JetPack 6.1.

After download, do

{% code overflow="wrap" %}
```bash
pip install ~/Downloads/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
```
{% endcode %}





### Method 2: using docker containers

NVIDIA also provided a couple of preconfigured containers that can be installed with the jetson-container command. Following this [instruction](https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md).

```bash
cd ~/Desktop/
git clone https://github.com/dusty-nv/jetson-containers
bash jetson-containers/install.sh
```



Edit `/etc/docker/daemon.json`

{% code title="daemon.json" %}
```json
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },

    "default-runtime": "nvidia"
}
```
{% endcode %}

```bash
sudo systemctl restart docker
```



If docker fails to start with error message "Failed to start Docker Application Container Engine", use this command to print the error message

```bash
sudo journalctl -u docker.service
```





Then, do&#x20;

```bash
jetson-containers run dustynv/pytorch:2.1-r36.2.0
```

{% hint style="warning" %}
**Note**

For some reason, the auto versioning command is not working:

```bash
jetson-containers run $(autotag pytorch)
```
{% endhint %}



### Testing Installation



Test if installation succeed

```bash
python
> import torch
> torch.zeros(64).cuda()
```



## Install Jtop

This is a helpful system resource monitor tool, similar to htop and nvidia-smi.

{% embed url="https://github.com/rbonghi/jetson_stats" %}

```bash
sudo pip3 install -U jetson-stats
```



After install, a system restart is required.



