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

Instead, we need to use jetson-container following this [instruction](https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md).

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
jetson-containers run $(autotag pytorch)
```





Test if installation succeed

```bash
python
> import torch
> torch.zeros(64).cuda()
```









