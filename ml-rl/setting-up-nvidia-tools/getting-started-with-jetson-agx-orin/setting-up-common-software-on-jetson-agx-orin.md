# Setting up Common Software on Jetson AGX Orin

## TL;DR

In general, we need to install the `aarch64` distro of the software, since this device is using the ARM64 ISA.





## Install CUDA

CUDA should be included with the Jetpack 6.1 SDK. We just need to add it to PATH

{% code title=".bashrc" %}
```bash
...

# CUDA
export PATH="/usr/local/cuda/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"

...
```
{% endcode %}





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

dependencies

```bash
sudo apt install curl libopenblas-dev
```



```bash
conda create -yn tensorrt python=3.10
```

{% code overflow="wrap" %}
```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124
```
{% endcode %}

Test if installation succeed

```bash
python
> import torch
> torch.zeros(64).cuda()
```









