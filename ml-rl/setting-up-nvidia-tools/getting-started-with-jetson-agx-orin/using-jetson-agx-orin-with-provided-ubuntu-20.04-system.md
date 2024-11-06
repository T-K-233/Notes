# Using Jetson AGX Orin with Provided Ubuntu 20.04 System



{% embed url="https://developer.nvidia.com/embedded/learn/get-started-jetson-agx-orin-devkit" %}

{% embed url="https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/developer_kit_layout.html" %}

{% embed url="https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/IN/QuickStart.html" %}





## Power Up

Connect USB keyboard and mouse, DisplayPort port, and USB Type-C power cable

<figure><img src="../../../.gitbook/assets/1000005996.jpg" alt=""><figcaption></figcaption></figure>







## Install Jetpack Tools

First, use this command to check board support package (BSP) version

```bash
cat /etc/nv_tegra_release
```

As seen in the output, the BSP is up-to-date, the apt sources lists are the latest, and we can proceed

<figure><img src="../../../.gitbook/assets/Screenshot from 2023-03-15 08-08-56.png" alt=""><figcaption></figcaption></figure>



Run the commands to install Jetpack

```bash
sudo apt update
sudo apt dist-upgrade
sudo reboot
sudo apt install nvidia-jetpack
```



Add the following lines to \~/.bashrc

{% code title=".bashrc" %}
```bash
...

# CUDA
export PATH="/usr/local/cuda/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"

...
```
{% endcode %}



Now it should detect CUDA

```bash
nvcc --version
```

<figure><img src="../../../.gitbook/assets/Screenshot from 2024-11-05 17-09-10.png" alt=""><figcaption></figcaption></figure>





## Install PyTorch

Install conda according to the "Install Conda" section in this guide

{% content-ref url="../../../risc-v-soc/quick-start-with-chipyard-on-ubuntu-or-wsl.md" %}
[quick-start-with-chipyard-on-ubuntu-or-wsl.md](../../../risc-v-soc/quick-start-with-chipyard-on-ubuntu-or-wsl.md)
{% endcontent-ref %}



```bash
conda create -yn tensorrt python=3.10
```

```bash
pip install torch
```







