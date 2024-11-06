# Getting Started with Jetson AGX Orin



{% embed url="https://developer.nvidia.com/embedded/learn/get-started-jetson-agx-orin-devkit" %}

{% embed url="https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/developer_kit_layout.html" %}





## Power Up

Connect USB keyboard and mouse, DisplayPort port, and USB Type-C power cable

<figure><img src="../../.gitbook/assets/1000005996.jpg" alt=""><figcaption></figcaption></figure>







## Install Jetpack Tools

First, use this command to check board support package (BSP) version

```bash
cat /etc/nv_tegra_release
```

As seen in the output, the BSP is up-to-date, the apt sources lists are the latest, and we can proceed

<figure><img src="../../.gitbook/assets/Screenshot from 2023-03-15 08-08-56.png" alt=""><figcaption></figcaption></figure>



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







