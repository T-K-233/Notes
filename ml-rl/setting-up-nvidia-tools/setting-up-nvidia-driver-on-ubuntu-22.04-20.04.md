# Setting up NVIDIA Driver on Ubuntu 22.04 / 20.04

## Installing NVIDIA Driver

{% tabs %}
{% tab title="Ubuntu GUI Install" %}
Launch "Software & Updates" application on Ubuntu.

In the "Additional Drivers" tab, select "Using NVIDIA driver metapackage from nvidia-driver-535 (proprietary)".

<figure><img src="../../.gitbook/assets/Screenshot from 2023-12-22 01-07-21.png" alt=""><figcaption></figcaption></figure>

nvidia-smi should work after rebooting the system.
{% endtab %}

{% tab title="Ubuntu CLI Install" %}
Suggested by this [tutorial](https://ubuntu.com/server/docs/nvidia-drivers-installation), we can install the driver from command line:

First, list all available drivers with this command

```bash
sudo ubuntu-drivers list
```



Then, install the driver with following command

```bash
sudo apt install nvidia-utils-535-server
sudo apt install nvidia-driver-535-server
```



nvidia-smi should work after rebooting the system.



(Optional) If the system is not configured correctly, try run the following command and manually select the packages.

```bash
sudo ubuntu-drivers install --gpgpu nvidia:535-server
```
{% endtab %}
{% endtabs %}



## Verifying Correct Installation

Restart the computer after installation.

Now, we should be able to run `nvidia-smi`

{% hint style="info" %}
**Note:**

When installing the driver, an NVIDIA CUDA toolkit that is compatible with this driver version will also be installed.
{% endhint %}

```bash
> nvidia-smi
Wed Mar 20 15:09:06 2024       
+---------------------------------------------------------------------------------------+
| NVIDIA-SMI 535.161.07             Driver Version: 535.161.07   CUDA Version: 12.2     |
|-----------------------------------------+----------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |         Memory-Usage | GPU-Util  Compute M. |
|                                         |                      |               MIG M. |
|=========================================+======================+======================|
|   0  NVIDIA GeForce GTX 1070        Off | 00000000:01:00.0  On |                  N/A |
| 50%   44C    P0              31W / 200W |    531MiB /  8192MiB |      1%      Default |
|                                         |                      |                  N/A |
+-----------------------------------------+----------------------+----------------------+
|   1  NVIDIA GeForce RTX 2080        Off | 00000000:02:00.0 Off |                  N/A |
| 24%   32C    P2              48W / 215W |    255MiB /  8192MiB |      0%      Default |
|                                         |                      |                  N/A |
+-----------------------------------------+----------------------+----------------------+
                                                                                         
+---------------------------------------------------------------------------------------+
| Processes:                                                                            |
|  GPU   GI   CI        PID   Type   Process name                            GPU Memory |
|        ID   ID                                                             Usage      |
|=======================================================================================|
|    0   N/A  N/A      1385      G   /usr/lib/xorg/Xorg                          234MiB |
|    0   N/A  N/A      1631      G   /usr/bin/gnome-shell                         65MiB |
|    0   N/A  N/A      3104      G   ...seed-version=20240320-050127.641000       77MiB |
|    0   N/A  N/A      7085      G   ...ures=SpareRendererForSitePerProcess       71MiB |
|    0   N/A  N/A      7541    C+G   ...sim-2023.1.1/kit/python/bin/python3       54MiB |
|    1   N/A  N/A      1385      G   /usr/lib/xorg/Xorg                            4MiB |
|    1   N/A  N/A      7541    C+G   ...sim-2023.1.1/kit/python/bin/python3      237MiB |
+---------------------------------------------------------------------------------------+
```









