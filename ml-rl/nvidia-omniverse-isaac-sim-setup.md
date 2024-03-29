# Setting up NVIDIA Omniverse Isaac Sim on Ubuntu 22.04 / 20.04

## System Overview

NVIDIA RTX 2080

Ubuntu 22.04 with [standard install](https://notes.tk233.xyz/tools/ubuntu-22.04-standard-installation-procedure)

## Prerequisite: NVIDIA Driver

{% content-ref url="setting-up-nvidia-tools/setting-up-nvidia-driver-on-ubuntu-22.04-20.04.md" %}
[setting-up-nvidia-driver-on-ubuntu-22.04-20.04.md](setting-up-nvidia-tools/setting-up-nvidia-driver-on-ubuntu-22.04-20.04.md)
{% endcontent-ref %}



## Install Omniverse

Download Omniverse from [here](https://www.nvidia.com/en-us/omniverse/download/).

After download, grant the executable right to the .AppImage file.

<figure><img src="../.gitbook/assets/image (1) (1) (2) (2).png" alt=""><figcaption></figcaption></figure>

The installation program requires FUSE to run.

Install FUSE with apt:

```bash
sudo apt install libfuse2
```



Create the file structures in the installation directory:

```bash
mkdir -p ~/Documents/Omniverse/pkg
mkdir -p ~/Documents/Omniverse/data
mkdir -p ~/Documents/Omniverse/contents
mkdir -p ~/Documents/Omniverse/cache
```





Launch the application by either double-clicking it or via the terminal.

Set up the directories in the launcher. Here's an example setup:

<figure><img src="../.gitbook/assets/image (209).png" alt=""><figcaption></figcaption></figure>

## Install Isaac Sim

Search for "Isaac Sim" in the EXCHANGE tab and click INSTALL button.

<figure><img src="../.gitbook/assets/image (207).png" alt=""><figcaption></figcaption></figure>













