# NVIDIA Omniverse Isaac Sim Setup

## System Overview

Intel&#x20;

NVIDIA RTX 2070

Ubuntu 22.04 with [standard install](https://notes.tk233.xyz/tools/ubuntu-22.04-standard-installation-procedure)





## Install NVIDIA driver

Launch "Software & Updates"

In the "Additional Drivers" tab, select "Using NVIDIA driver metapackage from nvidia-driver-525 (proprietary)".

<figure><img src="../.gitbook/assets/Screenshot from 2022-12-22 09-59-43.png" alt=""><figcaption></figcaption></figure>

Restart the computer after installation.



Now, we should be able to run `nvidia-smi`

<figure><img src="../.gitbook/assets/image (4) (5).png" alt=""><figcaption></figcaption></figure>





## Install Omniverse

{% embed url="https://docs.omniverse.nvidia.com/prod_install-guide/prod_launcher/installing_launcher.html#launcher-setup" %}

Download Omniverse from [here](https://www.nvidia.com/en-us/omniverse/download/).

After download, grant the executable right to the .AppImage file.

<figure><img src="../.gitbook/assets/image (1) (1).png" alt=""><figcaption></figcaption></figure>



The app will also require FUSE to run

```bash
sudo apt install libfuse2
```

Launch the application by either double-clicking it or via the terminal.



Setup the directories in the launcher. Here's an example setup:

<figure><img src="../.gitbook/assets/image (17).png" alt=""><figcaption></figcaption></figure>
