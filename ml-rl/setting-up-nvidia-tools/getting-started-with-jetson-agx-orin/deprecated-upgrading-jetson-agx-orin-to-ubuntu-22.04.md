# \[Deprecated] Upgrading Jetson AGX Orin to Ubuntu 22.04

{% embed url="https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/IN/QuickStart.html" %}

## Hardware connection

<figure><img src="../../../.gitbook/assets/image (3) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>

Connect to the host computer with the provided USB Type-C to Type A cable using the port near the 40 pin header.

Connect power cable as the last step.







{% embed url="https://developer.nvidia.com/sdk-manager" %}



























## Download the system image

download these two files on the host system

{% embed url="https://developer.nvidia.com/embedded/jetson-linux-r3640" %}

<figure><img src="../../../.gitbook/assets/image (3) (1) (1).png" alt=""><figcaption></figcaption></figure>



Install dependencies

```bash
sudo apt install qemu-user-static netcat-openbsd
```





```bash
tar xf ~/Downloads/Jetson_Linux_R36.4.0_aarch64.tbz2
sudo tar xpf ~/Downloads/Tegra_Linux_Sample-Root-Filesystem_R36.4.0_aarch64.tbz2 -C ~/Downloads/Linux_for_Tegra/rootfs/
cd ~/Downloads/Linux_for_Tegra/
sudo ./tools/l4t_flash_prerequisites.sh
sudo ./apply_binaries.sh
```







## Set the device to force recovery mode

Press and hold the recovery button (middle one), click the power button (left one), and then release the recovery button



Alternatively, hold down the recovery button, and plug in the power cable. To ensure this is effective, also press and release the power button while still holding the recovery button.







Confirm the device is in force recovery mode

<figure><img src="../../../.gitbook/assets/image (2) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>

\[[website](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/IN/QuickStart.html#to-determine-whether-the-developer-kit-is-in-force-recovery-mode)]



```bash
lsusb
```

<figure><img src="../../../.gitbook/assets/image (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>

Yes





To flash onto the eMMC drive, do

```bash
sudo ./flash.sh jetson-agx-orin-devkit internal
```



After flashing, it will automatically reset the device and boot it up.

<figure><img src="../../../.gitbook/assets/image (4) (1) (1).png" alt=""><figcaption></figcaption></figure>







