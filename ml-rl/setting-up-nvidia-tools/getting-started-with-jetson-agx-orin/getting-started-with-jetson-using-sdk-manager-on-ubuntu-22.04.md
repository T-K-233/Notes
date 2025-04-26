---
description: The finale version, after many, many attempts
---

# Getting Started with Jetson Using SDK Manager on Ubuntu 22.04

This tutorial will cover how to install Ubuntu 22.04 on NVIDIA Jetson AGX Orin and Jetson Orin NX with the NVIDIA SDK Manager.

The NVIDIA Jetson AGX Orin ships with Ubuntu 20.04 with relatively easy setup procedures for NVIDIA drivers and tools. If this is sufficient for your application, please refer to this tutorial instead:

{% content-ref url="using-jetson-agx-orin-with-provided-ubuntu-20.04-system.md" %}
[using-jetson-agx-orin-with-provided-ubuntu-20.04-system.md](using-jetson-agx-orin-with-provided-ubuntu-20.04-system.md)
{% endcontent-ref %}



## Requirements

* NVIDIA Jetson AGX Orin Development Kit or NVIDIA Jetson Orin NX Development Kit and its accessories.
* DisplayPort cable and a compatible monitor.
* USB Type A keyboard and mouse
* A Linux host computer with Ubuntu 22.04 or 20.04.



NVIDIA SDK Manager has strict requirement on the host OS version. The system compatibility matrix is available from the [SDK Manager website](https://developer.nvidia.com/sdk-manager):

<figure><img src="../../../.gitbook/assets/image (6).png" alt=""><figcaption></figcaption></figure>



## Install NVIDIA SDK Manager

Download SDK Manager from [this website](https://developer.nvidia.com/sdk-manager).



After download, use apt package manager to install the application.

```bash
sudo apt install ~/Downloads/sdkmanager_2.2.0-12021_amd64.deb
```



Launch SDKManager from the app search page.

<figure><img src="../../../.gitbook/assets/Screenshot from 2024-11-05 21-50-11.png" alt=""><figcaption></figcaption></figure>



## Hardware Connection

<figure><img src="../../../.gitbook/assets/image (3) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>

Connect USB keyboard and mouse, and the DisplayPort port to a monitor.

Connect to the host computer with the provided USB Type-C to Type A cable using the port near the 40 pin header.

Lastly, while **holding the recovery button** (middle button), connect the USB Type-C power cable. The device will enter recover mode and can be detected on the USB port.



## Flashing the Device

After connecting the hardware and powering up the device, it should automatically detect the USB device connection.

Select the correct device type.

<figure><img src="../../../.gitbook/assets/Screenshot from 2024-11-05 21-50-48.png" alt=""><figcaption></figcaption></figure>



Choose the desired component to install. Here we select everything.

<figure><img src="../../../.gitbook/assets/image (232).png" alt=""><figcaption></figcaption></figure>



After entering the system password, it will begin to download and install the components.

<figure><img src="../../../.gitbook/assets/image (233).png" alt=""><figcaption></figcaption></figure>

After it's done, it will prompt to configure the account information and ask for the installation location.

We will install to the built-in eMMC device.

<figure><img src="../../../.gitbook/assets/image (234).png" alt=""><figcaption></figcaption></figure>



About one third into the installation process, the Jetson will reboot and boot into the system.

When this happens, the SDKManager on the host computer will create this prompt.

Wait until the Jetson boots up fully, then log into the system, and then click Install on the host side.

<figure><img src="../../../.gitbook/assets/image (235).png" alt=""><figcaption></figcaption></figure>

If it cannot find the device through the USB connection, try Ethernet connection instead.



{% hint style="info" %}
## Note

Sometimes it will report this error:

{% code overflow="wrap" %}
```sh
ERROR Flash Jetson Linux - flash: exportfs: Failed to stat /home/tk/Downloads/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/: No such file or directory
```
{% endcode %}

In case this happens, a possible solutions is as follows.

```bash
sudo nano /etc/exports
```

Remove ALL the uncommented lines in the file.

<img src="../../../.gitbook/assets/image (7).png" alt="" data-size="original">\
\
Then, try the installation again.
{% endhint %}





The rest of the installation takes about one hour to finish.

<figure><img src="../../../.gitbook/assets/image (236).png" alt=""><figcaption></figcaption></figure>



Finally, to verify the installed version, run the following commands on Jetson

```bash
sudo apt show nvidia-jetpack -a
```







## Reference

[https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/two\_ways\_to\_set\_up\_software.html](https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/two_ways_to_set_up_software.html)





### Starting over

To remove all the existing configurations of the SDK Manager, run the following command

```bash
rm -rf ~/.nvsdkm/
```







