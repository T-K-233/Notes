---
description: The finale version, after many, many attempts
---

# Getting Started with Jetson AGX Orin and Ubuntu 22.04

This tutorial will cover how to install Ubuntu 22.04 on NVIDIA Jetson AGX Orin with the NVIDIA software tools.

The NVIDIA Jetson AGX Orin ships with Ubuntu 20.04 with relatively easy setup procedures for NVIDIA drivers and tools. If this is sufficient for your application, please refer to this tutorial instead:

{% content-ref url="using-jetson-agx-orin-with-provided-ubuntu-20.04-system.md" %}
[using-jetson-agx-orin-with-provided-ubuntu-20.04-system.md](using-jetson-agx-orin-with-provided-ubuntu-20.04-system.md)
{% endcontent-ref %}



## Requirements

* NVIDIA Jetson AGX Orin Development Kit and its accessories.
* DisplayPort cable and a compatible monitor.
* USB Type A keyboard and mouse
* A Linux host computer with Ubuntu 22.04 or 20.04.

{% hint style="danger" %}
**WARNING**

Ubuntu 24.04 is not supported as of 2024-11-05.
{% endhint %}





## Install NVIDIA SDK Manager

Download SDK Manager from [this website](https://developer.nvidia.com/sdk-manager).



After download, use apt package manager to install the application.

```bash
sudo apt install ~/Downloads/sdkmanager_2.2.0-12021_amd64.deb
```



Launch SDKManager from the app search page.

<figure><img src="../../../.gitbook/assets/Screenshot from 2024-11-05 21-50-11.png" alt=""><figcaption></figcaption></figure>



## Hardware Connection

<figure><img src="../../../.gitbook/assets/image (3).png" alt=""><figcaption></figcaption></figure>

Connect USB keyboard and mouse, and the DisplayPort port to a monitor.

Connect to the host computer with the provided USB Type-C to Type A cable using the port near the 40 pin header.

Lastly, connect the USB Type-C power cable.



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



The rest of the installation takes about one hour to finish.

<figure><img src="../../../.gitbook/assets/image (236).png" alt=""><figcaption></figcaption></figure>





