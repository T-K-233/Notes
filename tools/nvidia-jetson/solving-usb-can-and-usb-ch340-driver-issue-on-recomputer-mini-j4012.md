# Solving USB-CAN and USB CH340 Driver Issue on reComputer Mini J4012

Official wiki guide:

{% embed url="https://wiki.seeedstudio.com/recomputer_jetson_mini_getting_started/" %}



It seems that gs\_usb is not included by default on Jetpack 6.X.



Useful links for USB-CAN

[https://forums.developer.nvidia.com/t/does-jetpack-6-0-not-support-gs-usb-usb-to-can/328702](https://forums.developer.nvidia.com/t/does-jetpack-6-0-not-support-gs-usb-usb-to-can/328702)

[https://forums.developer.nvidia.com/t/missing-gs-usb-kernel-module-for-jetpack-6/275287/7](https://forums.developer.nvidia.com/t/missing-gs-usb-kernel-module-for-jetpack-6/275287/7)

[https://forums.developer.nvidia.com/t/flashing-orion-nx-on-seed-recomputer-j4012-success/290669](https://forums.developer.nvidia.com/t/flashing-orion-nx-on-seed-recomputer-j4012-success/290669)



Useful links for CH340 Driver

[https://forums.developer.nvidia.com/t/issue-with-ch340-usb-to-serial-converter-not-creating-device-files-on-jetson-orin-nano-super/326022](https://forums.developer.nvidia.com/t/issue-with-ch340-usb-to-serial-converter-not-creating-device-files-on-jetson-orin-nano-super/326022)

[https://nvidia-jetson.piveral.com/jetson-orin-nano/orin-nano-wont-detect-arduino-dev-ttyusb-or-dev-ttyacm/](https://nvidia-jetson.piveral.com/jetson-orin-nano/orin-nano-wont-detect-arduino-dev-ttyusb-or-dev-ttyacm/)





Here, we assume that SDK Manager is used to prepare the Jetson SDK and toolchain.

The installation directory is assumed to be at&#x20;

```
/home/tk/Documents/nvidia/nvidia_sdk/
```

and the download folder to be at

```
/home/tk/Downloads/nvidia/sdkm_downloads/
```



Sync the Kernel Sources with Git

{% code overflow="wrap" %}
```
cd /home/tk/Documents/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/source/
./source_sync.sh -k -t jetson_36.4.3

```
{% endcode %}



release note can be found from [Jetson Linux Release Notes](https://docs.nvidia.com/jetson/archives/r36.4.3/ReleaseNotes/Jetson_Linux_Release_Notes_r36.4.3.pdf)







to build rt kernel

```
./generic_rt_build.sh "enable"
```



Change module settings

```
nano ./kernel/kernel-jammy-src/arch/arm64/configs/defconfig
```

change

```diff
CONFIG_CAN=m
+ CONFIG_CAN_GS_USB=m
CONFIG_CAN_VCAN=m
CONFIG_CAN_FLEXCAN=m
CONFIG_CAN_RCAR=m
CONFIG_CAN_RCAR_CANFD=m
```





Additionally, enable CH340 device driver support by adding

```diff
CONFIG_USB_SERIAL=m
CONFIG_USB_SERIAL_CP210X=m
+ CONFIG_USB_SERIAL_CH341=m
CONFIG_USB_SERIAL_FTDI_SIO=m
CONFIG_USB_SERIAL_OPTION=m
```





```
sudo apt-get install make build-essential libncurses-dev bison flex libssl-dev libelf-dev
```





Download toolchain

[https://developer.nvidia.com/embedded/jetson-linux-r3643](https://developer.nvidia.com/embedded/jetson-linux-r3643)

<figure><img src="../../.gitbook/assets/image (5).png" alt=""><figcaption></figcaption></figure>

and then do

```bash
tar xf aarch64--glibc--stable-2022.08-1.tar.bz2 
```



### Building the Jetson Linux Kernel

{% code overflow="wrap" %}
```bash
export CROSS_COMPILE=/home/tk/Documents/nvidia/nvidia_sdk/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
```
{% endcode %}



```bash
make -C kernel
```



{% code overflow="wrap" %}
```bash
export INSTALL_MOD_PATH=/home/tk/Documents/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/rootfs

sudo -E make install -C ./kernel/

cp ./kernel/kernel-jammy-src/arch/arm64/boot/Image /home/tk/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/kernel/Image
```
{% endcode %}





Modules

```
cd <install-path>/Linux_for_Tegra/source
```







```
export KERNEL_HEADERS=$PWD/kernel/kernel-jammy-src
make modules
```

{% code overflow="wrap" %}
```
export INSTALL_MOD_PATH=/home/tk/Documents/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/rootfs/
sudo -E make modules_install
```
{% endcode %}

```
```



Lastly, flash with SDK Manager.







after boot, do

```
sudo modprobe ch341
```



on jetson it might conflict with brltty:

can use this command to see system log when plugging and unplugging the device:

```
sudo dmesg --follow
```

```
[ 3763.532473] tegra-xusb 3610000.usb: Firmware timestamp: 2023-02-10 03:48:10 UTC
[ 3763.827034] usb 3-4: usbfs: interface 0 claimed by ch341 while 'brltty' sets config #1
[ 3763.834483] ch341-uart ttyUSB0: ch341-uart converter now disconnected from ttyUSB0
[ 3763.834546] ch341 3-4:1.0: device disconnected
[ 3794.251698] usb 3-4: USB disconnect, device number 16
[ 3796.370012] usb 3-4: new full-speed USB device number 17 using xhci_hcd
[ 3796.553318] ch341 3-4:1.0: ch341-uart converter detected
[ 3796.567375] usb 3-4: ch341-uart converter now attached to ttyUSB0
[ 3796.660855] tegra-xusb 3610000.usb: Firmware timestamp: 2023-02-10 03:48:10 UTC
[ 3796.958238] usb 3-4: usbfs: interface 0 claimed by ch341 while 'brltty' sets config #1
[ 3796.965900] ch341-uart ttyUSB0: ch341-uart converter now disconnected from ttyUSB0
[ 3796.965957] ch341 3-4:1.0: device disconnected
```



* **`usbfs: interface 0 claimed by ch341 while 'brltty' sets config #1`**: This suggests a conflict. The CH341 driver claimed the USB interface, but **`brltty`**, a service for braille terminals, also attempted to set the USB configuration at the same time.
* **`brltty` interfered**: This conflict caused the CH341 device to become unstable or be disconnected.

when this happens, `/dev/ttyUSB`  will not appear.

To resolve this, stop and permanently disable brltty:

```
sudo systemctl stop brltty
sudo systemctl disable brltty
```



If this **still** does not solve the problem, consider uninstall brltty:

```
sudo apt remove brltty
```











