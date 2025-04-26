# Solving USB-CAN Issue on reComputer Mini J4012

Official wiki guide:

{% embed url="https://wiki.seeedstudio.com/recomputer_jetson_mini_getting_started/" %}



It seems that gs\_usb is not included by default on Jetpack 6.X.



Useful links

[https://forums.developer.nvidia.com/t/does-jetpack-6-0-not-support-gs-usb-usb-to-can/328702](https://forums.developer.nvidia.com/t/does-jetpack-6-0-not-support-gs-usb-usb-to-can/328702)

[https://forums.developer.nvidia.com/t/missing-gs-usb-kernel-module-for-jetpack-6/275287/7](https://forums.developer.nvidia.com/t/missing-gs-usb-kernel-module-for-jetpack-6/275287/7)

[https://forums.developer.nvidia.com/t/flashing-orion-nx-on-seed-recomputer-j4012-success/290669](https://forums.developer.nvidia.com/t/flashing-orion-nx-on-seed-recomputer-j4012-success/290669)





Here, we assume that SDK Manager is used to prepare the Jetson SDK and toolchain.

The installation directory is assumed to be at&#x20;

```
/home/tk/nvidia/nvidia_sdk/
```

and the download folder to be at

```
/home/tk/Downloads/nvidia/sdkm_downloads
```



Sync the Kernel Sources with Git

{% code overflow="wrap" %}
```
cd /home/tk/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/source/
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





```
sudo apt-get install make build-essential libncurses-dev bison flex libssl-dev libelf-dev
```





Download toolchain

[https://developer.nvidia.com/embedded/jetson-linux-r3643](https://developer.nvidia.com/embedded/jetson-linux-r3643)

<figure><img src="../../../.gitbook/assets/image.png" alt=""><figcaption></figcaption></figure>

and then do

```bash
tar xf aarch64--glibc--stable-2022.08-1.tar.bz2 
```



### Building the Jetson Linux Kernel

{% code overflow="wrap" %}
```bash
export CROSS_COMPILE=/home/tk/nvidia/nvidia_sdk/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
```
{% endcode %}



```bash
make -C kernel
```



{% code overflow="wrap" %}
```bash
export INSTALL_MOD_PATH=/home/tk/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/rootfs

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
export INSTALL_MOD_PATH=/home/tk/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/rootfs/
sudo -E make modules_install
```
{% endcode %}

```
```



Lastly, flash with SDK Manager.









