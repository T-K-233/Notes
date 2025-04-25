# Solving USB-CAN Issue on reComputer Mini J4012

Official wiki guide:

{% embed url="https://wiki.seeedstudio.com/recomputer_jetson_mini_getting_started/" %}



It seems that gs\_usb is not included by default on Jetpack 6.X.



Useful links

[https://forums.developer.nvidia.com/t/does-jetpack-6-0-not-support-gs-usb-usb-to-can/328702](https://forums.developer.nvidia.com/t/does-jetpack-6-0-not-support-gs-usb-usb-to-can/328702)

[https://forums.developer.nvidia.com/t/missing-gs-usb-kernel-module-for-jetpack-6/275287/7](https://forums.developer.nvidia.com/t/missing-gs-usb-kernel-module-for-jetpack-6/275287/7)

[https://forums.developer.nvidia.com/t/flashing-orion-nx-on-seed-recomputer-j4012-success/290669](https://forums.developer.nvidia.com/t/flashing-orion-nx-on-seed-recomputer-j4012-success/290669)









{% code overflow="wrap" %}
```
cd /home/tk/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/source/
./source_sync.sh -k -t jetson_36.4.3

```
{% endcode %}



release note is found here:

[https://docs.nvidia.com/jetson/archives/r36.4.3/ReleaseNotes/Jetson\_Linux\_Release\_Notes\_r36.4.3.pdf](https://docs.nvidia.com/jetson/archives/r36.4.3/ReleaseNotes/Jetson_Linux_Release_Notes_r36.4.3.pdf)





to buiuld rt kernel

```
./generic_rt_build.sh "enable"
```



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



{% code overflow="wrap" %}
```
tar xf aarch64--glibc--stable-2022.08-1.tar.bz2 

export CROSS_COMPILE=/home/tk/Downloads/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
```
{% endcode %}





```
make -C kernel
```



{% code overflow="wrap" %}
```
export INSTALL_MOD_PATH=/home/tk/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/rootfs

sudo -E make install -C ./kernel/

cp ./kernel/kernel-jammy-src/arch/arm64/boot/Image /home/tk/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/kernel/Image
```
{% endcode %}





Modules





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
cd ..
sudo ./tools/l4t_update_initrd.sh
```







