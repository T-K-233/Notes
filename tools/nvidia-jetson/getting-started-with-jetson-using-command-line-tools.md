# Getting Started with Jetson Using Command Line Tools



[https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/IN/QuickStart.html](https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/IN/QuickStart.html)



```bash
sudo apt-get install make build-essential libncurses-dev bison flex libssl-dev libelf-dev
```







Download [Sample Root Filesystem Sources](https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v5.0/sources/ubuntu_jammy-l4t_aarch64_src.tbz2) and [Bootlin Toolchain gcc 11.3](https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v3.0/toolchain/aarch64--glibc--stable-2022.08-1.tar.bz2) from the website:

{% embed url="https://developer.nvidia.com/embedded/jetson-linux-r365" %}

### Source File

Unpack the sources to the SDK directory:

{% code overflow="wrap" %}
```bash
tar xf ./public_sources.tbz2 -C ~/Documents/nvidia/nvidia_sdk/JetPack_6.2.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/..
```
{% endcode %}

A bunch of file will appear under `~/Documents/nvidia/nvidia_sdk/JetPack_6.2.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/source`



Then, unpack the respective modules:

{% code overflow="wrap" %}
```bash
cd ~/Documents/nvidia/nvidia_sdk/JetPack_6.2.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/source/
tar xf ./kernel_src.tbz2
tar xf ./kernel_oot_modules_src.tbz2
tar xf ./nvidia_kernel_display_driver_source.tbz2

```
{% endcode %}

This extracts the kernel source to the `kernel/` subdirectory (under `kernel-jammy-src/`), and the NVIDIA out-of-tree kernel modules sources to the current directory.



### Toolchain

Unpack the toolchain to path `~/Documents/nvidia/toolchain/aarch64--glibc--stable-2022.08-1/` .





### Configure the Kernel

{% code overflow="wrap" %}
```bash
cd ~/Documents/nvidia/nvidia_sdk/JetPack_6.2.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/source/
./generic_rt_build.sh "enable"
```
{% endcode %}

```bash
nano ./kernel/kernel-jammy-src/arch/arm64/configs/defconfig
```



### Building the Jetson Linux Kernel

{% code overflow="wrap" %}
```bash
export CROSS_COMPILE=/home/tk/Documents/nvidia/toolchain/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
make -C kernel
```
{% endcode %}



{% code overflow="wrap" %}
```bash
export INSTALL_MOD_PATH=/home/tk/Documents/nvidia/nvidia_sdk/JetPack_6.2.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/rootfs/

sudo -E make install -C ./kernel/

cp ./kernel/kernel-jammy-src/arch/arm64/boot/Image /home/tk/Documents/nvidia/nvidia_sdk/JetPack_6.2.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/kernel/Image
```
{% endcode %}



### Build OOT Module

{% code overflow="wrap" %}
```bash
cd ~/Documents/nvidia/nvidia_sdk/JetPack_6.2.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/source/
```
{% endcode %}

```bash
export IGNORE_PREEMPT_RT_PRESENCE=1
```

{% code overflow="wrap" %}
```bash
export CROSS_COMPILE=/home/tk/Documents/nvidia/toolchain/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
export KERNEL_HEADERS=$PWD/kernel/kernel-jammy-src/
make modules
```
{% endcode %}

{% code overflow="wrap" %}
```bash
export INSTALL_MOD_PATH=/home/tk/Documents/nvidia/nvidia_sdk/JetPack_6.2.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/rootfs/
sudo -E make modules_install
```
{% endcode %}



{% code overflow="wrap" %}
```bash
cd ~/Documents/nvidia/nvidia_sdk/JetPack_6.2.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/
sudo ./tools/l4t_update_initrd.sh
```
{% endcode %}



### Build DTB

{% code overflow="wrap" %}
```bash
cd ~/Documents/nvidia/nvidia_sdk/JetPack_6.2.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/source/
```
{% endcode %}

{% code overflow="wrap" %}
```bash
export CROSS_COMPILE=/home/tk/Documents/nvidia/toolchain/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
export KERNEL_HEADERS=$PWD/kernel/kernel-jammy-src/
make dtbs
```
{% endcode %}

{% code overflow="wrap" %}
```bash
cp kernel-devicetree/generic-dts/dtbs/* /home/tk/Documents/nvidia/nvidia_sdk/JetPack_6.2.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/kernel/dtb/
```
{% endcode %}











Download both the Linux source and sample file system under "SOURCE" from here:

{% embed url="https://developer.nvidia.com/embedded/jetson-linux-r365" %}

| [Driver Package (BSP) Sources](https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v5.0/sources/public_sources.tbz2)                 |
| ------------------------------------------------------------------------------------------------------------------------------------------------ |
| [Sample Root Filesystem Sources](https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v5.0/sources/ubuntu_jammy-l4t_aarch64_src.tbz2) |



Also download toolchain from here:

{% embed url="https://developer.nvidia.com/embedded/jetson-linux-r365" %}

[Bootlin Toolchain gcc 11.3](https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v3.0/toolchain/aarch64--glibc--stable-2022.08-1.tar.bz2)









