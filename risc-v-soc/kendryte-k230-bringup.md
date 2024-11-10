# Kendryte K230 Bringup

## 0. Preprequisite

Running on Ubuntu 22.04 (gym.cs.berkeley)



## 1. Set up Docker environment

Install docker

```bash
sudo apt install docker.io
```



Pull the docker image

```bash
docker pull ghcr.io/kendryte/k230_sdk
```



Check if pull succeed

```bash
docker images | grep k230_sdk
```



unused:

```bash
sudo apt install flex bison libssl-dev libyaml-dev pkg-config scons
```

scons might require python to install



## 2. Set up repository

Clone and set up repository

```bash
git clone https://github.com/kendryte/k230_sdk
cd ./k230_sdk/
make prepare_sourcecode
```

the `make prepare_sourcecode` step will download Linux, RT-Smart toolchain, buildroot package, AI package etc. and will take some time. Make sure that there is no error message during the download process.



## 3. Compile K230 SDK

{% code overflow="wrap" %}
```bash
# cd ./k230_sdk/
docker run -u root -it -v $(pwd):$(pwd) -v $(pwd)/toolchain:/opt/toolchain -w $(pwd) ghcr.io/kendryte/k230_sdk /bin/bash
```
{% endcode %}



```bash
make CONF=k230_evb_defconfig
```



If encounter this error, need to apply the patch

<figure><img src="../.gitbook/assets/image (186).png" alt=""><figcaption></figcaption></figure>

```bash
wget https://raw.githubusercontent.com/keyfour/openwrt/2722d51c5cf6a296b8ecf7ae09e46690403a6c3d/tools/m4/patches/011-fix-sigstksz.patch

patch -p1 ./011-fix-sigstksz.patch
```

Reference [here](https://github.com/openwrt/openwrt/issues/9055) for more information.





## 4. Prepare bootload image

```bash
sudo dd if=sysimage-sdcard.img of=/dev/sdx bs=1M oflag=sync
```

<figure><img src="../.gitbook/assets/image (3) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>







Mount eMMC device

p4 corresponds to the app folder



```bash
mount /dev/mmcblk1p4 /mnt
cd /mnt/

```



```bash
dd if=/mnt/sysimage-sdcard.img of=/dev/mmcblk0 bs=1M oflag=sync
```







## 5. Set up Ethernet connection

Plug USB to Ethernet adapter to **USB0** port. USB1 port DOES NOT WORK.



```bash
ifconfig eth0 up
```

```bash
ifconfig eth0 10.0.0.2 netmask 255.255.255.0
```



```bash
scp tk@10.0.0.1:/home/tk/Downloads/Baremetal-NN/build/tests/tests
```







## Reference

{% embed url="https://github.com/kendryte/k230_sdk" %}
