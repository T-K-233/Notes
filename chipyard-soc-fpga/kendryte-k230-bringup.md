# Kendryte K230 Bringup

## 1. Prerequisites

```bash
sudo apt install flex bison libssl-dev libyaml-dev pkg-config scons

```

scons might require python to install





```bash
git clone https://github.com/kendryte/k230_sdk
cd k230_sdk
make prepare_sourcecode
```





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







## Reference

{% embed url="https://github.com/kendryte/k230_sdk" %}
