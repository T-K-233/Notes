# Installing QEMU



```bash
sudo apt install ninja-build flex bison
```



```bash
git clone https://gitlab.com/qemu-project/qemu.git
cd qemu
git submodule init
git submodule update --recursive
./configure
make
```





```bash
./configure \
--target-list=riscv32-linux-user,riscv64-linux-user,riscv32-softmmu,riscv64-softmmu \
--enable-slirp
```



The `--enable-slirp` is used to enable user network backend support. See [here](https://stackoverflow.com/questions/75641274/network-backend-user-is-not-compiled-into-this-binary) for more information.

## Reference

{% embed url="https://www.qemu.org/download/" %}

