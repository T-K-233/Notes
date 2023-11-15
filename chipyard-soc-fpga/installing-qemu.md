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
./configure --target-list=riscv32-linux-user,riscv64-linux-user,riscv32-softmmu,riscv64-softmmu
```





## Reference

{% embed url="https://www.qemu.org/download/" %}

