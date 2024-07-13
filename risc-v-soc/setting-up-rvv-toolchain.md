# Setting up RVV Toolchain

Using Millennium A12 machine



Create target directory

```bash
mkdir /scratch/tk/Documents/RISCV/
```



Clone riscv-gnu-toolchain repository

```bash
cd /scratch/tk/Downloads/
git clone https://github.com/riscv-collab/riscv-gnu-toolchain.git
cd ./riscv-gnu-toolchain/
```



To enable fp16 support, we need GCC 14.1.0

```bash
cd ./gcc/

# checkout to gcc 14.1.0 release tag
git checkout releases/gcc-14.1.0

# or do
# git checkout cd0059a1976303638cea95f216de129334fc04d1

cd ../
```



The gcc submodule is from [https://gcc.gnu.org/git/gcc.git](https://gcc.gnu.org/git/gcc.git), but here is a Github mirror of it that is more convenient to work with and reference:

{% embed url="https://github.com/gcc-mirror/gcc/tags" %}

Configure the project

```bash
./configure --prefix=/scratch/tk^Cocuments/RISCV/riscv64-unknown-too
lchain/ --with-cmodel=medany --enable-multilib
```



And build

```bash
make -j16
```









