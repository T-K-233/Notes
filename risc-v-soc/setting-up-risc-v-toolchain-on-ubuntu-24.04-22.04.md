# Setting up RISC-V Toolchain on Ubuntu 24.04/22.04



Install dependencies

```bash
sudo apt install build-essential gawk texinfo bison flex libmpc-dev
```





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



To enable fp16 support, we need GCC 14.1.0. By 2024-12-29, the source has already bumped to GCC 14.2, so we don't need to do anything here.



The gcc submodule is from [https://gcc.gnu.org/git/gcc.git](https://gcc.gnu.org/git/gcc.git), but here is a Github mirror of it that is more convenient to work with and reference:

{% embed url="https://github.com/gcc-mirror/gcc/tags" %}

Configure the project

{% code overflow="wrap" %}
```bash
./configure --prefix=/scratch/tk/Documents/RISCV/riscv64-unknown-toolchain/ --with-cmodel=medany --enable-multilib
```
{% endcode %}

prefix should match the target installation directory, and must be in absolute path.



Finally, build the project

```bash
make -j16
```



The toolchain should be installed in the target directory. Add the following to \~/.bashrc to include them on PATH

```bash
export RISCV="/scratch/tk/Documents/RISCV/"
export PATH="$RISCV/riscv64-unknown-toolchain/bin/:$PATH"
```





