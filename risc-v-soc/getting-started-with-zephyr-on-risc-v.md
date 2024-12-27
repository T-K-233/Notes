# Getting Started with Zephyr on RISC-V

Using a12.mill



## Installation

Create a conda virtual environment.

```bash
conda create -yp ./.conda-env/ python=3.10
conda activate ./.conda-env/
```



Install system dependencies.

on a12.mill, cmake version is too low, so we need to install a newer one in conda environment.

```bash
conda install anaconda::cmake
```

additionally, we need these

```bash
conda install conda-forge::ninja
```







Install west, the build tool for zephyr.

```bash
pip install west
```



Create a new project.

```bash
west init ./getting-started/
cd ./getting-started/
west update
```



Export a [Zephyr CMake package](https://docs.zephyrproject.org/latest/build/zephyr_cmake_package.html#cmake-pkg). This allows CMake to automatically load boilerplate code required for building Zephyr applications.

```
west zephyr-export
```



Install Python dependencies.

```bash
pip install -r ./zephyr/scripts/requirements.txt
```





Install Zephyr SDK

```bash
cd ./zephyr/
west sdk install
```





Build blinky

```bash
west build -p always -b sparkfun_red_v_things_plus samples/basic/blinky
```



build for spike

```bash
west build -p always -b qemu_riscv64 samples/hello_world
```







on windows

[https://github.com/ninja-build/ninja/releases](https://github.com/ninja-build/ninja/releases)











The workflow roughly follows the official tutorial.

{% embed url="https://docs.zephyrproject.org/latest/develop/getting_started/index.html" %}

