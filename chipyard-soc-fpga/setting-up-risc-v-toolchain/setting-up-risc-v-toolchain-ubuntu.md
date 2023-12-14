# Setting up RISC-V Toolchain - Ubuntu

This is the recommended method.

## 1. Install RISC-V Toolchain

First, we need to install the following dependencies.

```bash
sudo apt install texinfo bison flex libgmp-dev gawk
```

Clone the RISC-V GNU Toolchain repo.

```bash
cd ~/Downloads/
git clone https://github.com/riscv-collab/riscv-gnu-toolchain.git
cd ~/Downloads/riscv-gnu-toolchain/
```

Run configuration. The prefix is where we want to install the toolchain. Here, we will be installing under the `riscv64-unknown-toolchain` directory.

{% code overflow="wrap" %}
```bash
./configure --prefix=/home/tk/Documents/RISCV/riscv64-unknown-toolchain/ --with-multilib-generator="rv32i-ilp32--;rv32im-ilp32--;rv32ima-ilp32--;rv32imac-ilp32--;rv32imafc-ilp32f--;rv64i-lp64--;rv64im-lp64--;rv64ima-lp64--;rv64imac-lp64--;rv64imaf-lp64f--;rv64imafd-lp64d--;rv64imafdc-lp64d--"
```
{% endcode %}

Build the toolchain

```bash
make
```

## 2. Install OpenOCD

First, we need to install the following dependencies.

```bash
sudo apt install libtool pkg-config
sudo apt install libusb-1.0-0-dev
sudo apt install libftdi-dev
```

Clone the RISC-V OpenOCD repo

```bash
cd ~/Documents/RISCV/
git clone https://github.com/riscv/riscv-openocd.git
cd ~/Documents/RISCV/riscv-openocd/
```



Run the following command to generate the necessary configuration and build files for OpenOCD.

```bash
./bootstrap
```

<figure><img src="../../.gitbook/assets/image (170).png" alt=""><figcaption></figcaption></figure>

Configure the build process according to the current system and settings by running the following command.

```bash
./configure
```

<figure><img src="../../.gitbook/assets/image (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>



Finally, compile and install the software

```bash
make
sudo make install
```

<figure><img src="../../.gitbook/assets/image (172).png" alt=""><figcaption></figcaption></figure>







## Common Error

### libtool: Version mismatch error when building openocd

{% embed url="https://github.com/riscv/riscv-openocd/issues/974" %}

