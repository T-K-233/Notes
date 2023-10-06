# Setting up RISC-V Toolchain - Windows

## 1. Install Make.

Follow the tutorial Install Make on Windows.

{% content-ref url="../../tools/windows/install-make-on-windows.md" %}
[install-make-on-windows.md](../../tools/windows/install-make-on-windows.md)
{% endcontent-ref %}



## 2. Download RISC-V Toolchain

Download RISC-V toolchain from [here](https://www.sifive.com/software). We are going to need the toolchain and OpenOCD.

> Update: as of 2023-05-22, SiFive changed their website, and the OpenOCD is no longer available. Use [this link](https://static.dev.sifive.com/dev-tools/riscv-openocd-0.10.0-2020.04.6-x86\_64-w64-mingw32.zip?\_ga=2.194046829.1921518046.1596069236-38171805.1596069236) to download an older version (openocd-0.10.0-2020.04.6).

> If the link above still does not work, here is the archive links:
>
> [riscv-openocd-0.10.0-2020.12.1-x86\_64-w64-mingw32.zip](https://github.com/ucb-ee290c/ucb-ee290c.github.io/releases/download/RISC-V\_toolchain\_win/riscv-openocd-0.10.0-2020.12.1-x86\_64-w64-mingw32.zip)
>
> [riscv64-unknown-elf-toolchain-10.2.0-2020.12.8-x86\_64-w64-mingw32.zip](https://github.com/ucb-ee290c/ucb-ee290c.github.io/releases/download/RISC-V\_toolchain\_win/riscv64-unknown-elf-toolchain-10.2.0-2020.12.8-x86\_64-w64-mingw32.zip)



Extract them to known locations, and put the path to system PATH.

```
D:\Documents\RISCV\riscv64-unknown-elf-toolchain-10.2.0-2020.12.8-x86_64-w64-mingw32\bin
```

```
D:\Documents\RISCV\riscv-openocd-0.10.0-2020.12.1-x86_64-w64-mingw32\bin
```





