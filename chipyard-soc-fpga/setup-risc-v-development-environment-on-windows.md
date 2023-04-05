# Setup RISC-V Development Environment on Windows

## 1. Download MSYS2.

Download MSYS2 from [here](https://www.msys2.org/).



After installation, in the MSYS2 terminal, enter

```
pacman -S make gettext base-devel
```



Add the following path to the system PATH variable.

```
D:\Documents\MSYS2\usr\bin
```

_Note: the exact path will be different depending on the installation location._



## 2. Download RISC-V Toolchain

Download RISC-V toolchain from [here](https://www.sifive.com/software). We are going to need the toolchain and OpenOCD.



Extract them to known locations, and put the path to system PATH.

```
D:\Documents\RISCV\riscv64-unknown-elf-toolchain-10.2.0-2020.12.8-x86_64-w64-mingw32\bin
```

```
D:\Documents\RISCV\riscv-openocd-0.10.0-2020.12.1-x86_64-w64-mingw32\bin
```





