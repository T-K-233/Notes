# Debugging BearlyML with JTAG and GDB

## 1. VS Code Method (Preferred)

In VSCode, search and install the "cortex-debug" extension.

<figure><img src="../../.gitbook/assets/image (8) (3).png" alt=""><figcaption></figcaption></figure>

in the `.vscode` folder, create `launch.json`

```json
{
  // Use IntelliSense to learn about possible attributes.
  // Hover to view descriptions of existing attributes.
  // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
  "version": "0.2.0",
  "configurations": [
    {
      "name": "RISC-V Debug",
      "type": "cortex-debug",
      "cwd": "${workspaceRoot}",
      "executable": "${workspaceRoot}/build/firmware.elf",
      "request": "launch",
      "servertype": "openocd",
      "serverpath": "openocd",
      "toolchainPrefix": "riscv64-unknown-elf",
      "configFiles": [
        "${workspaceRoot}/debug/bearlyml.cfg"
      ],
      "searchDir": [],
      "runToEntryPoint": "main",
      "showDevDebugOutput": "none""svdFile": "${workspaceRoot}/debug/bearlyml.svd",
      "numberOfProcessors": 5,
      "currentProcessor": 1
    }
  ]
}
```

Launch the RISC-V Debug routine.

<figure><img src="../../.gitbook/assets/image (3) (1) (2).png" alt=""><figcaption></figcaption></figure>

## 2. Terminal-Only Method

Terminal A:

```bash
openocd -f .\debug\bearlyml.cfg
```

```bash
PS C:\Users\TK\Desktop\HAL> openocd -f .\debug\bearlyml.cfg
Open On-Chip Debugger 0.11.0-rc1+dev (SiFive OpenOCD 0.10.0-2020.12.1)   
Licensed under GNU GPL v2
For bug reports:
        https://github.com/sifive/freedom-tools/issues
Info : clock speed 4000 kHz
Info : JTAG tap: riscv.cpu tap/device found: 0x00000001 (mfg: 0x000 (<invalid>), part: 0x0000, ver: 0x0)
Info : datacount=2 progbufsize=16
Info : Disabling abstract command reads from CSRs.
Info : Examined RISC-V core; found 5 harts
Info :  hart 0: XLEN=64, misa=0x800000000094112d
Info : starting gdb server for riscv.cpu0 on 3333
Info : Listening on port 3333 for gdb connections
Ready for Remote Connections
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections

```

Terminal B:

```bash
cd firmware
riscv64-unknown-elf-gdb.exe .\build\firmware.elf
```

```bash
PS C:\Users\TK\Desktop\HAL\firmware> riscv64-unknown-elf-gdb.exe .\build\firmware.elf
D:\Documents\RISCV\riscv64-unknown-elf-toolchain-10.2.0-2020.12.8-x86_64-w64-mingw32\bin\riscv64-unknown-elf-gdb.exe: warning: Couldn't determine a path for the index cache directory.
GNU gdb (SiFive GDB-Metal 10.1.0-2020.12.7) 10.1
Copyright (C) 2020 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.      
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "--host=x86_64-w64-mingw32 --target=riscv64-unknown-elf".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<https://github.com/sifive/freedom-tools/issues>.
Find the GDB manual and other documentation resources online at:        
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from .\build\firmware.elf...
(gdb) 
```

#### Connect to target

```bash
(gdb) target extended-remote localhost:3333
```

#### Reset chip

```bash
(gdb) monitor reset
JTAG tap: riscv.cpu tap/device found: 0x00000001 (mfg: 0x000 (<invalid>), part: 0x0000, ver: 0x0)
```

#### Load program

```bash
(gdb) load
Loading section .text, size 0x3a08 lma 0x20000000
Loading section .rodata, size 0x680 lma 0x20003a08
Loading section .data, size 0x350 lma 0x20004088
Loading section .sdata, size 0x18 lma 0x200043d8
Start address 0x00000000200000a8, load size 17392
Transfer rate: 3 KB/sec, 4348 bytes/write.
```

#### Set breakpoint

```bash
(gdb) break main
Breakpoint 1 at 0x200003e0: file core/src/main.c, line 150.
Note: automatically using hardware breakpoints for read-only addresses.gs
```

#### List current program

```bash
(gdb) list
146     int main() {
147       UART_InitTypeDef UART_init_config;
148       UART_init_config.baudrate = 115200;
149       UART_init_config.mode = UART_MODE_TX_RX;
150       UART_init_config.stopbits = UART_STOPBITS_1;
151       HAL_UART_init(UART0, &UART_init_config);
152
153       GPIO_InitTypeDef GPIO_init_config;
154       GPIO_init_config.mode = GPIO_MODE_OUTPUT;
155       GPIO_init_config.pull = GPIO_PULL_NONE;
```

#### Step

```bash
(gdb) n 
_start () at bsp/bearlyml/startup/bearlyml_startup.S:108
108       li x2, 0
```
