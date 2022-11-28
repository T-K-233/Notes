# Booting BearlyML With External SPI Flash

Connect SPI to FE310.

We need to use J-Link to communicate with the board/chip. So run `JLink.exe`



To connect device

```
connect FE310
```



List Regs

```
Regs
RReg pc
```



Examine memory content

```
Mem32 0x20000000, 2
```



Erase Flash

```
Erase 0x20000000, 0x20001000
```



Load file to Flash

```
LoadFile C:\Users\TK\Desktop\HAL\firmware\build\firmware.bin, 0x20000000
```



Common commands as GDB

```
Halt
Go
Reset
Step
```
