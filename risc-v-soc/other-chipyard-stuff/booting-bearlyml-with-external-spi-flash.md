# Booting BearlyML With External SPI Flash

Connect SPI to FE310.

We need to use J-Link to communicate with the board/chip. So run `JLink.exe`

## J-Link Commands

[Here](https://wiki.segger.com/J-Link\_Commander) and [here](https://docs.rs-online.com/5cbd/0900766b8165024a.pdf) are some references. Can also type `?` in terminal to see list of command.

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

## SPI Wiring

<figure><img src="../../.gitbook/assets/image (5) (2).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (3) (1) (4) (1).png" alt=""><figcaption><p>Sparkfun FE310 Board Connection</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (6) (1) (2).png" alt=""><figcaption><p>Arty 35T Board Connection</p></figcaption></figure>

_Note: the #RESET pin need to be tied to HIGH._

<figure><img src="../../.gitbook/assets/image (7) (1) (1).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (2) (6) (1) (1).png" alt=""><figcaption></figcaption></figure>
