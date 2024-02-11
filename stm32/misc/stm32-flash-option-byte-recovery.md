# STM32 Flash Option Byte Recovery

## 1. Change debugger Settings

In **Debugger** tab, change **Debug Probe** to "ST-Link (OpenOCD)".

<figure><img src="../../.gitbook/assets/image (4) (2).png" alt=""><figcaption></figcaption></figure>

In **Startup** tab, uncheck **Download** to prevent the debugger from flashing Flash.

<figure><img src="../../.gitbook/assets/image (12) (1) (1).png" alt=""><figcaption></figcaption></figure>

## 2. Change Settings

### STM32G431

Default Flash Settings:

```c
x/16 0x40022000
0x40022000:	0x00040603	0x00000000	0x00000000	0x00000000
0x40022010:	0x00000000	0xc0000000	0x00000000	0x00000000
0x40022020:	0xffeff8aa	0xffffffff	0x7fffc000	0xffc0ffff
0x40022030:	0xffc0ffff	0x00000000	0x00000000	0x00000000
```

```c
// FLASH->CR
x 0x40022014
// should be 0xc0000000

// unlock Flash
// FLASH->KEYR
set *(int *)0x40022008=0x45670123U
set *(int *)0x40022008=0xCDEF89ABU

x 0x40022014
// should be 0x40000000

// unlock Flash Option
// FLASH->OPTKEYR
set *(int *)0x4002200C=0x08192A3BU
set *(int *)0x4002200C=0x4C5D6E7FU

x 0x40022014
// should be 0x00000000

// FLASH->OPTR
x 0x40022020
set *(int *)0x40022020=0xFBEFF8AAU

// set option start bit
set *(int *)0x40022014=(*(int *)0x40022014) | 0x00020000

// FLASH->SR
x 0x40022010
// should be 0x0000C000

// clear status register
set *(int *)0x40022010=(*(int *)0x40022010) | 0x0000C000
x 0x40022010
// should be 0x00004000

// note that for G431, we need to first launch and then lock Flash...
// launch option
set *(int *)0x40022014=(*(int *)0x40022014) | 0x08000000
x 0x40022010
// should be 0x00000000

// lock Flash
set *(int *)0x40022014=(*(int *)0x40022014) | 0x80000000

```

<figure><img src="../../.gitbook/assets/image (1) (1) (1) (1) (1) (1) (1) (1) (1) (1) (1) (1) (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>

### STM32F042

Default Option Byte Settings:

```c
// User and read protection option byte
x 0x1FFFF800
0x1ffff800:	0x00ff55aa

// User data option byte
x 0x1FFFF804
0x1ffff804:	0x00ff00ff

// Write protection option byte
x 0x1FFFF808
0x1ffff808:	0x00ff00ff
```

Default Flash Settings:

```c
x/16 0x40022000
0x40022000:	0x00000031	0x00000000	0x00000000	0x00000000
0x40022010:	0x00000000	0x00000000	0x00000000	0xffffff00
0x40022020:	0xffffffff	0x00000000	0x00000000	0x00000000
0x40022030:	0x00000000	0x00000000	0x00000000	0x00000000
```

```c
// FLASH->CR
x 0x40022010
// should be 0x00000080

// unlock Flash
// FLASH->KEYR
set *(int *)0x40022004=0x45670123U
set *(int *)0x40022004=0xCDEF89ABU

x 0x40022010
// should be 0x00000000

// unlock Flash Option
// FLASH->OPTKEYR
set *(int *)0x40022008=0x45670123U
set *(int *)0x40022008=0xCDEF89ABU

x 0x40022010
// should be 0x00000200

/* Option Erase */

// Set the OPTER bit in the FLASH_CR register to enable option byte erasing
set *(int *)0x40022010 |= 1U<<5U

x 0x40022010
// should be 0x00000220

// Set the STRT bit in the FLASH_CR register to start the erasing */
set *(int *)0x40022010 |= 1U<<6U

x 0x4002200C
// should be 0x00000024

// Clear EOP flag by software by writing EOP at 1
set *(int *)0x4002200C = 1U<<5U
x 0x4002200C
// should be 0x00000004
x 0x1FFFF800
// should be 0xffffffff

set *(int *)0x40022010 &= ~(1U<<5U)
x 0x40022010
// should be 0x00000200

/* Option Programming */

// Set the OPTPG bit in the FLASH_CR register to enable programming
set *(int *)0x40022010 |= 1U<<4U
x 0x40022010
// should be 0x00000210

// set user option
set *(uint16_t *)0x1FFFF802 = 0x807F
set *(uint16_t *)0x1FFFF800 = 0x55AA
x 0x1FFFF800
// should be 0x807f55aa

set *(uint16_t *)0x1FFFF806 = 0x00FF
set *(uint16_t *)0x1FFFF804 = 0x00FF
x 0x1FFFF804

set *(uint16_t *)0x1FFFF80A = 0x00FF
set *(uint16_t *)0x1FFFF808 = 0x00FF
x 0x1FFFF808

set *(int *)0x40022010 &= ~(1U<<4U)
x 0x40022010
// should be 0x00000200

// lock option
set *(int *)0x40022010 &= ~(1U<<9U)
x 0x40022010
// should be 0x00000000

// lock Flash
set *(int *)0x40022010 |= 1U<<7U
x 0x40022010
// should be 0x00000080

// launch option
set *(int *)0x40022010 |= 1U<<13U




```
