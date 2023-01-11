# Flash Option Byte Recovery

### Default Flash Settings

```c
x/16 0x40022000
0x40022000:	0x00040603	0x00000000	0x00000000	0x00000000
0x40022010:	0x00000000	0xc0000000	0x00000000	0x00000000
0x40022020:	0xffeff8aa	0xffffffff	0x7fffc000	0xffc0ffff
0x40022030:	0xffc0ffff	0x00000000	0x00000000	0x00000000
```





### 1. change debugger to use ST-Link (OpenOCD)

<figure><img src="../../.gitbook/assets/image (4) (2).png" alt=""><figcaption></figcaption></figure>

In startup behavior, uncheck "Download" to prevent debugger from flashing Flash.&#x20;

<figure><img src="../../.gitbook/assets/image (12).png" alt=""><figcaption></figcaption></figure>







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

// launch option
set *(int *)0x40022014=(*(int *)0x40022014) | 0x08000000
x 0x40022010
// should be 0x00000000

// lock Flash
set *(int *)0x40022014=(*(int *)0x40022014) | 0x80000000


```

