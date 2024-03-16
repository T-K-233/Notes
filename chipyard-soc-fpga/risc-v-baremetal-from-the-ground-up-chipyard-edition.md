---
description: >-
  This article will walk you through the behind-the-scenes of how a baremetal C
  program is compiled and linked as a RISC-V binary file.
---

# RISC-V: Baremetal From The Ground Up (Chipyard Edition)



Let's start with something simple. The "hello world" equivalent program in the embedded systems world would be the blinkly LED program:

```c
// some handy macros to do bit operations
#define SET_BITS(REG, BIT)              ((REG) |= (BIT))
#define CLEAR_BITS(REG, BIT)            ((REG) &= !(BIT))

// peripheral MMIO addresses
#define GPIOA_OUTPUT_VAL                0x1001000CUL
#define GPIOA_OUTPUT_EN                 0x10010008UL
#define CLINT_MTIME                     0x0200BFF8UL

// the pin we are using
const unsigned int GPIO_PIN = 0x01;

// a global counter
volatile unsigned int counter;

// A simple delay function. 
void delay(unsigned int ticks) {
  unsigned int mtime_start;
  while ((*(volatile unsigned int *)CLINT_MTIME) - mtime_start < ticks) {}
}

void main() {
  // enable GPIOA as output
  SET_BITS(*(volatile unsigned int *)GPIOA_OUTPUT_EN, GPIO_PIN);

  while (1) {
    // if counter is even, turn on the LED, otherwise turn it off
    if (counter % 2 == 0) {
      SET_BITS(*(volatile unsigned int *)GPIOA_OUTPUT_VAL, GPIO_PIN);
    } else {
      CLEAR_BITS(*(volatile unsigned int *)GPIOA_OUTPUT_VAL, GPIO_PIN);
    }

    // delay for 1 second
    delay(1000);
    
    // increment the counter
    counter += 1;
  }

  // we won't reach here if everything is working
}

```



This might look intimidating. Let's break down the elements:

First, we define `SET_BITS` and `CLEAR_BITS` as macro functions. These will become handy, since when operating Memory-Mapped Input/Output (MMIO) registers, in most cases we are operating on a bit-level, only touching fields that we are focused on and leave the rest bits intact.



Then, we define `GPIOA_OUTPUT_VAL`, `GPIOA_OUTPUT_EN`, and `CLINT_MTIME`. These are the memory address of the corresponding MMIO registers.



Followed by that, we write our first actual line of C program, which defines `GPIO_PIN` as a constant.  We also define a global variable called `counter`.



{% hint style="info" %}
**Note:**

We only defined the address of MMIO registers that we are going to use here. And to demonstrate the read-only data section, we delibriately define the `GPIO_PIN`as a global constant instead of macro.

In a more proper program, these elements are defined in a slightly different way (see [CLINT](https://github.com/ucb-bar/Baremetal-IDE-bsp/blob/main/common/inc/ll\_clint.h#L12), [GPIO](https://github.com/ucb-bar/Baremetal-IDE-bsp/blob/main/common/inc/ll\_gpio.h#L12), and [HAL\_GPIO](https://github.com/ucb-bar/Baremetal-IDE-bsp/blob/main/common/inc/hal\_gpio.h#L22) in Baremetal-IDE as an example)
{% endhint %}

Then, we define `delay()`, which reads from the `mtime` register in CLINT to keep track of the time, and halt the program for a given amount of `ticks`. &#x20;



{% hint style="info" %}
**Note:**

Note that we are using ticks, instead of a physical unit of time like seconds or milliseconds, in the delay function. This is because without special circuits such as Real-time Clock (RTC), the SoC does not have a sense of how fast the real-world wall clock. The ratio between ticks and seconds is determined by the input clock frequency and the internal clock tree settings of the SoC.
{% endhint %}



Finally, we move to the `main()` function. Inside the function, we first enable the GPIO output functionality, and then proceed to an infinite loop. Inside the loop, we toggle the LED every time the loop restarts, delay for 1000 ticks, and then increment the counter.

{% hint style="info" %}
**Note:**

You might be more familiar with the main that looks like this:

```c
int main(int argc, char* argv[]) {}
```

This is because in embedded systems, main normally would be an infinite loop and will not return. It does not make sense for an embedded program to "exit from main()", since there is no additional code past main().&#x20;
{% endhint %}



In order to compile this C program to something our SoC can understand, we need to use the RISC-V Toolchain.



## RISC-V Toolchain

The RISC-V Toolchain is a collection of executables that helps us to compile, assemble, and link the program we write in C/C++ to binary format. It can also provide tools for us to debug and analyze the generated binaries.

There is a wide range of choices of toolchains, usually marked by different prefixes. The following is a simple list of the common ones that we may encounter:

* TODO

Here, we will use the [riscv-gnu-toolchain](https://github.com/riscv-collab/riscv-gnu-toolchain) from [riscv-collab](https://github.com/riscv-collab) (it comes with the prefix `riscv64-unknown-elf-`).

For toolchain installation, see [Setting up RISC-V Toolchain](https://ucb-bar.gitbook.io/chipyard/quickstart/setting-up-risc-v-toolchain).

In the toolchain directory, we can see a set of executables:

`-gcc` is the most general one. You can consider it as the entry executable which can invoke the compiler, the linker, and the assembler by passing it different compiler flags.

`-ar` is the assembler itself.

`-ld` is the linker itself.

`-objdump`, `-readelf`, and `-nm` are elf file analyzers.

`-objcopy` is the format converter. It can convert between elf format, binary, hex, and many other.

All of these toolchain executables will run on the host machine, but it knows the architecture of the target SoC, and thus can build the binary in the format that our target can understand.



## Build Process

Now let's dive into the build process. There are several stages in the build process. Normally, the toolchain will join several stages together to speed up the build process. Here, we pass special flags to the toolchain to let it stop at each stage, so we can take a look at the intermediate contents.



### Pre-processing Stage

<figure><img src="../.gitbook/assets/image (2) (5).png" alt=""><figcaption></figcaption></figure>

The first stage is the pre-processing stage.&#x20;

In this stage, the compiler will resolve all the [compiler macros](https://gcc.gnu.org/onlinedocs/cpp/Macros.html) (basically, everything we defined with "#" marks).

By default, the compiler will not generate this intermediate "main.i" file for us. To do this, we will pass the `-E` argument to tell the compiler stop after pre-processing. We use the `-o` argument to specify the output file.

```bash
riscv64-unknown-elf-gcc -E -o ./main.i ./main.c
```

We can see that in `main.i`, all of the macro defines are processed and replaced with their definition contents.

```c
# 1 "./main.c"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "./main.c"
# 11 "./main.c"
const unsigned int GPIO_PIN = 0x01;


volatile unsigned int counter;


void delay(unsigned int ticks) {
  unsigned int mtime_start;
  while ((*(volatile unsigned int *)0x0200BFF8UL) - mtime_start < ticks) {}
}

void main() {

  ((*(volatile unsigned int *)0x10010008UL) |= (GPIO_PIN));

  while (1) {

    if (counter % 2 == 0) {
      ((*(volatile unsigned int *)0x1001000CUL) |= (GPIO_PIN));
    } else {
      ((*(volatile unsigned int *)0x1001000CUL) &= !(GPIO_PIN));
    }


    delay(1000);


    counter += 1;
  }


}

```



### Code Generation Stage

<figure><img src="../.gitbook/assets/image (16) (3).png" alt=""><figcaption></figcaption></figure>

The "main.i" file is then passed through the compiler again for the code-generation stage.

In this stage, all of the high-level C/C++ code will be converted to architecture-specific assembly language.

Similarly, the compiler will not generate this intermediate file for us, and we need to use the `-S` argument to command the compiler stop after code-generation.

```bash
riscv64-unknown-elf-gcc -S -o ./main.S ./main.c
```



The resulting file is our familiar RISC-V assembly code.

```armasm
	.file	"main.c"
	.option nopic
	.attribute arch, "rv64i2p0_m2p0_a2p0_f2p0_d2p0_c2p0"
	.attribute unaligned_access, 0
	.attribute stack_align, 16
	.text
	.globl	GPIO_PIN
	.section	.srodata,"a"
	.align	2
	.type	GPIO_PIN, @object
	.size	GPIO_PIN, 4
GPIO_PIN:
	.word	1
	.globl	counter
	.section	.sbss,"aw",@nobits
	.align	2
	.type	counter, @object
	.size	counter, 4
counter:
	.zero	4
	.text
	.align	1
	.globl	delay
	.type	delay, @function
delay:
	addi	sp,sp,-48
	sd	s0,40(sp)
	addi	s0,sp,48
	mv	a5,a0
	sw	a5,-36(s0)
	nop
.L2:
	li	a5,33603584
	addi	a5,a5,-8
	lw	a5,0(a5)
	sext.w	a4,a5
	lw	a5,-20(s0)
	subw	a5,a4,a5
# ...
# more contents omitted
```



### Assembling Stage

<figure><img src="../.gitbook/assets/image (15) (4).png" alt=""><figcaption></figcaption></figure>

At the assembling stage, the assembly language will be further converted into binary instructions.

The output file is also called "relocatable object file". The word “relocatable” indicates that the addresses in the program (where to put each piece of code in the memory) are not determined yet.

Same as before, we need to supply the `-c` flag to prevent compiler proceed to linking stage.

<pre class="language-bash"><code class="lang-bash"><strong>riscv64-unknown-elf-gcc -c -o ./main.o ./main.c
</strong></code></pre>



The format of the relocatable object file is in Executable and Linkable Format (ELF). Since it's a binary format, we cannot examine the content directly with text editor anymore, so we need the toolchain to decode the content.&#x20;



### Analyzing Relocatable Object Files <a href="#id-2dca" id="id-2dca"></a>

There's still one last stage (linking stage) remaining, but let's take a side track here and examine the content of the generated "main.o" file first.

The ELF format describes how various elements of the code (e.g. code, data, read-only data, uninitialized data) are located in different sections.

We will use the `riscv-unknown-elf-objdump` to analyze our program



### Display Section Headers <a href="#id-593d" id="id-593d"></a>

Let's first examine the section headers in main.o.

By running objdump with `-h` argument, we can print out all the section headers in an ELF file.

```bash
riscv64-unknown-elf-objdump -h ./main.o
```



```bash

./main.o:     file format elf64-littleriscv

Sections:
Idx Name          Size      VMA               LMA               File off  Algn
  0 .text         000000d4  0000000000000000  0000000000000000  00000040  2**1
                  CONTENTS, ALLOC, LOAD, RELOC, READONLY, CODE
  1 .data         00000000  0000000000000000  0000000000000000  00000114  2**0
                  CONTENTS, ALLOC, LOAD, DATA
  2 .bss          00000000  0000000000000000  0000000000000000  00000114  2**0
                  ALLOC
  3 .srodata      00000004  0000000000000000  0000000000000000  00000114  2**2
                  CONTENTS, ALLOC, LOAD, READONLY, DATA
  4 .sbss         00000004  0000000000000000  0000000000000000  00000118  2**2
                  ALLOC
  5 .comment      00000031  0000000000000000  0000000000000000  00000118  2**0
                  CONTENTS, READONLY
  6 .riscv.attributes 00000035  0000000000000000  0000000000000000  00000149  2**0
                  CONTENTS, READONLY
```

`.text` section holds the code of the program.

`.data` section holds the initialized global data.

`.bss` section holds the uninitialized global data. The actual memory mapped with this section will be reset to zero by the program boot code. The name "bss" stands for "block starting symbol", and is chosen due to historical reasons. Due to the RISC-V compiler's default setting, it's also generating a `.sbss` section, which stands for "small .bss data" and decides to put our `counter` variable there.&#x20;

`.rodata` section holds the read-only data. Due to the RISC-V compiler's default setting, it instead generats a `.srodata` section, which stands for "small .rodata data" and decides to put our `GPIO_PIN` constant there.&#x20;

`.comment` and `.riscv.attributes` are sections added by the compiler for debugging purposes.



Note that all the sections start at address 0x00. This is the reason why `.o` files are called relocatable. All of the addresses are relative, and it will be during the linking stage to let linker to convert these relative addresses into absolute locations.



### Display Full Content <a href="#id-73a7" id="id-73a7"></a>

With `-s` argument, we can print out the full content of the ELF file. . The result will be large, so we redirect the output to a file.

```bash
riscv64-unknown-elf-objdump -s ./main.o
```

```log
./main.o:     file format elf64-littleriscv

Contents of section .text:
 0000 797122f4 0018aa87 232ef4fc 0100b7c7  yq".....#.......
 0010 0002e117 9c431b87 07008327 c4febb07  .....C.....'....
 0020 f7401b87 07008327 c4fd8127 e361f7fe  .@.....'...'.a..
 0030 01000100 22744561 82804111 06e422e0  ...."tEa..A...".
 0040 0008b707 0110a107 9c439b86 07000547  .........C.....G
 0050 b7070110 a107558f 012798c3 b7070000  ......U..'......
 0060 83a70700 8127858b 812799ef b7070110  .....'...'......
 0070 b1079c43 9b860700 0547b707 0110b107  ...C.....G......
 0080 558f0127 98c325a0 b7070110 b1079c43  U..'..%........C
 0090 9b860700 854793b7 170093f7 f70f1b87  .....G..........
 00a0 0700b707 0110b107 758f0127 98c31305  ........u..'....
 00b0 803e9700 0000e780 0000b707 000083a7  .>..............
 00c0 07008127 85271b87 0700b707 000023a0  ...'.'........#.
 00d0 e70069b7                             ..i.            
Contents of section .srodata:
 0000 01000000                             ....            
Contents of section .comment:
 0000 00474343 3a202853 69466976 65204743  .GCC: (SiFive GC
 0010 432d4d65 74616c20 31302e32 2e302d32  C-Metal 10.2.0-2
 0020 3032302e 31322e38 29203130 2e322e30  020.12.8) 10.2.0
 0030 00                                   .               
Contents of section .riscv.attributes:
 0000 41340000 00726973 63760001 2a000000  A4...riscv..*...
 0010 04100572 76363469 3270305f 6d327030  ...rv64i2p0_m2p0
 0020 5f613270 305f6632 70305f64 3270305f  _a2p0_f2p0_d2p0_
 0030 63327030 00                          c2p0.           

```



### Display Disassembly <a href="#f5e0" id="f5e0"></a>

With `-d` argument, we can print out the disassembly code from the text section

```
riscv64-unknown-elf-objdump -d main.o > main.log
```



```log
main.o:     file format elf64-littleriscv


Disassembly of section .text:

0000000000000000 <delay>:
   0:	7179                	addi	sp,sp,-48
   2:	f422                	sd	s0,40(sp)
   4:	1800                	addi	s0,sp,48
   6:	87aa                	mv	a5,a0
   8:	fcf42e23          	sw	a5,-36(s0)
   c:	0001                	nop

000000000000000e <.L2>:
   e:	0200c7b7          	lui	a5,0x200c
  12:	17e1                	addi	a5,a5,-8
  14:	439c                	lw	a5,0(a5)
  16:	0007871b          	sext.w	a4,a5
  1a:	fec42783          	lw	a5,-20(s0)
  1e:	40f707bb          	subw	a5,a4,a5
# ...
# more contents omitted
```





### Linking Stage

<figure><img src="../.gitbook/assets/image (1) (1) (3).png" alt=""><figcaption></figcaption></figure>

This is the final stage before we can get an executable binary program.

The linker will put different pieces of code and data to our desired address locations, resolve all the not-yet-defined symbols, and merge all the programs and external libraries into a single file.



We need to tell the linker how we want the program to be linked together, and that is through the use of a **linker script**.&#x20;

Linker scripts are written in linker commands, with the file extension `.ld`.



### Linker Commands <a href="#id-98c0" id="id-98c0"></a>

`ENTRY` defines the entry point of the program. It is the first piece of code the MCU will execute. Debugger will also set the initial PC location according to this entry value.

Syntax of the ENTRY command is shown below, where `entry_symbol_name` is the name of the entry function.

```linker-script
ENTRY(entry_symbol_name)
```

`MEMORY` defines the various memory regions in the MCU and provides info of their locations and sizes. Linker also calculates the total code size and memory usage from this value to determine if the program can fit inside the memory.

Syntax of the MEMORY command is shown below.

```linker-script
MEMORY {
    name_of_memory_section(attribute): ORIGIN = origin, LENGTH = length
}
```

The attribute is defined as follows:

`R` Read-only sections

`W` Read and write sections

`X` Sections containing executable code

`A` Allocated sections

`I` Initialized sections

`L` Initialized sections, same as I

`!` Invert the meaning of the following symbols

`SECTION` defines which symbol sections are mapped to which memory regions, as well as the order of the mapping. It will generate the defined sections in the final ELF file. For example, we can map `.text` section to `FLASH` region.

Syntax of the SECTION command is shown below.

```linker-script
SECTIONS {
  .text : {
    // sections
  }> virtual_memory_address AT> load_memory_address
  .data : {
  }> virtual_memory_address 
}
```

When virtual memory address and load memory address are the same, we only need to write the virtual memory address.

### Writing Linker Script <a href="#id-98f3" id="id-98f3"></a>

For the sake of simplicity and ease of understanding, for now we will not care about the C runtime hassles and interrupt routines. We will make our program enter directly to main, and start to run our blinky LED program.

Thus, the **entry** symbol of our program will just be `main`

```linker-script
ENTRY(main)
```



In Chipyard tutorial SoC design, we have three memory regions

```linker-script
MEMORY {
  SCRATCH(rwx): ORIGIN = 0x08000000, LENGTH = 128K
  DRAM(rwx): ORIGIN = 0x80000000, LENGTH = 4096M
}
```



To keep things simple, we will stack every section on top of each other on scratchpad memory.

```linker-script
SECTIONS {
  .text : {
    *(.text)
  }> SCRATCH
  .rodata : {
    *(.rodata)
    *(.srodata)
  }> SCRATCH
  .data : {
    *(.data)
  }> SCRATCH
  .bss : {
    *(.bss)
    *(.sbss)
  }> SCRATCH
}

```



Now we have our unsafe-but-usable linker script:

```linker-script
ENTRY(main)

MEMORY {
  SCRATCH(rwx): ORIGIN = 0x08000000, LENGTH = 128K
  DRAM(rwx): ORIGIN = 0x80000000, LENGTH = 4096M
}

SECTIONS {
  .text : {
    *(.text)
  }> SCRATCH
  .rodata : {
    *(.rodata)
    *(.srodata)
  }> SCRATCH
  .data : {
    *(.data)
  }> SCRATCH
  .bss : {
    *(.bss)
    *(.sbss)
  }> SCRATCH
}

```



Finally, we are ready for linking.



With `-T` argument, we can tell gcc to link the target programs.

Also for simplicity, we are not going to link the standard C library for now. To do that, we are adding the `-nostdlib` argument.

```bash
riscv64-unknown-elf-gcc -nostdlib -T ./linker.ld -o ./main.elf ./main.o
```



### Format Converison

<figure><img src="../.gitbook/assets/image (18) (4).png" alt=""><figcaption></figcaption></figure>



### Loading the Program

TODO







## Startup Code

Our LED has successfully blinked. However, if we try running other more complex programs, they might fail. This is because we have made a lot of assumptions about the state of the SoC when we enter the main() function.

This is usually set up with a startup file. This piece of the program will be responsible for setting up the interrupt vector, initializing the stack, zeroing out the `.bss` section, and sometimes also copying the `.data` section to SRAM. Hence, we will write our own startup file to properly initialize the SoC.

// TODO: change

Boot Flow:

1. The program starts at the BootROM \`path\`.
2. Jump to the entry point, which is at the label: \_enter in `freedom-metal/src/entry.S`.
3. Initialize global pointer gp register using the generated symbol `__global_pointer$`.
4. Write mtvec register with early\_trap\_vector as default exception handler.
5. Read mhartid into register a0 and call \_start, which exists in crt0.S.
6. Initialize stack pointer, sp, with \_sp generated symbol. Harts with mhartid of one or larger are offset by (\_sp + \_\_stack\_size \* mhartid). The \_\_stack\_size field is generated in the linker file.
7. Check if mhartid == \_\_metal\_boot\_hart and run the init code if they are equal. All other harts skip init and go to the Post Init Flow, step #15.
8. Boot Hart Init Flow Begins Here
9. Init data section to destination in defined RAM space
10. Copy ITIM section, if ITIM code exists, to destination
11. Zero out bss section
12. Call atexit library function which registers the libc and freedom-metal destructors to run after main returns
13. Call \_\_libc\_init\_array library function, which runs all functions marked with **attribute**((constructor)).
14. Post Init Flow Begins Here
15. Call the C routine \_\_metal\_synchronize\_harts, where hart 0 will release all harts once their individual msip bits are set. The msip bit is typically used to assert a software interrupt on individual harts, however interrupts are not yet enabled, so msip in this case is used as a gatekeeping mechanism
16. Check misa register to see if floating point hardware is part of the design, and set up mstatus accordingly.
17. Single or multi-hart design redirection step
18. If design is a single hart only, or a multi-hart design without a C-implemented function secondary\_main, ONLY the boot hart will continue to main(). b. For multi-hart designs, all other CPUs will enter sleep via WFI instruction via the weak secondary\_main label in crt0.S, while boot hart runs the application program. c. In a multi-hart design which includes a C-defined secondary\_main function, all harts will enter secondary\_main as the primary C function.

### Interrupt Vector

### Stack Initialization

\_\_stack\_size

\_\_boot\_hart\_idx

\_\_global\_pointer$

\_sp: Address of the end of stack for hart 0, used to initialize the beginning of the stack since the stack grows lower in memory. On a multi-hart system, the start address of the stack for each hart is calculated using (\_sp + \_\_stack\_size \* mhartid)

metal\_segment\_bss\_target\_start & metal\_segment\_bss\_target\_end ◦ Used to zero out global data mapped to .bss section

metal\_segment\_data\_source\_start, metal\_segment\_data\_target\_start, metal\_segment\_data\_target\_end ◦ Used to copy data from image to its destination in RAM.

metal\_segment\_itim\_source\_start, metal\_segment\_itim\_target\_start, metal\_segment\_itim\_target\_end ◦ Code or data can be placed in itim sections using the \_\_attribute\_\_section(".itim")
