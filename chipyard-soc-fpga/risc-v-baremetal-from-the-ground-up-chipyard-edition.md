# RISC-V: Baremetal From The Ground Up (Chipyard Edition)

Let's start with something simple. The "hello world" equivalent program in the embedded systems world would be the blinkly LED program.

```c
#define SET_BITS(REG, BIT)              ((REG) |= (BIT))
#define CLEAR_BITS(REG, BIT)            ((REG) &= !(BIT))

#define GPIOA_BASE                      0x10010100UL

volatile unsigned int counter;

void simple_delay() {
  for (int i=0; i<1000; i+=1) {
    asm volatile("nop");
  }
}

int main() {
  *(unsigned int volatile *)(GPIOA_BASE + 0x00) = 0x01;

  while (1) {
    SET_BITS(*(unsigned int volatile *)(GPIOA_BASE + 0x00), 0x01);
    simple_delay();
    
    CLEAR_BITS(*(unsigned int volatile *)(GPIOA_BASE + 0x00), 0x01);
    simple_delay();
    
    counter += 1;
  }
  return 0;
}

```

Embedded programs, most of the time, are all about MMIO register manipulations.

Now, in order to compile this C program to something our SoC can understand, we need to use the RISC-V Toolchain.



## RISC-V Toolchain

The RISC-V Toolchain is a collection of executables that helps us to compile, assemble, and link the program we write in C/C++ to binary format. It can also provide tools for us to debug and analyze the generated binaries.

There is a wide range of choices of toolchains, usually marked by different prefixes. The following is a simple list of the common ones that we may encounter:

* TODO

Here, we will use the [riscv-gnu-toolchain](https://github.com/riscv-collab/riscv-gnu-toolchain) from [riscv-collab](https://github.com/riscv-collab) (it comes with the prefix `riscv64-unknown-elf-`).

For toolchain installation, see \[TODO link]



After installation, we can see that we have a set of executables in the installation directory.

`-gcc` is the most general one. You can consider it as the entry executable which can invoke the compiler, the linker, and the assembler by passing it different compiler flags.

`-ar` is the assembler itself.

`-ld` is the linker itself.

`-objdump`, `-readelf`, and `-nm` are elf file analyzers.

`-objcopy` is the format converter. It can convert between elf format, binary, hex, and many other.

All of these toolchain executables will run on the host machine, but it knows the architecture of the target SoC, and thus can build the binary in the format that our target can understand.



## Build Process

### Pre-processing Stage

<figure><img src="../.gitbook/assets/image (2) (5).png" alt=""><figcaption></figcaption></figure>

The first stage is the pre-processing stage. In this stage, the compiler will resolve all the [compiler macros](https://gcc.gnu.org/onlinedocs/cpp/Macros.html) (basically, everything we defined with "#" marks).&#x20;



By default, the compiler will not generate this intermediate "main.i" file for us. To do this, we will pass the `-E` argument to tell the compiler stop after pre-processing. We use the `-o` argument to specify the output file.

> Note: because the toolchain can be installed on different paths on different machines, we will represent the path to the toolchain with `$RISCV` environment variable.

```bash
$RISCV/riscv64-unknown-elf-gcc -E -o main.i main.c
```



We can see that in `main.i`, all of the macro defines are processed and replaced with their actual contents.

```c
# 0 "main.c"
# 0 "<built-in>"
# 0 "<command-line>"
# 1 "main.c"





volatile unsigned int counter;

void simple_delay() {
  for (int i=0; i<1000; i+=1) {
    asm volatile("nop");
  }
}

int main() {
  *(unsigned int volatile *)(0x10010100UL + 0x00) = 0x01;

  while (1) {
    ((*(unsigned int volatile *)(0x10010100UL + 0x00)) |= (0x01));
    simple_delay();

    ((*(unsigned int volatile *)(0x10010100UL + 0x00)) &= !(0x01));
    simple_delay();

    counter += 1;
  }
  return 0;
}

```



### Code Generation Stage

<figure><img src="../.gitbook/assets/image (16) (3).png" alt=""><figcaption></figcaption></figure>





### Assembling Stage

<figure><img src="../.gitbook/assets/image (15) (4).png" alt=""><figcaption></figcaption></figure>



### Linking Stage



<figure><img src="../.gitbook/assets/image (1) (1) (3).png" alt=""><figcaption></figcaption></figure>



### Format Converison

<figure><img src="../.gitbook/assets/image (18) (4).png" alt=""><figcaption></figcaption></figure>



## Startup Code

Our LED has successfully blinked. However, if we try running other more complex programs, they might fail. This is because we have made a lot of assumptions about the state of the SoC when we enter the main() function.&#x20;

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

&#x20;



### Interrupt Vector



### Stack Initialization







\_\_stack\_size

\_\_boot\_hart\_idx

\_\_global\_pointer$

\_sp: Address of the end of stack for hart 0, used to initialize the beginning of the stack since the stack grows lower in memory. On a multi-hart system, the start address of the stack for each hart is calculated using (\_sp + \_\_stack\_size \* mhartid)

metal\_segment\_bss\_target\_start & metal\_segment\_bss\_target\_end ◦ Used to zero out global data mapped to .bss section

metal\_segment\_data\_source\_start, metal\_segment\_data\_target\_start, metal\_segment\_data\_target\_end ◦ Used to copy data from image to its destination in RAM.

metal\_segment\_itim\_source\_start, metal\_segment\_itim\_target\_start, metal\_segment\_itim\_target\_end ◦ Code or data can be placed in itim sections using the \_\_attribute\_\_section(".itim")







