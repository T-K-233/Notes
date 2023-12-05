# Running Chipyard Simulation - Ubuntu

```bash
source ./env.sh
```

```bash
cd ./sims/verilator/
```

```bash
make run-binary CONFIG=ExampleChipConfig BINARY=../../tests/hello.riscv
```

```bash
make run-binary CONFIG=ExampleChipConfig BINARY=../../tests/hello.riscv TIMEOUT_CYCLES=100000000
```

For loading large programs, use `LOADMEM=1`

```bash
make run-binary CONFIG=ExampleChipConfig LOADMEM=1 BINARY=../../software/baremetal-ide/workspace/build/firmware.elf TIMEOUT_CYCLES=1000000
```
