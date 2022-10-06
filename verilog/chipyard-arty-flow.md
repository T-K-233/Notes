# Chipyard Arty Flow

```
cd chipyard

./scripts/init-fpga.sh
```



add vivado to env.sh script

```
PATH=/home/tk/Documents/Xilinx/Vivado/2022.1/bin:$PATH
```



```
source ./env.sh
```



```
cd fpga
```



```
make SUB_PROJECT=arty bitstream
```

