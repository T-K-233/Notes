# Setting up TensorRT on PyTorch





## Install Protobuf

```bash
sudo apt install libprotobuf-dev protobuf-compiler
```







```bash
cmake .. \
-DTENSORRT_ROOT=/home/tk/Documents/mambaforge/envs/diff/lib/python3.10/site-packages/tensorrt \
-DProtobuf_INCLUDE_DIR=/usr/include/google/protobuf
```
