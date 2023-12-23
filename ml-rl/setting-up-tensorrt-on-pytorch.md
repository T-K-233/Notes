# Setting up TensorRT on PyTorch

## Summary

NVIDIA driver version: 535 (530.30.02)

CUDA version: 12.1.1

cuDNN version: 8.9.7 for CUDA 12.X

~~protobuf version: 3.6.1~~

TensorRT version: 8







roughly follow this tutorial

{% embed url="https://github.com/sithu31296/PyTorch-ONNX-TRT" %}







## Install NVIDIA Driver

<figure><img src="../.gitbook/assets/Screenshot from 2023-12-22 01-07-21.png" alt=""><figcaption></figcaption></figure>

or, from this [reference](https://ubuntu.com/server/docs/nvidia-drivers-installation), install from command line:

first, list all available drivers with this command

```bash
sudo ubuntu-drivers list
```



then install the driver with following command

```bash
sudo apt install nvidia-utils-535-server
```



run the following command to apply the changes

```bash
sudo ubuntu-drivers install --gpgpu nvidia:535-server
```



nvidia-smi should work after restart.







## Install CUDA

{% embed url="https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html" %}

Download CUDA from the [CUDA website](https://developer.nvidia.com/cuda-toolkit-archive). Select CUDA 12.1.1



```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.1.1/local_installers/cuda-repo-ubuntu2004-12-1-local_12.1.1-530.30.02-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-12-1-local_12.1.1-530.30.02-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2004-12-1-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda
```



After installation, add CUDA to \~/.bashrc:

```bash
...

# CUDA
export PATH="/usr/local/cuda-12.1/bin/:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda-12.1/lib64/:$LD_LIBRARY_PATH"

...
```



To test cuda installation, run the following command.

```bash
$ nvcc -V
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2023 NVIDIA Corporation
Built on Mon_Apr__3_17:16:06_PDT_2023
Cuda compilation tools, release 12.1, V12.1.105
Build cuda_12.1.r12.1/compiler.32688072_0
```



> Note: nvidia-smi might fail to run with the following error:
>
> ```bash
> $ nvidia-smi
> Failed to initialize NVML: Driver/library version mismatch
> ```
>
> This happens when the CUDA we installed is a different version than the one comes with the driver. If this happens, reboot the system to let nvidia-smi reload the correct CUDA.
>
>
>
> After reboot:
>
> ```bash
> tk@DESKTOP-Scratch:~$ nvidia-smi
> Fri Dec 22 13:48:54 2023       
> +---------------------------------------------------------------------------------------+
> | NVIDIA-SMI 530.30.02              Driver Version: 530.30.02    CUDA Version: 12.1     |
> |-----------------------------------------+----------------------+----------------------+
> | GPU  Name                  Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
> | Fan  Temp  Perf            Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
> |                                         |                      |               MIG M. |
> |=========================================+======================+======================|
> |   0  NVIDIA GeForce GTX 1070         On | 00000000:27:00.0 Off |                  N/A |
> | 34%   37C    P8                8W / 190W|    178MiB /  8192MiB |      1%      Default |
> |                                         |                      |                  N/A |
> +-----------------------------------------+----------------------+----------------------+
>                                                                                          
> +---------------------------------------------------------------------------------------+
> | Processes:                                                                            |
> |  GPU   GI   CI        PID   Type   Process name                            GPU Memory |
> |        ID   ID                                                             Usage      |
> |=======================================================================================|
> |    0   N/A  N/A       987      G   /usr/lib/xorg/Xorg                           44MiB |
> |    0   N/A  N/A      1259      G   /usr/bin/gnome-shell                        131MiB |
> +---------------------------------------------------------------------------------------+
>
> ```





## Install cuDNN

{% embed url="https://developer.nvidia.com/rdp/cudnn-download" %}

<figure><img src="../.gitbook/assets/Screenshot from 2023-12-22 01-42-04.png" alt=""><figcaption></figcaption></figure>



{% embed url="https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html#installlinux-deb" %}



Follow this:

[https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html#installlinux-deb](https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html#installlinux-deb)

```bash
sudo dpkg -i ./cudnn-local-repo-ubuntu2004-8.9.7.29_1.0-1_amd64.deb 
```

```bash
sudo cp /var/cudnn-local-repo-ubuntu2004-8.9.7.29/cudnn-local-30472A84-keyring.gpg /usr/share/keyrings/
```



```bash
sudo apt update
```

Install the runtime library.

```bash
sudo apt-get install libcudnn8=8.9.7.29-1+cuda12.2
```

Install the developer library.

```bash
sudo apt-get install libcudnn8-dev=8.9.7.29-1+cuda12.2
```

Install the code samples.

```bash
sudo apt-get install libcudnn8-samples=8.9.7.29-1+cuda12.2
```



run the test program to see if success



[https://forums.developer.nvidia.com/t/verify-cudnn-install-failed/167220/4](https://forums.developer.nvidia.com/t/verify-cudnn-install-failed/167220/4)



### this does not work

download the TAR file

```bash
tar -xvf ./cudnn-linux-x86_64-8.9.7.29_cuda12-archive.tar.xz 
```



<pre class="language-bash"><code class="lang-bash">sudo cp cudnn-*-archive/include/cudnn*.h /usr/local/cuda/include
<strong>sudo cp -P cudnn-*-archive/lib/libcudnn* /usr/local/cuda/lib64
</strong>sudo chmod a+r /usr/local/cuda/include/cudnn*.h /usr/local/cuda/lib64/libcudnn*
</code></pre>





## Install Protobuf

```bash
sudo apt install gcc
```



this does not work:

```bash
sudo apt install libprotobuf-dev protobuf-compiler
```



also do not work:



follow this to install protobuf C++ compiler

{% embed url="https://github.com/protocolbuffers/protobuf/blob/v25.1/src/README.md" %}



If Bazel cannot be located, do following, reference [this thread](https://stackoverflow.com/questions/61982500/cannot-install-bazel-on-ubuntu-20-04-invalid-expkeysig), we are using david's method

```bash
sudo apt install apt-transport-https curl gnupg
curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor >bazel-archive-keyring.gpg
sudo mv bazel-archive-keyring.gpg /usr/share/keyrings
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list
```



```bash
bazel build :protoc :protobuf
```

<figure><img src="../.gitbook/assets/Screenshot from 2023-12-22 01-31-00.png" alt=""><figcaption></figcaption></figure>

```bash
sudo cp bazel-bin/protoc /usr/local/bin
```



Verify install correctly:

```bash
$ protoc --version
libprotoc 26.0

$ whereis protoc
protoc: /usr/local/bin/protoc
```



Also download the release here and move the `include` folder to `/usr/local/include/google`

{% embed url="https://github.com/protocolbuffers/protobuf/releases" %}

DOES NOT WORK:

```bash
sudo apt install protobuf-compiler
```



Download the prebuilt file, copy the include/google folder to /usr/include

```bash
sudo cp -r ./include/google/ /usr/include/
```



Also need to download source, and copy the `protobuf-25.1/src/google/protobuf/stubs` folder to there

```bash
cd ~/Downloads/protobuf-25.1/src/google/protobuf/
sudo cp -r ./stubs/ /usr/include/google/protobuf/
```





FINALLY WORKS

```bash
sudo apt install autogen autoconf libtool
```





```bash
git clone https://github.com/protocolbuffers/protobuf.git
cd protobuf
git checkout v3.6.1
```



i think this works up to 3.12.3, havent tried



instruction is [here](https://github.com/protocolbuffers/protobuf/blob/v3.6.1/src/README.md)

```bash
git submodule update --init --recursive
./autogen.sh

./configure --prefix=/usr
make -j8
make check
sudo make install
sudo ldconfig # refresh shared library cache.

```





## TensorRT

There are two parts of TensorRT installation

### TensorRT GA

Goto [https://developer.nvidia.com/tensorrt](https://developer.nvidia.com/tensorrt)

Download "TensorRT 8.6 GA for Linux x86\_64 and CUDA 12.0 and 12.1 TAR Package"

> Be careful to download to match with your CUDA install method. For example, if you installed CUDA with deb file, download TensorRT deb file also. Otherwise, it won't work.



[https://docs.nvidia.com/deeplearning/tensorrt/archives/tensorrt-713/install-guide/index.html#installing-tar](https://docs.nvidia.com/deeplearning/tensorrt/archives/tensorrt-713/install-guide/index.html#installing-tar)



instead do

```bash
tar xzvf ./TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-12.0.tar.gz 
```

move the unpacked directory to the installation path (\~/Documents/), and add to bashrc

```bash
...

# TensorRT
export TRT_LIBPATH="/home/tk/Documents/TensorRT-8.6.1.6/"
export LD_LIBRARY_PATH="/home/tk/Documents/TensorRT-8.6.1.6/lib/:$LD_LIBRARY_PATH"

...
```

```bash
cd ./TensorRT-8.6.1.6/python
pip install ./tensorrt-8.6.1-cp38-none-linux_x86_64.whl
```



```bash
cd ./TensorRT-8.6.1.6/graphsurgeon
pip install ./graphsurgeon-0.4.5-py2.py3-none-any.whl
```





### TensorRT OSS

For python:

pip install tensorrt





```bash
git clone -b main https://github.com/nvidia/TensorRT TensorRT
cd TensorRT
git submodule update --init --recursive
```



```bash
 cd TensorRT
 mkdir -p build && cd build
```



```bash
cmake .. -DTRT_LIB_DIR=$TRT_LIBPATH/lib/ -DTRT_OUT_DIR=`pwd`/out
```



```bash
make -j$(nproc)
```



## Misc

```bash
pip3 install pycuda
```

```bash
sudo apt install libssl-dev

sudo apt install cmake
```



```bash
$ cmake --version
cmake version 3.16.3

CMake suite maintained and supported by Kitware (kitware.com/cmake).
```





## ONNX TensorRT

```bash
sudo apt install swig
```



```bash
git clone https://github.com/onnx/onnx-tensorrt
cd onnx-tensorrt
git submodule update --init --recursive
```



```bash
mkdir -p build && cd build
```

```bash
cmake .. \
-DTENSORRT_ROOT=~/Documents/TensorRT-8.6.1.6/ \
-DProtobuf_INCLUDE_DIR=/usr/include/
```

or

```bash
cmake .. \
-DTENSORRT_ROOT=~/Documents/TensorRT-8.6.1.6/ \
-DProtobuf_INCLUDE_DIR=/usr/include/ \
-DProtobuf_LIBRARY=/usr/lib/
```







```bash
$ cmake .. -DTENSORRT_ROOT=~/Documents/TensorRT-8.6.1.6/ -DProtobuf_INCLUDE_DIR=/usr/include/
-- Found Protobuf: /usr/lib/x86_64-linux-gnu/libprotobuf.so;-lpthread (found version "3.6.1") 
Generated: /home/tk/Downloads/onnx-tensorrt/build/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.proto
Generated: /home/tk/Downloads/onnx-tensorrt/build/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.proto
Generated: /home/tk/Downloads/onnx-tensorrt/build/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.proto
-- 
-- ******** Summary ********
--   CMake version             : 3.16.3
--   CMake command             : /usr/bin/cmake
--   System                    : Linux
--   C++ compiler              : /usr/bin/c++
--   C++ compiler version      : 9.4.0
--   CXX flags                 :  -Wall -Wno-deprecated-declarations -Wno-unused-function -Wnon-virtual-dtor
--   Build type                : Release
--   Compile definitions       : SOURCE_LENGTH=33;ONNX_NAMESPACE=onnx2trt_onnx;__STDC_FORMAT_MACROS
--   CMAKE_PREFIX_PATH         : 
--   CMAKE_INSTALL_PREFIX      : /usr/local
--   CMAKE_MODULE_PATH         : 
-- 
--   ONNX version              : 1.13.1
--   ONNX NAMESPACE            : onnx2trt_onnx
--   ONNX_USE_LITE_PROTO       : OFF
--   USE_PROTOBUF_SHARED_LIBS  : OFF
--   Protobuf_USE_STATIC_LIBS  : ON
--   ONNX_DISABLE_EXCEPTIONS   : OFF
--   ONNX_WERROR               : OFF
--   ONNX_BUILD_TESTS          : OFF
--   ONNX_BUILD_BENCHMARKS     : OFF
-- 
--   Protobuf compiler         : /usr/bin/protoc
--   Protobuf includes         : /usr/include
--   Protobuf libraries        : /usr/lib/x86_64-linux-gnu/libprotobuf.so;-lpthread
--   BUILD_ONNX_PYTHON         : OFF
-- Found CUDA headers at /usr/local/cuda/include
-- Found TensorRT headers at /home/tk/Documents/TensorRT-8.6.1.6/include
-- Find TensorRT libs at /home/tk/Documents/TensorRT-8.6.1.6/lib/libnvinfer.so;/home/tk/Documents/TensorRT-8.6.1.6/lib/libnvinfer_plugin.so
-- Configuring done
-- Generating done
-- Build files have been written to: /home/tk/Downloads/onnx-tensorrt/build

```



and make



```bash
$ make -j8
[  4%] Built target gen_onnx_proto
[  9%] Built target gen_onnx_data_proto
[ 14%] Built target gen_onnx_operators_proto
[ 19%] Building CXX object third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx_onnx2trt_onnx-ml.pb.cc.o
[ 19%] Building CXX object third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx-data_onnx2trt_onnx.pb.cc.o
[ 21%] Building CXX object third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc.o
[ 23%] Linking CXX static library libonnx_proto.a
[ 38%] Built target onnx_proto
Scanning dependencies of target nvonnxparser_static
Scanning dependencies of target nvonnxparser
[ 42%] Building CXX object CMakeFiles/nvonnxparser.dir/ImporterContext.cpp.o
[ 42%] Building CXX object CMakeFiles/nvonnxparser.dir/NvOnnxParser.cpp.o
[ 45%] Building CXX object CMakeFiles/nvonnxparser.dir/onnxErrorRecorder.cpp.o
[ 47%] Building CXX object CMakeFiles/nvonnxparser.dir/ModelImporter.cpp.o
[ 52%] Building CXX object CMakeFiles/nvonnxparser.dir/onnx2trt_utils.cpp.o
[ 52%] Building CXX object CMakeFiles/nvonnxparser.dir/builtin_op_importers.cpp.o
[ 54%] Building CXX object CMakeFiles/nvonnxparser_static.dir/NvOnnxParser.cpp.o
[ 57%] Building CXX object CMakeFiles/nvonnxparser.dir/ShapedWeights.cpp.o
[ 59%] Building CXX object CMakeFiles/nvonnxparser.dir/ShapeTensor.cpp.o
[ 61%] Building CXX object CMakeFiles/nvonnxparser.dir/LoopHelpers.cpp.o
[ 64%] Building CXX object CMakeFiles/nvonnxparser_static.dir/ModelImporter.cpp.o
[ 66%] Building CXX object CMakeFiles/nvonnxparser_static.dir/builtin_op_importers.cpp.o
[ 69%] Building CXX object CMakeFiles/nvonnxparser.dir/RNNHelpers.cpp.o
[ 71%] Building CXX object CMakeFiles/nvonnxparser.dir/OnnxAttrs.cpp.o
[ 73%] Building CXX object CMakeFiles/nvonnxparser.dir/ConditionalHelpers.cpp.o
[ 76%] Building CXX object CMakeFiles/nvonnxparser_static.dir/onnx2trt_utils.cpp.o
[ 78%] Building CXX object CMakeFiles/nvonnxparser_static.dir/onnxErrorRecorder.cpp.o
[ 80%] Building CXX object CMakeFiles/nvonnxparser_static.dir/ImporterContext.cpp.o
[ 83%] Building CXX object CMakeFiles/nvonnxparser_static.dir/ShapedWeights.cpp.o
[ 85%] Building CXX object CMakeFiles/nvonnxparser_static.dir/ShapeTensor.cpp.o
[ 88%] Building CXX object CMakeFiles/nvonnxparser_static.dir/LoopHelpers.cpp.o
[ 90%] Building CXX object CMakeFiles/nvonnxparser_static.dir/RNNHelpers.cpp.o
[ 92%] Building CXX object CMakeFiles/nvonnxparser_static.dir/OnnxAttrs.cpp.o
[ 95%] Building CXX object CMakeFiles/nvonnxparser_static.dir/ConditionalHelpers.cpp.o
[ 97%] Linking CXX shared library libnvonnxparser.so
[ 97%] Built target nvonnxparser
[100%] Linking CXX static library libnvonnxparser_static.a
[100%] Built target nvonnxparser_static

```



```bash
pip install onnx
pip install onnxruntime
pip install onnxruntime-gpu
pip install onnx-simplifier
```





```bash

sudo make install
cd ..

# from TensorRT
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/tk/Documents/TensorRT-8.6.1.6/lib/

python3 setup.py build
python3 setup.py install

```



might need to grant permission

```bash
sudo chmod 777 /usr/local/lib/python3.8/dist-packages/
sudo chmod -R 777 /usr/local/bin/
```

