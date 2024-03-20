# Setting up TensorRT Environment on Ubuntu 2X.04

## Summary

Ubuntu 22.04 or Ubuntu 20.04

NVIDIA driver version: 535 (535.129.03)

CUDA version: 12.1.1

cuDNN version: 8.9.7 for CUDA 12.X

TensorRT version: 8.6 GA





## Installing NVIDIA Driver

{% tabs %}
{% tab title="Ubuntu GUI Install" %}
Install NVIDIA Driver from the "Software & Updates" window. Select `nvidia-driver-535-server`

<figure><img src="../../.gitbook/assets/Screenshot from 2023-12-22 01-07-21.png" alt=""><figcaption></figcaption></figure>

nvidia-smi should work after rebooting the system.
{% endtab %}

{% tab title="Ubuntu CLI Install" %}
Suggested by this [tutorial](https://ubuntu.com/server/docs/nvidia-drivers-installation), we can also install the driver from command line:

First, list all available drivers with this command

```bash
sudo ubuntu-drivers list
```



Then, install the driver with following command

```bash
sudo apt install nvidia-utils-535-server
```



nvidia-smi should work after rebooting the system.



(Optional) If the system is not configured correctly, try run the following command and manually select the packages.

```bash
sudo ubuntu-drivers install --gpgpu nvidia:535-server
```
{% endtab %}
{% endtabs %}





Verify NVIDIA Driver is correctly installed by running `nvidia-smi`









## Installing CUDA

Follow the official [instruction](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html), download CUDA from [here](https://developer.nvidia.com/cuda-toolkit-archive).&#x20;

Select CUDA 12.2.2 (August 2023), and then execute the commands prompted by the instruction on the website.

Here's an example set of command to run:

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.2.2/local_installers/cuda-repo-ubuntu2004-12-2-local_12.2.2-535.104.05-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-12-2-local_12.2.2-535.104.05-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2004-12-2-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt update
sudo apt install -y cuda
```



After installation, add CUDA to \~/.bashrc:

```bash
# ~/.bashrc
...

# CUDA
export PATH="/usr/local/cuda-12.2/bin/:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda-12.2/lib64/:$LD_LIBRARY_PATH"

...
```



To test cuda installation, run the following command. The system should be able to find `nvcc`.

```bash
$ nvcc -V
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2023 NVIDIA Corporation
Built on Mon_Apr__3_17:16:06_PDT_2023
Cuda compilation tools, release 12.1, V12.1.105
Build cuda_12.1.r12.1/compiler.32688072_0
```



{% hint style="info" %}
**Note:**&#x20;

nvidia-smi might fail to run with the following error:

```bash
$ nvidia-smi
Failed to initialize NVML: Driver/library version mismatch
```

This happens when the CUDA we installed is a different version than the one comes with the driver. If this happens, reboot the system to let nvidia-smi reload the correct CUDA.

After reboot:

```bash
tk@DESKTOP-Scratch:~$ nvidia-smi
Fri Dec 22 13:48:54 2023       
+---------------------------------------------------------------------------------------+
| NVIDIA-SMI 530.30.02              Driver Version: 530.30.02    CUDA Version: 12.1     |
|-----------------------------------------+----------------------+----------------------+
| GPU  Name                  Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf            Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                                         |                      |               MIG M. |
|=========================================+======================+======================|
|   0  NVIDIA GeForce GTX 1070         On | 00000000:27:00.0 Off |                  N/A |
| 34%   37C    P8                8W / 190W|    178MiB /  8192MiB |      1%      Default |
|                                         |                      |                  N/A |
+-----------------------------------------+----------------------+----------------------+
                                                                                         
+---------------------------------------------------------------------------------------+
| Processes:                                                                            |
|  GPU   GI   CI        PID   Type   Process name                            GPU Memory |
|        ID   ID                                                             Usage      |
|=======================================================================================|
|    0   N/A  N/A       987      G   /usr/lib/xorg/Xorg                           44MiB |
|    0   N/A  N/A      1259      G   /usr/bin/gnome-shell                        131MiB |
+---------------------------------------------------------------------------------------+

```


{% endhint %}



## Installing cuDNN

Follow the official [instruction](https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html#installlinux-deb), download cuDNN [here](https://developer.nvidia.com/rdp/cudnn-download).

<figure><img src="../../.gitbook/assets/Screenshot from 2023-12-22 01-42-04.png" alt=""><figcaption></figcaption></figure>

Select "cuDNN v8.9.7 (December 5th, 2023), for CUDA 12.x" with the .deb file option, and then execute the commands prompted by the instruction on the website.

Here's an example set of command to run:

```bash
wget https://developer.download.nvidia.com/compute/cudnn/9.0.0/local_installers/cudnn-local-repo-ubuntu2004-9.0.0_1.0-1_amd64.deb
sudo dpkg -i cudnn-local-repo-ubuntu2004-9.0.0_1.0-1_amd64.deb
sudo cp /var/cudnn-local-repo-ubuntu2004-9.0.0/cudnn-*-keyring.gpg /usr/share/keyrings/
sudo apt update
sudo apt install -y cudnn
```



```bash
sudo apt install -y cudnn-cuda-12
```



(might not need do the following)

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



## Installing TensorRT

There are two parts of TensorRT installation

### TensorRT GA

Goto [https://developer.nvidia.com/tensorrt](https://developer.nvidia.com/tensorrt)

Download **both** the "TensorRT 8.6 GA for Linux x86\_64 and CUDA 12.0 and 12.1 TAR Package" and the DEB package

{% hint style="warning" %}
**Warning:** Be careful to download the version matching with system CUDA version.
{% endhint %}



Install the DEB package with Software Install.

Alternatively, do the following commands

```bash
sudo dpkg -i ./nv-tensorrt-local-repo-ubuntu2204-8.6.1-cuda-12.0_1.0-1_amd64.deb
sudo cp /var/nv-tensorrt-local-repo-ubuntu2204-8.6.1-cuda-12.0/nv-tensorrt-local-42B2FC56-keyring.gpg /usr/share/keyrings/
```



```bash
sudo apt update
sudo apt install nv-tensorrt-local-repo-ubuntu2204-8.6.1-cuda-12.0
```







We also need to link the libraries. Unpack the tar package:

```bash
tar xzvf ./TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-12.0.tar.gz 
```

Then. move the unpacked directory to the installation path (\~/Documents/), and add to bashrc

```bash
...

# TensorRT
export TRT_LIBPATH="/home/nai/Documents/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/lib/"
export LD_LIBRARY_PATH="/home/tk/Documents/TensorRT-8.6.1.6/lib/:$TRT_LIBPATH:$LD_LIBRARY_PATH"

...
```



Install to Python using the following command

```bash
cd ~/Documents/TensorRT-8.6.1.6/python/
pip install ./tensorrt-8.6.1-cp310-none-linux_x86_64.whl
```

Might also need to install this dependency

```bash
cd ~/Documents/TensorRT-8.6.1.6/graphsurgeon]/
pip install ./graphsurgeon-0.4.5-py2.py3-none-any.whl
```





### TensorRT OSS

Seems that normally we don't need this thing.



For python:

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



## Other Dependencies

```bash
pip3 install cuda-python
```

```bash
sudo apt install libssl-dev

sudo apt install cmake g++
```



```bash
$ cmake --version
cmake version 3.16.3

CMake suite maintained and supported by Kitware (kitware.com/cmake).
```



```bash
pip install onnx
pip install onnxruntime
pip install onnxruntime-gpu
pip install onnx-simplifier
```





## Final Step

Follow this repo to use TRT&#x20;

{% embed url="https://github.com/T-K-233/trt-samples-for-hackathon-cn/tree/main/cookbook" %}



## FAQ

```bash
Hit:7 http://us.archive.ubuntu.com/ubuntu focal-updates InRelease
Err:3 file:/var/nv-tensorrt-local-repo-ubuntu2004-8.6.1-cuda-12.0  InRelease
  The following signatures couldn't be verified because the public key is not available: NO_PUBKEY 6694DE8A9A1EDFBA
Hit:8 http://us.archive.ubuntu.com/ubuntu focal-backports InRelease
Reading package lists... Done
W: GPG error: file:/var/nv-tensorrt-local-repo-ubuntu2004-8.6.1-cuda-12.0  InRelease: The following signatures couldn't be verified because the public key is not available: NO_PUBKEY 6694DE8A9A1EDFBA
E: The repository 'file:/var/nv-tensorrt-local-repo-ubuntu2004-8.6.1-cuda-12.0  InRelease' is not signed.
N: Updating from such a repository can't be done securely, and is therefore disabled by default.
N: See apt-secure(8) manpage for repository creation and user configuration details.
```



solution:

the key is not installed. run the command in "Installing TensorRT" again

```bash
sudo cp /var/nv-tensorrt-local-repo-ubuntu2204-8.6.1-cuda-12.0/nv-tensorrt-local-42B2FC56-keyring.gpg /usr/share/keyrings/
```





