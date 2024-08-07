# Setting up TensorRT Environment on Ubuntu 22.04 / 20.04

## Summary

Ubuntu 22.04 or Ubuntu 20.04

NVIDIA driver version: 535 (535.129.03)

CUDA version: 12.1.1

cuDNN version: 8.9.7 for CUDA 12.X

TensorRT version: 8.6 GA





## Installing CUDA

Follow the official [instruction](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html), download CUDA from [here](https://developer.nvidia.com/cuda-toolkit-archive).&#x20;



{% tabs %}
{% tab title="Ubuntu 22.04" %}
Select CUDA 12.4.0 (March 2024), and then execute the commands prompted by the instruction on the website.

Here's an example set of commands to run:

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.4.0/local_installers/cuda-repo-ubuntu2204-12-4-local_12.4.0-550.54.14-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2204-12-4-local_12.4.0-550.54.14-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2204-12-4-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-4
```
{% endtab %}

{% tab title="Ubuntu 20.04" %}
Select CUDA 12.2.2 (August 2023), and then execute the commands prompted by the instruction on the website.

Here's an example set of commands to run:

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.2.2/local_installers/cuda-repo-ubuntu2004-12-2-local_12.2.2-535.104.05-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-12-2-local_12.2.2-535.104.05-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2004-12-2-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt update
sudo apt install -y cuda
```
{% endtab %}
{% endtabs %}



After installation, add CUDA to \~/.bashrc:

```bash
# ~/.bashrc
...

# CUDA
export PATH="/usr/local/cuda-12.4/bin/:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda-12.4/lib64/:$LD_LIBRARY_PATH"

...
```



To test cuda installation, run the following command. The system should be able to find `nvcc`.

```bash
$ nvcc -V
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2024 NVIDIA Corporation
Built on Tue_Feb_27_16:19:38_PST_2024
Cuda compilation tools, release 12.4, V12.4.99
Build cuda_12.4.r12.4/compiler.33961263_0
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

Follow the official [instruction](https://docs.nvidia.com/deeplearning/cudnn/latest/installation/linux.html#package-manager-local-installation), download cuDNN [here](https://developer.nvidia.com/rdp/cudnn-download).

<figure><img src="../../.gitbook/assets/Screenshot from 2023-12-22 01-42-04.png" alt=""><figcaption></figcaption></figure>

{% tabs %}
{% tab title="Ubuntu 22.04" %}
Select "cuDNN v9.3.0, for CUDA 12.x" with the .deb file option, and then execute the commands prompted by the instruction on the website.



Here's an example set of command to run:

```bash
wget https://developer.download.nvidia.com/compute/cudnn/9.3.0/local_installers/cudnn-local-repo-ubuntu2204-9.3.0_1.0-1_amd64.deb
sudo dpkg -i cudnn-local-repo-ubuntu2204-9.3.0_1.0-1_amd64.deb
sudo cp /var/cudnn-local-repo-ubuntu2204-9.3.0/cudnn-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cudnn
```
{% endtab %}

{% tab title="Ubuntu 20.04" %}
Select "cuDNN v8.9.7 (December 5th, 2023), for CUDA 12.x" with the .deb file option, and then execute the commands prompted by the instruction on the website.



Here's an example set of command to run:

```bash
wget https://developer.download.nvidia.com/compute/cudnn/9.0.0/local_installers/cudnn-local-repo-ubuntu2004-9.0.0_1.0-1_amd64.deb
sudo dpkg -i cudnn-local-repo-ubuntu2004-9.0.0_1.0-1_amd64.deb
sudo cp /var/cudnn-local-repo-ubuntu2004-9.0.0/cudnn-*-keyring.gpg /usr/share/keyrings/
sudo apt update
sudo apt install -y cudnn
```
{% endtab %}
{% endtabs %}







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

Download **both** the "TensorRT 10.3 GA for Linux x86\_64 and CUDA 12.0 to 12.5 TAR Package" and the DEB package

{% hint style="warning" %}
**Warning:** Be careful to download the version matching with system CUDA version.
{% endhint %}



Install the DEB package with Software Install.

Alternatively, do the following commands

```bash
sudo dpkg -i ./nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5_1.0-1_amd64.deb
sudo cp /var/nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5/nv-tensorrt-local-620E7D29-keyring.gpg /usr/share/keyrings/
```



```bash
sudo apt update
sudo apt install nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5
```



We also need to link the libraries. Unpack the tar package:

```bash
tar xzvf ./TensorRT-10.3.0.26.Linux.x86_64-gnu.cuda-12.5.tar.gz
```

Then. move the unpacked directory to the installation path (\~/Documents/), and add to bashrc

```bash
...

# TensorRT
export TRT_LIBPATH="/home/tk/Documents/TensorRT-10.3.0.26/targets/x86_64-linux-gnu/lib/"
export LD_LIBRARY_PATH="/home/tk/Documents/TensorRT-10.3.0.26/lib/:$TRT_LIBPATH:$LD_LIBRARY_PATH"

...
```



Install to Python using the following command

```bash
cd ~/Documents/TensorRT-10.3.0.26/python/
pip install ./tensorrt-10.3.0-cp38-none-linux_x86_64.whl
```





For TensorRT < 10.3, it might also need this dependency

```bash
cd ~/Documents/TensorRT-8.6.1.6/graphsurgeon/
pip install ./graphsurgeon-0.4.6-py2.py3-none-any.whl
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





