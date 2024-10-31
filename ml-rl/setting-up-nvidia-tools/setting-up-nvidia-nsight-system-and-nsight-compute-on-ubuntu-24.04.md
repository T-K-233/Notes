# Setting up NVIDIA Nsight System and Nsight Compute on Ubuntu 24.04



## Install dependencies

Nsight Compute requires libcursor to libxcb-cursor to run

[ref](https://doc.qt.io/qt-6/linux-requirements.html)

```bash
sudo apt install libxcb-cursor-dev
```





## Install Nsight Compute

Download Nsight Compute from the [website](https://developer.nvidia.com/tools-overview/nsight-compute/get-started).



Mark the downloaded file as executable, and then do

```bash
sudo ./nsight-compute-linux-2024.3.2.3-34861637.run 
```



The default installation directory is `/usr/local/NVIDIA-Nsight-Compute-2024.3`

<figure><img src="../../.gitbook/assets/image.png" alt=""><figcaption></figcaption></figure>





Launch Nsight Compute

```bash
/usr/local/NVIDIA-Nsight-Compute/ncu-ui
```







