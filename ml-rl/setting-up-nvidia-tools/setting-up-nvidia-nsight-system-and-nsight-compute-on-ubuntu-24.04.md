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
sudo ~/Downloads/nsight-compute-linux-2024.3.2.3-34861637.run 
```



The default installation directory is `/usr/local/NVIDIA-Nsight-Compute-2024.3/`

<figure><img src="../../.gitbook/assets/image (2) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>





Launch Nsight Compute

```bash
/usr/local/NVIDIA-Nsight-Compute/ncu-ui
```





## Install Nsight Systems

Download from [website](https://developer.nvidia.com/nsight-systems/get-started).

Mark as executable, and then run

```bash
sudo ~/Downloads/NsightSystems-linux-public-2024.6.1.90-3490548.run
```





Default installation directory is `/opt/nvidia/nsight-systems/2024.6.1/`



<figure><img src="../../.gitbook/assets/image (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>



Launch Nsight Systems

```bash
sudo /opt/nvidia/nsight-systems/2024.6.1/bin/nsys-ui
```





## Add to PATH

{% code title="~/.bashrc" %}
```bash
...

# NVIDIA
export PATH="/opt/nvidia/nsight-systems/2024.6.1/bin/:$PATH"
export PATH="/usr/local/NVIDIA-Nsight-Compute/:$PATH"

...
```
{% endcode %}





## Start Profiling



<figure><img src="../../.gitbook/assets/image (10).png" alt=""><figcaption></figcaption></figure>



