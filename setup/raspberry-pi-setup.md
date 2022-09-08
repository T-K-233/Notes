# Raspberry Pi Setup

## Set up IP address

use the `ifconfig` command to retrieve the MAC address of Pi:

```
ifconfig
```



## Set up Nomachine

Use the following command to get CPU version

```
cat /proc/cpuinfo
```



And download the corresponding file from Nomachine

![](<../.gitbook/assets/image (123).png>)



After download, **move the file in a desired installation directory before unzip and running the install**.



Extract the tar gz file:

```
sudo tar zxvf nomachine_7.9.2_1_armv7hl.tar.gz
```



Install:

```
sudo ./NX/nxserver --install
```



## Set up OpenCV-Python

```
pip3 install opencv-python

sudo apt install libatlas-base-dev

pip3 install --upgrade numpy

```
