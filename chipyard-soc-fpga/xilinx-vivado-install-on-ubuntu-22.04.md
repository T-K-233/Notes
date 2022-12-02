# Xilinx Vivado Install on Ubuntu 22.04

Reference Official Tutorial:

{% embed url="https://digilent.com/reference/programmable-logic/guides/installing-vivado-and-sdk" %}



Download Vivado installer from [download page](https://www.xilinx.com/member/forms/download/xef.html?filename=Xilinx\_Unified\_2022.1\_0420\_0327\_Lin64.bin).

After download, run the following command to grant executable right to the binary file.

```
chmod +x Xilinx_Unified_2022.1_0420_0327_Lin64.bin
```

Then, run the file using&#x20;

```
sudo ./Xilinx_Unified_2022.1_0420_0327_Lin64.bin
```



<figure><img src="../.gitbook/assets/image (101).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../.gitbook/assets/image (3) (1) (1).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../.gitbook/assets/image (4) (2) (1) (1).png" alt=""><figcaption></figcaption></figure>

Follow the commands in the installer. Installation directory is `~/Documents/Xilinx`

<figure><img src="../.gitbook/assets/image (5) (2).png" alt=""><figcaption></figcaption></figure>



After installation, we need another dependency:

```
sudo apt install libtinfo5
```



Start vivado by running

```
~/Documents/Xilinx/Vivado/2022.1/bin/vivado
```

We can also add it to PATH by

```
export PATH="/home/tk/Documents/Xilinx/Vivado/2022.1/bin:$PATH"
```

and then

```
source ~/.bashrc
```



### Install cable drivers

To connect to boards, we need to install additional drivers by running

```
cd ~/Documents/Xilinx/Vivado/2022.1/data/xicom/cable_drivers/lin64/install_script/install_drivers
sudo ./install_drivers
```

<figure><img src="../.gitbook/assets/image (53).png" alt=""><figcaption></figcaption></figure>



### Install board support files


