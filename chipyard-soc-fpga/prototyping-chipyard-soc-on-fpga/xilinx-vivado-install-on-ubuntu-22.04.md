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



<figure><img src="../../.gitbook/assets/image (101).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (3) (1) (1).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (4) (2) (1) (1).png" alt=""><figcaption></figcaption></figure>

Follow the commands in the installer. Installation directory is `~/Documents/Xilinx`

<figure><img src="../../.gitbook/assets/image (5) (2) (1).png" alt=""><figcaption></figcaption></figure>



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

<figure><img src="../../.gitbook/assets/image (53).png" alt=""><figcaption></figcaption></figure>



### Install board support files

Download the most recent [Master Branch ZIP Archive](https://github.com/Digilent/vivado-boards/archive/master.zip) of Digilent's [vivado-boards](https://github.com/Digilent/vivado-boards) Github repository and extract it.

Open the folder extracted from the archive and navigate to its `new/board_files` folder. You will be copying all of this folder's subfolders.

Open Vivado installation path `~/Documents/Xilinx/Vivado/`. Under this folder, navigate to its `<version>/data/boards/board_files` directory (e.g. `~/Documents/Xilinx/Vivado/2022.1/data/boards/board_files`). If this folder doesn't exist, create it.

**Copy** all of the folders found in vivado-boards' `new/board_files` folder, then **paste** them into this folder.

