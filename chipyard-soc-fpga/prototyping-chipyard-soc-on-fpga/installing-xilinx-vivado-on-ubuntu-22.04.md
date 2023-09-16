# Installing Xilinx Vivado on Ubuntu 22.04

Reference Official Tutorial:

{% embed url="https://digilent.com/reference/programmable-logic/guides/installing-vivado-and-sdk" %}



### Updates:

2023-06-15  update to Vivado 2023.



## 1. Download and Install Vivado

Download Vivado installer from [download page](https://www.xilinx.com/member/forms/download/xef.html?filename=Xilinx\_Unified\_2023.1\_0507\_1903\_Lin64.bin).

After download, run the following command to grant executable right to the binary file.

```bash
chmod +x ~/Downloads/Xilinx_Unified_2023.1_0507_1903_Lin64.bin
```

Then, run the file using&#x20;

```bash
sudo ~/Downloads/Xilinx_Unified_2023.1_0507_1903_Lin64.bin
```



In the pop-up window, log in with AMD account.



Select "Vivado".

<figure><img src="../../.gitbook/assets/image (162).png" alt=""><figcaption></figcaption></figure>

Select "Vivado ML Standard"

<figure><img src="../../.gitbook/assets/image (163).png" alt=""><figcaption></figcaption></figure>



Select installation contents. For Arty 100T, only "Production Devices > 7 Series > Artix-7" needs to be selected.

<figure><img src="../../.gitbook/assets/image (164).png" alt=""><figcaption></figcaption></figure>



Accept the License Agreements.

<figure><img src="../../.gitbook/assets/image (165).png" alt=""><figcaption></figcaption></figure>

Select the installation path. Here, we will be using `/home/tk/Documents/Xilinx`.

<figure><img src="../../.gitbook/assets/image (166).png" alt=""><figcaption></figcaption></figure>



<figure><img src="../../.gitbook/assets/image (5) (2) (1).png" alt=""><figcaption></figcaption></figure>



## 2. Install Additional Dependencies

After installation, we need another dependency:

```bash
sudo apt install libtinfo5
```



## 3. Launching Vivado

By default, vivado can only be executed by invoking the program with the installation path:

```bash
~/Documents/Xilinx/Vivado/2023.1/bin/vivado
```



To be able to directly invoke vivado, we need to add the installational path to PATH environment variable.

```bash
export PATH="/home/tk/Documents/Xilinx/Vivado/2023.1/bin:$PATH"
```

To make the change effective, run

```bash
source ~/.bashrc
```



## 4. Install cable drivers

To connect to boards, we need to install additional drivers by running

```bash
cd ~/Documents/Xilinx/Vivado/2023.1/data/xicom/cable_drivers/lin64/install_script/install_drivers
sudo ./install_drivers
```

<figure><img src="../../.gitbook/assets/image (53).png" alt=""><figcaption></figcaption></figure>



## 5. Install board support files

Download the most recent [Master Branch ZIP Archive](https://github.com/Digilent/vivado-boards/archive/master.zip) of Digilent's [vivado-boards](https://github.com/Digilent/vivado-boards) Github repository and extract it.

Open the folder extracted from the archive and navigate to its `new/board_files` folder. You will be copying all of this folder's subfolders.

Open Vivado installation path `~/Documents/Xilinx/Vivado/`. Under this folder, navigate to its `<version>/data/boards/board_files` directory (e.g. `~/Documents/Xilinx/Vivado/2023.1/data/boards/board_files`). If this folder doesn't exist, create it.

**Copy** all of the folders found in vivado-boards' `new/board_files` folder, then **paste** them into this folder.

