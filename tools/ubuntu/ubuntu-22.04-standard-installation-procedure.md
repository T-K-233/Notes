# Ubuntu 22.04 Standard Installation Procedure

## Installer Setup

### Page 1

Select "English" (default)

Click "Install Ubuntu" button



### Page 2

**Keyboard Layout**

"English(US)" and "English(US)" (default)



### Page 3

**What apps would you like to install to start with?**

* [ ] Normal Installation - Web browser, utilities, office software, games, and media players
* [x] Minimal Installation - Web browser and basic utilities

**Other options**

* [x] Download updates while installing Ubuntu
* [x] Install third-party software for graphics and Wi-Fi hardware and additional media formats



### Page 4

{% tabs %}
{% tab title="If one system disk (Recommended)" %}
_Note: This is the recommended option. It's much safer to remove data disks before system installation._



If there's only one system disk:

* [ ] (some other options)
* [x] Erase disk and install Ubuntu
* [ ] Something else
{% endtab %}

{% tab title="If multiple system disks" %}
If there are multiple system disks:

* [ ] (some other options)
* [ ] Erase disk and install Ubuntu
* [x] Something else

In the partition menu, clear out all the existing partitions, and create two new partitions:



| **Size**                           | 538 MB                            |
| ---------------------------------- | --------------------------------- |
| **Type for the new partition**     | Primary (default)                 |
| **Location for the new partition** | Beginning of this space (default) |
| **Use as**                         | EFI System Partition              |



_Note: EFI System Partition needs to be greater than 35 MB in size, and Ubuntu default sets it to be 538 MB._



| **Size**                           | remaining disk size (default)     |
| ---------------------------------- | --------------------------------- |
| **Type for the new partition**     | Primary (default)                 |
| **Location for the new partition** | Beginning of this space (default) |
| **Use as**                         | Ext4 journaling file system       |
| **Mount point**                    | /                                 |



**Device for boot loader installation**:

/dev/nvme0n1p1 (or whatever the path to the EFI partition)



[reference article](https://askubuntu.com/questions/343268/how-to-use-manual-partitioning-during-installation)
{% endtab %}
{% endtabs %}



### Page 5

**Where are you**

Los Angeles



### Page 6

**Your name**

TK

**Your computer's name**

Some-Computer-Name

**Pick a username**

tk

**Choose a password**

········

**Comfirm your password**

········

* [x] Log in automatically
* [ ] Require my password to log in



* [ ] Use Active Directory (default)



## After Installation

### System Update

```bash
sudo apt update

sudo apt upgrade

sudo reboot
```



### Install VS Code

Search and install "vscode" with "Ubuntu Software".



### Install Google Chrome

Search and download "64 bit .deb" version of "Chrome for Linux" with FireFox.

Right click the downloaded file and select "Open With Other Application". Select "Software Install".



### Install ssh

{% embed url="https://linuxize.com/post/how-to-enable-ssh-on-ubuntu-20-04/" %}

Install the openssh-server package

```bash
sudo apt update
sudo apt install openssh-server
```



Verify ssh is running

```bash
sudo systemctl status ssh
```



Config firewall

```bash
sudo ufw allow ssh
```





