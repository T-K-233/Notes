# Configure USB Access Permissions on Ubuntu

{% embed url="https://www.xmodulo.com/change-usb-device-permission-linux.html" %}



```bash
lsusb
```



```bash
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 003: ID 8087:0a2a Intel Corp. 
Bus 001 Device 002: ID 0483:5740 STMicroelectronics Virtual COM Port
Bus 001 Device 007: ID 045e:0b12 Microsoft Corp. Controller
Bus 001 Device 006: ID 045e:02ea Microsoft Corp. Xbox One S Controller
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```



```bash
sudo nano /etc/udev/rules.d/50-xboxcontroller.rules
```



```bash
SUBSYSTEMS=="usb", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="02ea", GROUP="users", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="0b12", GROUP="users", MODE="0666"
```



```bash
sudo udevadm control --reload
```



