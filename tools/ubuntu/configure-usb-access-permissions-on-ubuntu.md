# Configure USB Access Permissions (udev rules) on Ubuntu

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



### udev rule conventions

```
Files should be named xx-descriptive-name.rules, the xx should be
chosen first according to the following sequence points:

< 60  most user rules; if you want to prevent an assignment being
overriden by default rules, use the := operator.

these cannot access persistent information such as that from
vol_id

< 70  rules that run helpers such as vol_id to populate the udev db

< 90  rules that run other programs (often using information in the
udev db)

>=90  rules that should run last
```



## References

{% embed url="https://hackaday.com/2009/09/18/how-to-write-udev-rules/" %}

