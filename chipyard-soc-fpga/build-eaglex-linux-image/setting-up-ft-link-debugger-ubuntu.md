# Setting up FT-LINK Debugger - Ubuntu

Plug in the FT-LINK and run the following command

```bash
lsusb
```

There should be two entries with device ID of `0403:6010`.&#x20;

```bash
Bus 003 Device 003: ID 0403:6010 Future Technology Devices International, Ltd FT2232C/D/H Dual UART/FIFO IC
Bus 003 Device 002: ID 0403:6010 Future Technology Devices International, Ltd FT2232C/D/H Dual UART/FIFO IC
```



```bash
sudo nano /etc/udev/rules.d/48-ft-link-usb.rules
```

```bash
###########################################################################
#                                                                         #
#  48-ft-link-usb.rules -- UDEV rules for FT-LINK JTAG Debugger           #
#                                                                         #
###########################################################################
#  Author : -T.K.-                                                        #
#  Date   : 2023-10-02                                                    #
###########################################################################

# Create "/dev" entries for device with read and write permission         
# granted to all users.
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", GROUP="users", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", GROUP="users", MODE:="0666"

```

```bash
sudo udevadm control --reload
```







