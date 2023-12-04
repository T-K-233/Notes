# Setting up FT-LINK Debugger - Ubuntu

## Installing dependencies

```bash
sudo apt install libconfuse-dev
sudo apt install libftdi-dev
```



## Setting udev rules

"udev rule" are used to manage the device nodes in the `/dev` directory. These rules provide a way to configure how Linux's device manager (`udev`) handles devices like USB drives, hard disks, and peripherals when they are connected to the system.

To use FT-LINK, user need to set up udev rules to grant proper permission to the device.



Plug in the FT-LINK and run the following command

```bash
lsusb
```



The FT-Link comes with vendor id of `0403` and device id of `6010`. There should be two entries with device ID of `0403:6010`.&#x20;

```bash
Bus 003 Device 003: ID 0403:6010 Future Technology Devices International, Ltd FT2232C/D/H Dual UART/FIFO IC
Bus 003 Device 002: ID 0403:6010 Future Technology Devices International, Ltd FT2232C/D/H Dual UART/FIFO IC
```



If there are too many devices connected to the system, the above procedure can also be done with&#x20;

```bash
lsusb -d 0403:6010
```



With the `-v` option, we can see more information

```bash
lsusb -d 0403:6010 -v

Bus 003 Device 014: ID 0403:6010 Future Technology Devices International, Ltd FT2232C/D/H Dual UART/FIFO IC
Couldn't open device, some information will be missing
Device Descriptor:
  bLength                18
  bDescriptorType         1
  bcdUSB               2.00
  bDeviceClass            0 
  bDeviceSubClass         0 
  bDeviceProtocol         0 
  bMaxPacketSize0         8
  idVendor           0x0403 Future Technology Devices International, Ltd
  idProduct          0x6010 FT2232C/D/H Dual UART/FIFO IC
  bcdDevice            5.00
  iManufacturer           1 FTDI
  iProduct                2 Dual RS232
  iSerial                 0 
  bNumConfigurations      1
  Configuration Descriptor:
    bLength                 9
    bDescriptorType         2
    wTotalLength       0x0037
    bNumInterfaces          2
    bConfigurationValue     1
    iConfiguration          0 
    bmAttributes         0x80
      (Bus Powered)
    MaxPower               90mA
    Interface Descriptor:
      bLength                 9
      bDescriptorType         4
      bInterfaceNumber        0
      bAlternateSetting       0
      bNumEndpoints           2
      bInterfaceClass       255 Vendor Specific Class
      bInterfaceSubClass    255 Vendor Specific Subclass
      bInterfaceProtocol    255 Vendor Specific Protocol
      iInterface              2 
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x81  EP 1 IN
        bmAttributes            2
          Transfer Type            Bulk
          Synch Type               None
          Usage Type               Data
        wMaxPacketSize     0x0040  1x 64 bytes
        bInterval               0
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x02  EP 2 OUT
        bmAttributes            2
          Transfer Type            Bulk
          Synch Type               None
          Usage Type               Data
        wMaxPacketSize     0x0040  1x 64 bytes
        bInterval               0
    Interface Descriptor:
      bLength                 9
      bDescriptorType         4
      bInterfaceNumber        1
      bAlternateSetting       0
      bNumEndpoints           2
      bInterfaceClass       255 Vendor Specific Class
      bInterfaceSubClass    255 Vendor Specific Subclass
      bInterfaceProtocol    255 Vendor Specific Protocol
      iInterface              2 
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x83  EP 3 IN
        bmAttributes            2
          Transfer Type            Bulk
          Synch Type               None
          Usage Type               Data
        wMaxPacketSize     0x0040  1x 64 bytes
        bInterval               0
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x04  EP 4 OUT
        bmAttributes            2
          Transfer Type            Bulk
          Synch Type               None
          Usage Type               Data
        wMaxPacketSize     0x0040  1x 64 bytes
        bInterval               0
```



To grant permission to this device, we need to add a new rule to the system

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

After adding the rule file, use the following command to reload the udev rules

```bash
sudo udevadm control --reload
```



Then, unplug FT-LINK and plug in it back again after a few seconds



Now, when running the command

```bash
lsusb -d 0403:6010 -v
```

we should able to see more device status:

```bash
lsusb -d 0403:6010 -v

Bus 003 Device 015: ID 0403:6010 Future Technology Devices International, Ltd FT2232C/D/H Dual UART/FIFO IC
Device Descriptor:
  bLength                18
  bDescriptorType         1
  bcdUSB               2.00
  bDeviceClass            0 
  bDeviceSubClass         0 
  
...
        wMaxPacketSize     0x0040  1x 64 bytes
        bInterval               0
Device Status:     0x0000
  (Bus Powered)
```





