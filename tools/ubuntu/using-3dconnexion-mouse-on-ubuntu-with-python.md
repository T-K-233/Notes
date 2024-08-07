# Using 3DConnexion Mouse on Ubuntu with Python

```bash
sudo apt install libhidapi-dev
```

```bash
pip install pyspacemouse
```



Create UDev rule for hidraw:

```bash
sudo nano /etc/udev/rules.d/50-hidraw.rules
```

{% code title="50-hidraw.rules" %}
```bash
SUBSYSTEMS=="usb", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c626", GROUP:="users", MODE:="0666"
```
{% endcode %}

And then

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```





An easy way to check if the udev rule is applied successfully is through lsusb command.

Run

```bash
lsusb -d 046d: -v
```



If the access is granted, there will be no errors at the top of the log:

```bash
> lsusb -d 046d: -v

Bus 001 Device 022: ID 046d:c626 Logitech, Inc. 3Dconnexion Space Navigator 3D Mouse
Device Descriptor:
  bLength                18
  bDescriptorType         1
  bcdUSB               2.00
  bDeviceClass            0 
  bDeviceSubClass         0 
  bDeviceProtocol         0 
  bMaxPacketSize0        32
...
```



Otherwise, it will show the `Couldn't open device` error message:

```bash
lsusb -d 046d: -v

Bus 001 Device 024: ID 046d:c626 Logitech, Inc. 3Dconnexion Space Navigator 3D Mouse
Couldn't open device, some information will be missing
Device Descriptor:
  bLength                18
  bDescriptorType         1
  bcdUSB               2.00
  bDeviceClass            0 
  bDeviceSubClass         0 
  bDeviceProtocol         0 
  bMaxPacketSize0        32
...
```





