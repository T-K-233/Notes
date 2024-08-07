# Solving Gamepad not Detected on Ubuntu 22.04

The symptom is that when the Xbox controller device is plugged in, the system does not register it as a gamepad, and hence fail to create the `/dev/input/js0` directory.



First, we use the following command to monitor the kernel log.

```bash
sudo dmesg -w
```



The log is shown below. Note that the controller is only registered as a USB device, but not a HID device. I then plugged in a normal keyboard, and the system register that as an input device correctly.&#x20;

```bash
[  341.476545] usb 1-9: new full-speed USB device number 10 using xhci_hcd
[  341.625847] usb 1-9: New USB device found, idVendor=1532, idProduct=0a29, bcdDevice= 1.01
[  341.625853] usb 1-9: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[  341.625855] usb 1-9: Product: Razer Wolverine V2
[  341.625856] usb 1-9: Manufacturer: Razer
[  341.625858] usb 1-9: SerialNumber: 0000180FB79F929D
[  365.724632] usb 1-10: new low-speed USB device number 11 using xhci_hcd
[  365.884049] usb 1-10: New USB device found, idVendor=03f0, idProduct=2f4a, bcdDevice= 0.11
[  365.884060] usb 1-10: New USB device strings: Mfr=1, Product=2, SerialNumber=0
[  365.884064] usb 1-10: Product: HP Business Slim Keyboard
[  365.884068] usb 1-10: Manufacturer: Chicony
[  365.888935] input: Chicony HP Business Slim Keyboard as /devices/pci0000:00/0000:00:14.0/usb1/1-10/1-10:1.0/0003:03F0:2F4A.0005/input/input21
[  365.948931] hid-generic 0003:03F0:2F4A.0005: input,hidraw4: USB HID v1.10 Keyboard [Chicony HP Business Slim Keyboard] on usb-0000:00:14.0-10/input0
[  365.953729] input: Chicony HP Business Slim Keyboard Consumer Control as /devices/pci0000:00/0000:00:14.0/usb1/1-10/1-10:1.1/0003:03F0:2F4A.0006/input/input22
[  366.012837] input: Chicony HP Business Slim Keyboard System Control as /devices/pci0000:00/0000:00:14.0/usb1/1-10/1-10:1.1/0003:03F0:2F4A.0006/input/input23
[  366.012945] input: Chicony HP Business Slim Keyboard as /devices/pci0000:00/0000:00:14.0/usb1/1-10/1-10:1.1/0003:03F0:2F4A.0006/input/input24
[  366.013113] hid-generic 0003:03F0:2F4A.0006: input,hiddev0,hidraw5: USB HID v1.10 Device [Chicony HP Business Slim Keyboard] on usb-0000:00:14.0-10/input1
[  368.740323] usb 1-10: USB disconnect, device number 11
```



I have tried many, many method on the Internet, but turns out Chat has the best solution&#x20;

[https://chatgpt.com/share/dc8b6af3-bf1a-4741-90db-0e3b0e2cea38](https://chatgpt.com/share/dc8b6af3-bf1a-4741-90db-0e3b0e2cea38)

### **1. Install `xboxdrv`**

`xboxdrv` is a driver for Xbox controllers. You can install it using the following command:

```bash
sudo apt-get install xboxdrv
```

### **2. Check if `xpad` module is loaded**

The `xpad` module is the default driver for Xbox controllers on Linux. Ensure it is loaded:

```bash
lsmod | grep xpad
```

If it is not loaded, load it using:

```bash
sudo modprobe xpad
```



### **3. Add Udev Rules**

Sometimes, custom udev rules can help. Create a new udev rule for Xbox controllers:

```bash
sudo nano /etc/udev/rules.d/50-xboxcontroller.rules
```

Add the following content:

{% code title="50-xboxcontroller.rules" %}
```bash
# Microsoft Corp. Xbox Wireless Controller (model 1914)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="0b12", GROUP:="users", MODE:="0666", ENV{ID_INPUT_JOYSTICK}:="1"

SUBSYSTEMS=="usb", ATTRS{idVendor}=="02d1", ATTRS{idProduct}=="0a29", GROUP:="users", MODE:="0666", ENV{ID_INPUT_JOYSTICK}:="1"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="02ea", GROUP:="users", MODE:="0666", ENV{ID_INPUT_JOYSTICK}:="1"

# Razer USA, Ltd Razer Wolverine V2
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1532", ATTRS{idProduct}=="0a29", GROUP:="users", MODE:="0666", ENV{ID_INPUT_JOYSTICK}:="1"
```
{% endcode %}

Replace `idVendor` and `idProduct` with the corresponding values from your device.



Reload udev rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then, see if lsusb can see the full information of the device

```bash
lsusb -d 1532: -v
```



### **4. Test with `jstest`**

Install `jstest-gtk` to test your joystick:

```bash
sudo apt install jstest-gtk
```

Run it to see if your controller is recognized:

```bash
jstest-gtk
```











