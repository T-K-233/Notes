# FT Debugger Driver Setup



Download VCP Drivers [here](https://ftdichip.com/drivers/vcp-drivers/).

Click the "setup executable" text to download the actual installer.

![](<../.gitbook/assets/image (18).png>)



After download, extract and run the installer.&#x20;

![](<../.gitbook/assets/image (90).png>)

![](<../.gitbook/assets/image (109).png>)

It will install two drivers:

FTDI CDM Driver Package - Bus/D2XX Driver (07/05/2021 2.12.36.4)

FTDI CDM Driver Package - VCP Driver (07/05/2021 2.12.36.4)





Plug in the FT Debugger.

There will be several possible drivers that Windows will decide to assign to it automatically. The correct one will be "USB Composite Device".



If it appears to be a different one, right-click the device and select "Update driver". For example, here Windows recognize the debugger as a "Dual RS232" device.

![](<../.gitbook/assets/image (39).png>)



Select "Browse my computer for drivers".

![](<../.gitbook/assets/image (65).png>)



"Let me pick from a list of available drivers on my computer".

![](<../.gitbook/assets/image (47).png>)



Select the "USB Composite Device"

![](<../.gitbook/assets/image (12).png>)



\<TODO>





This is the correct combination

![](<../.gitbook/assets/image (102).png>)





Additionally, if we want Windows to always select the correct device driver, we can do this by uninstalling other incorrect drivers.

In Device Manager, select the device with the incorrect driver associated (we can do this by manually updating it to the incorrect drivers).

Right-click and select Uninstall device. In the pop-up window, check the "Delete the driver software for this device".&#x20;

Unplug and re-plug the device, repeat this for all the possible incorrect drivers.

![](<../.gitbook/assets/image (50).png>)





Reference

[ESP32 & PIO Unified Debugger](https://community.platformio.org/t/esp32-pio-unified-debugger/4541/20)







