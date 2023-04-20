# FT LINK JTAG Debugger Driver Setup



Download FT2232 Drivers from [here](https://ftdichip.com/drivers/vcp-drivers/). Select the VCP driver.

Scroll down and click the "setup executable" text to download the executable installer.

![](<../.gitbook/assets/image (18) (1) (1).png>)



After download, extract and run the installer.&#x20;

![](<../.gitbook/assets/image (90).png>)

![](<../.gitbook/assets/image (109).png>)

It will install two drivers:

FTDI CDM Driver Package - Bus/D2XX Driver (07/05/2021 2.12.36.4)

FTDI CDM Driver Package - VCP Driver (07/05/2021 2.12.36.4)





Plug in the FT LINK debugger.

After the FT LINK debugger is inserted, now we should see two USB Serial Converters under the "Universal Serial Bus controllers" tag in device manager

<figure><img src="../.gitbook/assets/image (3) (1) (2).png" alt=""><figcaption></figcaption></figure>



Then, we use [zadig](https://zadig.akeo.ie/#google\_vignette) to load the correct driver of the device.

Open zadig, check Options -> List All Devices.

In the main dropdown menu, select "Dual RS232 (Interface 0). Make sure the Interface 0 is selected, instead of 1.

Select the target driver to be "WinUSB", and click the "Replace Driver" button.

<figure><img src="../.gitbook/assets/image (2) (3) (1) (1).png" alt=""><figcaption></figcaption></figure>



After loading the driver, only USB Serial Converter B should be shown under the Universal Serial Bus controllers. Interface 0 should be shown as Dual RS232 device under Universal Serial Bus devices.

<figure><img src="../.gitbook/assets/image (1) (3) (2) (1).png" alt=""><figcaption></figcaption></figure>



#### Fixing Driver

It's possible that sometimes even after setting up the debugger following the instructions above, the driver is broken. Here's the procedure of resetting the driver.



If in the device manager, the device appears to be a different name, right-click the device and select "Update driver". For example, here Windows recognize the debugger as a "Dual RS232" device.

![](<../.gitbook/assets/image (39).png>)



Select "Browse my computer for drivers".

![](<../.gitbook/assets/image (65).png>)



"Let me pick from a list of available drivers on my computer".

![](<../.gitbook/assets/image (47).png>)



Select the "USB Composite Device"

![](<../.gitbook/assets/image (12) (1) (1) (1).png>)



\<TODO>





Additionally, if we want Windows to always select the correct device driver, we can do this by uninstalling other incorrect drivers.

In Device Manager, select the device with the incorrect driver associated (we can do this by manually updating it to the incorrect drivers).

Right-click and select Uninstall device. In the pop-up window, check the "Delete the driver software for this device".&#x20;

Unplug and re-plug the device, repeat this for all the possible incorrect drivers.

![](<../.gitbook/assets/image (50).png>)





Reference

[ESP32 & PIO Unified Debugger](https://community.platformio.org/t/esp32-pio-unified-debugger/4541/20)







