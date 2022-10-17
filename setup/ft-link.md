# FT LINK

Download FT2232 Driver from [here](https://ftdichip.com/products/ft2232hq/). Select the VCP driver.

<figure><img src="../.gitbook/assets/Screenshot 2022-10-03 161543.png" alt=""><figcaption></figcaption></figure>



Scroll down and select the setup executable option.

<figure><img src="../.gitbook/assets/Screenshot 2022-10-03 161452.png" alt=""><figcaption></figcaption></figure>



<figure><img src="../.gitbook/assets/image (9).png" alt=""><figcaption></figcaption></figure>

After the FT debugger is inserted, now we should see two USB Serial Converters under the "Universal Serial Bus controllers" tag in device manager

<figure><img src="../.gitbook/assets/image (3).png" alt=""><figcaption></figcaption></figure>



Then, we use [zadig](https://zadig.akeo.ie/#google\_vignette) to load the correct driver of the device.

Open zadig, check Options -> List All Devices.

In the main dropdown menu, select "Dual RS232 (Interface 0). Make sure the Interface 0 is selected, instead of 1.

Select the target driver to be "WinUSB", and click the "Replace Driver" button.

<figure><img src="../.gitbook/assets/image (2).png" alt=""><figcaption></figcaption></figure>



After loading the driver, only USB Serial Converter B should be shown under the Universal Serial Bus controllers. Interface 0 should be shown as Dual RS232 device under Universal Serial Bus devices.

<figure><img src="../.gitbook/assets/image (6).png" alt=""><figcaption></figcaption></figure>
