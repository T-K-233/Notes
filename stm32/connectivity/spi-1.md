# SPI - GC9A01A LCD Screen

## 0. Connection

| STM32 | GC9A01A |                        |
| ----- | ------- | ---------------------- |
| 3V3   | VIN     | 5V or 3V3 power supply |
| GND   | GND     | Ground                 |
| PA5   | SCL     | SPI 1 SCLK             |
| PA7   | SDA     | SPI 1 MOSI             |
| PA9   | RES     | IC Reset N             |
| PA8   | DC      | Data / Command select  |
| PB6   | CS      | SPI 1 Chip Select N    |
| PC7   | BLK     | Backlight enable       |



## 1. Configure STM32

In the left sidebar, select **Connectivity** -> **SPI1**.

Select **Mode** to "Transmit Only Master".

Select **Hardware NSS Signal** to "Disable".

Set SPI Mode to MODE0

Set **Clock Parameters** -> **Prescaler** to 2. The chip can run up to 40 Mbits/s.

<figure><img src="../../.gitbook/assets/image (218).png" alt=""><figcaption></figcaption></figure>



## 2. Code

The code is adapted from Adafruit library

{% embed url="https://github.com/adafruit/Adafruit_GC9A01A/blob/main/Adafruit_GC9A01A.h" %}

{% embed url="https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_SPITFT.h" %}

One tricky think is the SPI\_WRITE16 function. The byte order is easy to mess up.

In STM, when passing the uint16\_t value to the transmit function, the lower byte in the memory address is transmitted first. However, we want to maintain the byte order, and thus we need to swap the uint16\_t btyes before we invoke the transmission function.





## 3. Result

After connecting an RFID-RC522 SPI module, we can see that we can read the register from the sensor. The SPI signal looks like this:

![](<../../.gitbook/assets/image (118).png>)

[https://community.st.com/s/question/0D53W00001nAhvYSAS/how-to-use-spi-nss-on-stm32g0](https://community.st.com/s/question/0D53W00001nAhvYSAS/how-to-use-spi-nss-on-stm32g0)
