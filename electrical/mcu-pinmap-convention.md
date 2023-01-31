# MCU Pinmap Convention

## As Motor Controller

#### STM32F103CxTx

![](<../.gitbook/assets/image (33) (1).png>)

I2C1 is used for connecting to AS5600 magnetic encoder. If incremental encoder is used, then pin PB8, PB9, and PC13 can be used as ABI input.

SPI1 is used for connecting to DRV8305 gate driver.

PB0-PB2, and PB10-PB11 are free pins to hook up to either gate driver GPIOs or other GPIO functions. PB0 and PB1 also has ADC input capability.

PB3-PB5 are used to drive indicator RGB LEDs. Red and blue channels are connected to TIM3, which can be used to indicate rotation speed and direciton.&#x20;

TIM4 is reserved to be an internal system scheduling timer, but can also be configured to output on PB6 and PB7.



#### STM32F446RET6

![](<../.gitbook/assets/image (69).png>)



#### STM32G431CBT6

<figure><img src="../.gitbook/assets/image (2) (4).png" alt=""><figcaption></figcaption></figure>

