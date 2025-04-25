---
description: Using Analog to Digital Converter on STM32
---

# Using ADC on STM32

## Software Trigger Polling Mode



Set **External Trigger Conversion Source** to be "Regular Conversion launched by software"

<figure><img src="../../.gitbook/assets/image (245).png" alt=""><figcaption></figcaption></figure>



```c
HAL_ADC_Start(&hadc1);
HAL_StatusTypeDef status = HAL_ADC_PollForConversion(&hadc1, 100);
uint32_t value = (int32_t)HAL_ADC_GetValue(&hadc1);
```





## Timer Triggered Mode

It is more convenient to use injected channel.

<figure><img src="../../.gitbook/assets/image (247).png" alt=""><figcaption></figcaption></figure>



Configure Timer 2 to generate reset event on TRGO at 1 kHz.

The 1 kHz is calculated from `APB1_TIMER_CLOCKS / (PSC + 1) / (ARR + 1)`:

$$
\frac{80 MHz}{(79 + 1) \times (999 + 1)} = 1 kHz
$$

<figure><img src="../../.gitbook/assets/image (8).png" alt=""><figcaption></figcaption></figure>







