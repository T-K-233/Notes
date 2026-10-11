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

## Using OPAMP Outputs as ADC Inputs

To use an OPAMP output as an ADC input, enable the OPAMP and set the ADC to read the OPAMP output channel (STM32F3 reference manual, section 18.3.4):

| OPAMP  | ADC channel    |
| ------ | -------------- |
| OPAMP1 | ADC1 channel 3 |
| OPAMP2 | ADC2 channel 3 |
| OPAMP3 | ADC3 channel 1 |
| OPAMP4 | ADC4 channel 3 |

OPAMP1, OPAMP3, and OPAMP4 are only available on STM32F303xB/C/D/E, STM32F358xC, and STM32F398xE.
