# Using ADC on STM32



```c
HAL_ADC_Start(&hadc1);
HAL_StatusTypeDef status = HAL_ADC_PollForConversion(&hadc1, 100);
uint32_t value = (int32_t)HAL_ADC_GetValue(&hadc1);
```







