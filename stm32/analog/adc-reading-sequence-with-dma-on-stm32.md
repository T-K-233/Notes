# ADC Reading Sequence with DMA on STM32



Setup

ADC is configured to sample one regular channel, trigger sourced set to Timer 2 TRGO event.

<figure><img src="../../.gitbook/assets/image (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>

DMA is configured to use normal mode and 16bit transfer size.

<figure><img src="../../.gitbook/assets/image (3) (1) (1).png" alt=""><figcaption></figcaption></figure>



Timer 2 is configured to generate a 10 kHz signal on TRGO with counter update event.

<figure><img src="../../.gitbook/assets/image (2) (1) (1).png" alt=""><figcaption></figcaption></figure>



Code

```c

#define ADC_SAMPLES    128

uint8_t completed = 0;
uint16_t adc_data_buffer[ADC_SAMPLES];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  // this function will be invoked after DMA finishes all transfers
  completed = 1;
}

void APP_init() {
  // start the timer, which is the trigger source of ADC conversion
  HAL_TIM_Base_Start(&htim2);
}

void APP_main() {
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_data_buffer, ADC_SAMPLES);

  while (!completed) {}
  completed = 0;
  
  for (size_t i=0; i<ADC_SAMPLES; i+=1) {
    sprintf(str, "0 %d 4096\n", adc_data_buffer[i]);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
  }

  HAL_Delay(100);
}

```





The terminal correctly prints the audio waveform of a 800 Hz test sound.

<figure><img src="../../.gitbook/assets/image (4) (1) (1).png" alt=""><figcaption></figcaption></figure>





