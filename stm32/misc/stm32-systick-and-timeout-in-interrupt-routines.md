# STM32 Systick and Timeout in Interrupt Routines

In STM32, the system time is tracked by the system tick `uwTick`, and this variable is updated by a software interrupt routine, which will be called by timer at 1kHz.

By default, the interrupt routine has the lowest priority 15.

So here comes the question: will the HAL\_Delay, and various blocking functions that uses timeout function correctly in interrupt routines that has higher priority than the systick handler?



## Setup

We use TIM1 update interrupt, and the period is set to be 1 second (1 Hz).



### HAL\_Delay()

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  char str[128];
  sprintf(str, "current tick: %d\n", HAL_GetTick());
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);

  HAL_Delay(500);

  sprintf(str, "after 0.5 s: %d\n", HAL_GetTick());
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

void APP_init() {
  HAL_TIM_Base_Start_IT(&htim1);

}

void APP_main() {
  HAL_Delay(100);
}

```

The code above will get stuck at the HAL\_Delay() function. What makes things worse is that the next timer interrupt cannot interrupt into this, so the entire program is halted.



Now we replace the delay with some computation that does not need uwTick

```c
void someComputationTask() {
  volatile float counter = 1.f;
  for (uint16_t i=0; i<100000; i+=1) {
    counter /= 2.f;
  }
}
```



```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  char str[128];
  sprintf(str, "<interrupt> current tick: %d\n", HAL_GetTick());
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);

  someComputationTask();

  sprintf(str, "and now current tick: %d\n", HAL_GetTick());
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

void APP_init() {
  char str[128];
  sprintf(str, "<blocking> current tick: %d\n", HAL_GetTick());
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);

  someComputationTask();

  sprintf(str, "and now current tick: %d\n", HAL_GetTick());
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);

  HAL_TIM_Base_Start_IT(&htim1);

}

void APP_main() {
  HAL_Delay(100);
}

```





<figure><img src="../../.gitbook/assets/image (1) (1).png" alt=""><figcaption></figcaption></figure>

From the result, we can see that the task should take about 23 ticks to finish, but in the interrupt, the uwTick is not changing.





### Peripheral Blocking Mode Function Call

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  char str[128];
  sprintf(str, "<interrupt>\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);

  uint8_t data = 2;
  uint8_t status = HAL_I2C_Master_Transmit(&hi2c1, 0x00, &data, 1, 100);

  sprintf(str, "transmit finish with: %d\n", status);
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}
```

We now test peripheral call in blocking mode within the interrupt routine. We will use the frustrating I2C protocol, as it's guaranteed to fail with no device connected to the bus.



<figure><img src="../../.gitbook/assets/image (1) (5).png" alt=""><figcaption></figcaption></figure>

Turns out that we indeed get stuck in this blocking function, as the `I2C_WaitOnTXISFlagUntilTimeout` implements a timeout mechanism that expects the uwTick to be updated.
