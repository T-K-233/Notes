# GPIO

## 0. Pin Map

On the STM32F446RET6 Nucleo board, the LD2 LED is connected to PA5, and the USER Button is connected to PC13.

We will demonstrate how to set up a generic GPIO output pin with the LED, and a generic GPIO input pin with the user button.

## 1. Configure STM32

First, set the configuration from the [Starter Project](../getting-started-stm32-edition/running-a-starter-project.md).



Click on **PA5**, set it to "GPIO\_Output".

![](<../../.gitbook/assets/image (107).png>)



Click on **PC13**, set it to "GPIO\_Input".

![](<../../.gitbook/assets/image (16) (1).png>)

Save the .ioc file and generate code.

## 2. Code

First, add the code from the [Starter Project](../getting-started-stm32-edition/running-a-starter-project.md).



In `main.c`, add the following code

```c
  /* USER CODE BEGIN 2 */
  char str[64];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(250);

    uint8_t val = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
    sprintf(str, "button value: %d\r\n", val);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
  }
  /* USER CODE END 3 */
```

![](<../../.gitbook/assets/image (119).png>)



After saving, upload the code

## 3. Result

We can see LED LD2 blinking, and when we press/release the USER Button, the serial output changes.

![](<../../.gitbook/assets/image (28) (1).png>)

