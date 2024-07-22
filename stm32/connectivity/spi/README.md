# SPI

## 0. Pin Map

On the STM32F446RET6 Nucleo board, the SPI1 is connected as below

|     |      |              |
| --- | ---- | ------------ |
| PA4 | CS   | A2           |
| PA5 | MOSI | PWM/MOSI/D11 |
| PA6 | MISO | MISO/D12     |
| PA7 | SCLK | SCK/D13      |

## 1. Configure STM32

First, set the configuration from the [Template Project](https://notes.tk233.xyz/stm32/0.-template-project).

Set PA4 as GPIO Output.

In the left sidebar, select **Connectivity** -> **SPI1**.

Select **Mode** to "Full-Duplex Master".

Select **Hardware NSS Signal** to "Disable".

Set **Clock Parameters** -> **Prescaler** to 128 or more, to ensure the Baud Rate is slower than 1Mbits/s.

![](<../../../.gitbook/assets/image (135) (1).png>)

> Errata: the hardware-driven NSS on STM32 is a bit mysterious. It seems that we need to deinit the SPI module and re-init to control the CS pin. Also, the CS pin is open-drain, so it requires an external pull-up resistor to function. We will save all the hassle and just use software-controlled CS pin. Detailed discussion of hardware NSS can be found [here](https://stackoverflow.com/questions/35780290/how-to-use-hardware-nss-spi-on-stm32f4).



### SPI Mode

Clock Polarity: CPOL

Clock Phase: CPHA

<table><thead><tr><th width="122">Mode</th><th width="152">Clock Polarity</th><th width="154">Clock Phase</th><th>Output Edge</th><th>Data Capture</th></tr></thead><tbody><tr><td>MODE 0</td><td>Low</td><td>1 Edge</td><td>Falling</td><td>Rising</td></tr><tr><td>MODE 1</td><td>Low</td><td>2 Edge</td><td>Rising</td><td>Falling</td></tr><tr><td>MODE 2</td><td>High</td><td>1 Edge</td><td>Rising</td><td>Falling</td></tr><tr><td>MODE 3</td><td>High</td><td>2 Edge</td><td>Falling</td><td>Rising</td></tr></tbody></table>







## 2. Code

First, add the code from the [Template Project](https://notes.tk233.xyz/stm32/0.-template-project).

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
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    uint8_t spi_tx_data[4];
    uint8_t spi_rx_data[4];
    spi_tx_data[0] = 0b10000000 | (0x36 << 1);
    spi_tx_data[1] = 0;
    spi_tx_data[2] = 0;
    spi_tx_data[3] = 0;
    HAL_SPI_TransmitReceive(&hspi1, spi_tx_data, spi_rx_data, 2, 100);
    while (hspi1.State == HAL_SPI_STATE_BUSY) {}
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    sprintf(str, "TX: %d %d %d %d\tRX: %d %d %d %d\r\n",
        spi_tx_data[0],
        spi_tx_data[1],
        spi_tx_data[2],
        spi_tx_data[3],
        spi_rx_data[0],
        spi_rx_data[1],
        spi_rx_data[2],
        spi_rx_data[3]
        );
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
    HAL_Delay(100);
  }
  /* USER CODE END 3 */
```

After saving, upload the code.

## 3. Result

After connecting an RFID-RC522 SPI module, we can see that we can read the register from the sensor. The SPI signal looks like this:

![](<../../../.gitbook/assets/image (118).png>)

[https://community.st.com/s/question/0D53W00001nAhvYSAS/how-to-use-spi-nss-on-stm32g0](https://community.st.com/s/question/0D53W00001nAhvYSAS/how-to-use-spi-nss-on-stm32g0)
