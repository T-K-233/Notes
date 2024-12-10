# CAN

## 0. Pin Map

On the STM32F446RET6 Nucleo board, the SPI1 is connected as below

|      |          |   |
| ---- | -------- | - |
| PA11 | CAN1\_RX |   |
| PA12 | CAN1\_TX |   |

## 1. Configure STM32

First, set the configuration from the [Template Project](https://notes.tk233.xyz/stm32/0.-template-project).

Assuming system clock is configured to be **160 MHz**.

In the left sidebar, select **Connectivity** -> **CAN1**.

Enable **Activated** checkbox.

Set **Bit Timings Parameters** -> **Prescaler** to 32. Time Quantum should be 1000 ns.

Set **Bit Timings Parameters** -> **Time Quanta in Bit Segment 1** to 3.

Set **Bit Timings Parameters** -> **Time Quanta in Bit Segment 2** to 1.

This way, we get a **250 kbps** CAN bus with a sampling point at **75%** position.

We can also change the prescaler value to **16** to get a **500 kbps** bus, or **8** to get a **1 Mbps** bus.

The optimal sampling point is **87.5%**, and we are pretty close to it.

This is a useful website for [CAN bit-timing](http://www.bittiming.can-wiki.info/).

![](<../../.gitbook/assets/image (5) (1) (1) (1) (1) (1) (1) (1) (1).png>)

To debug the CAN bus, set **Advanced Parameters** -> **Operating Mode** to "Loopback".

![](<../../.gitbook/assets/image (142).png>)

> Errata: When using loopback mode, the RX pin should still be pulled-up externally. We can do this simply by configuring the PA11 GPIO to enable the internal pull-up resistor, but please remember to do this. Otherwise, the `HAL_CAN_Start()` method will timeout. Detailed information can be found [here](https://electronics.stackexchange.com/questions/353005/can-initialization-timeout-error-in-stm32f4).

## 2. Code

First, add the code from the [Template Project](https://notes.tk233.xyz/stm32/0.-template-project).

In `main.c`, add the following code

```c
  /* USER CODE BEGIN 2 */

  uint8_t counter = 0;

  uint32_t filter_id = 0;
  uint32_t filter_mask = 0x0;

  CAN_FilterTypeDef filter_config;
  filter_config.FilterBank = 0;
  filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
  filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter_config.FilterIdHigh = filter_id << 5;
  filter_config.FilterIdLow = 0;
  filter_config.FilterMaskIdHigh = filter_mask << 5;
  filter_config.FilterMaskIdLow = 0;
  filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
  filter_config.FilterActivation = CAN_FILTER_ENABLE;
  filter_config.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan1, &filter_config);

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    while (1)
    HAL_UART_Transmit(&huart2, (uint8_t *) "CAN init Error\r\n", strlen("CAN init Error\r\n"), 100);
  }

  HAL_Delay(2000);
  /* USER CODE END 2 */
```

```c
    /* USER CODE BEGIN 3 */
    // CAN TX
    uint32_t tx_mailbox;

    CAN_TxHeaderTypeDef tx_header;
    tx_header.DLC = 4;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.StdId = 0x00A;
    tx_header.TransmitGlobalTime = DISABLE;

    uint8_t tx_data[8];
    tx_data[0] = counter;
    tx_data[1] = 0x07;
    tx_data[2] = 0x08;
    tx_data[3] = 0x09;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
      HAL_UART_Transmit(&huart2, (uint8_t *) "CAN TX Error\r\n", strlen("CAN TX Error\r\n"), 100);
    }

    HAL_Delay(1);

    // CAN RX
    uint32_t rx_fifo_level = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) || HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1);

    char rx_level_str[50];
    sprintf(rx_level_str, "level: %d\r\n", rx_fifo_level);
    HAL_UART_Transmit(&huart2, (uint8_t *)rx_level_str, strlen(rx_level_str), 100);

    if (rx_fifo_level > 0) {
      HAL_UART_Transmit(&huart2, (uint8_t *)"CAN msg pending\r\n", strlen("CAN msg pending\r\n"), 100);

      CAN_RxHeaderTypeDef rx_header;
      uint8_t rx_data[8];

      HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);

      char rx_data_str[32];
      sprintf(rx_data_str, "receive data: %d\r\n", rx_data[0]);
      HAL_UART_Transmit(&huart2, (uint8_t *)rx_data_str, strlen(rx_data_str), 100);
    }

    counter += 1;
    HAL_Delay(100);

  }
  /* USER CODE END 3 */
```

After saving, upload the code.

Detailed information on how CAN filter works can be found [here](https://schulz-m.github.io/2017/03/23/stm32-can-id-filter/).

## 3. Result

## 4. Using both CAN1 and CAN2

{% embed url="https://stackoverflow.com/questions/65290032/using-both-can1-can2-both-in-stm32f446-properly" %}

{% embed url="http://www.bittiming.can-wiki.info/" %}

## 5. Interrupt

```c
HAL_CAN_ConfigFilter(&hcan1, &filter_config);
HAL_CAN_Start(&hcan1);
HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
```
