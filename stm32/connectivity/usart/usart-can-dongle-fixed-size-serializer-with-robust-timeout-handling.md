# USART - CAN Dongle (Fixed Size Serializer with Robust Timeout Handling)

We will build a CAN-USB (UART) Adapter in this section.

We will be using the [SerialBus](https://python-can.readthedocs.io/en/stable/interfaces/serial.html) packet format from [python-can](https://python-can.readthedocs.io/en/stable/index.html).

The UART packet will be framed as follows:

|                   | Start of frame | Timestamp                                     | DLC                           | Arbitration ID          | Payload | End of frame |
| ----------------- | -------------- | --------------------------------------------- | ----------------------------- | ----------------------- | ------- | ------------ |
| **Length (Byte)** | 1              | 4                                             | 1                             | 4                       | 0 - 8   | 1            |
| **Data type**     | Byte           | Unsigned 4 byte integer                       | Unsigned 1 byte integer       | Unsigned 4 byte integer | Byte    | Byte         |
| **Byte order**    | -              | Little-Endian                                 | Little-Endian                 | Little-Endian           | -       | -            |
| **Description**   | Must be 0xAA   | Usually s, ms or µs since start of the device | Length in byte of the payload | -                       | -       | Must be 0xBB |

Notice that this is actually a variable-length data packet ---- the packet length can range from 11 to 19 depending on the number of bytes in the payload. However, because we can calculate the size of the entire packet from the DLC field, we still regard this as a fixed-size data packet.

Receiving an unknown length of data will be much more challenging, and involves another entirely different set of mechanisms.

Because the TX and RX process of our CAN dongle is independent, and we need to respond to both of them in time, we will use interrupt-based message handling.

First, we enable the NVIC interrupt on UART2.

<figure><img src="../../../.gitbook/assets/image (6) (3).png" alt=""><figcaption></figcaption></figure>

At the end of code initialization, we start receiving UART using interrupt.

Because we cannot know the length of the data field before we read out the header section, we will receive for 11 bytes here, which handles the following two circumstances:

* If DLC is 0, then we have received all data available, and can go ahead and process the packet. The index 10 will contain the End of Frame.
* If DLC is greater than 0, we have received the header section as well as the first byte of data section. We will index 10 to index 0 of our data, and proceed to receive another `DLC` number of bytes, which will contain the remaining data and the End of Frame.

Because of the two different states mentioned above, we need to define a global variable `uart_rx_data_pending` to signal our interrupt routine which part of the packet we are receiving ---- header part, or the payload part.

```c
uint8_t uart_rx_buffer[64];
uint8_t uart_rx_data_pending = 0U;

void APP_init() {
  // ... other initialization scripts
  uart_rx_data_pending = 0U;
  HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 11);
}
```

Then, in the interrupt handler, we process the data and do state changing.

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (!uart_rx_data_pending) {
    uint8_t is_valid_frame = uart_rx_buffer[0] == 0xAAU;
    if (!is_valid_frame) {
      HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 11);
      return;
    }
    can_tx_frame.id_type = CAN_ID_STANDARD;
    can_tx_frame.frame_type = CAN_FRAME_DATA;
//    uint32_t timestamp = ((uart_rx_buffer[1])     // timestamp is not used
//        | (uart_rx_buffer[2] << 8U)
//        | (uart_rx_buffer[3] << 16U)
//        | (uart_rx_buffer[4] << 24U));
    can_tx_frame.size = uart_rx_buffer[5];
    can_tx_frame.id = ((uart_rx_buffer[6])
        | (uart_rx_buffer[7] << 8U)
        | (uart_rx_buffer[8] << 16U)
        | (uart_rx_buffer[9] << 24U));
    if (can_tx_frame.size) {
      uart_rx_data_pending = 1U;
      can_tx_frame.data[0] = uart_rx_buffer[10];
      HAL_UART_Receive_IT(&huart2, uart_rx_buffer, can_tx_frame.size);
      return;
    }
  }
  else {
    memcpy(can_tx_frame.data+1, uart_rx_buffer, can_tx_frame.size-1);
  }

  CAN_putTxFrame(&hfdcan1, &can_tx_frame);
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);

  uart_rx_data_pending = 0U;
  HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 11);
}

```
