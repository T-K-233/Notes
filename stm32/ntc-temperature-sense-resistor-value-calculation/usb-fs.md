# USB - FS

## Clock Settings

USB requires a relatively precise 48MHz clock supplied to the peripheral module. Thus, it requires us to use external crystal as the clock source.

In the RCC section, enable HSE.

<figure><img src="../../.gitbook/assets/image (5) (1) (1).png" alt=""><figcaption></figcaption></figure>

Configure the clock tree to supply 48MHz to USB clock domain.





## As Virtual COM Port (COM)

1.Enable USB FS support

<figure><img src="../../.gitbook/assets/image (2) (2) (3).png" alt=""><figcaption></figcaption></figure>

Enable USB\_DEVICE

Select class as "Communication Device Class (Vistual Port Com)"

<figure><img src="../../.gitbook/assets/image (4) (8).png" alt=""><figcaption></figcaption></figure>

Generate code



In addition to the default folders, it will also generate "USB\_DEVICE" folder.

<figure><img src="../../.gitbook/assets/image (1) (2) (2).png" alt=""><figcaption></figcaption></figure>

In the usbd\_cdc\_if.c file, we have the receive and transmit functions

Every message received will invoke the `static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)` function.

And we can use the `uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)` function to transmit data.



### Simple Echo USB VCP Device

To build a simple echo program, we just need to implement the transmit in the USB receive handler.

```c
/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  CDC_Transmit_FS(Buf, *Len);

  return (USBD_OK);
  /* USER CODE END 6 */
}

```



### CAN-USB Adapter

Here's a more complete code example, implementing CAN-USB message transfer.



```c
void APP_main() {
  char str[128];
  sprintf(str, "hello\n");
  CDC_Transmit_FS((uint8_t *)str, strlen(str));
  HAL_Delay(1000);
}
```







main.c:

```c
void APP_handleUSBMessage() {
  // check if the first byte is the correct Start of Frame
  uint8_t is_valid_frame = usb_rx_buffer[0] == 0xAAU;
  if (!is_valid_frame) {
    // if not, discard and continue receiving
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    return;
  }

  // decode the header section
  can_tx_frame.id_type = CAN_ID_STANDARD;
  can_tx_frame.frame_type = CAN_FRAME_DATA;
  uint32_t timestamp = ((uart_rx_buffer[1])     // timestamp is not used
                      | (uart_rx_buffer[2] << 8U)
                      | (uart_rx_buffer[3] << 16U)
                      | (uart_rx_buffer[4] << 24U));
  can_tx_frame.size = usb_rx_buffer[5];
  can_tx_frame.id = (((uint32_t)usb_rx_buffer[6] << 0U)
                   | ((uint32_t)usb_rx_buffer[7] << 8U)
                   | ((uint32_t)usb_rx_buffer[8] << 16U)
                   | ((uint32_t)usb_rx_buffer[9] << 24U));


  for (uint16_t i=0; i<can_tx_frame.size; i+=1) {
    can_tx_frame.data[i] = usb_rx_buffer[10+i];
  }

  // does not really need this piece of code
//  if (!HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)) {
//    uint32_t fifo_idx = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
//    HAL_FDCAN_AbortTxRequest(&hfdcan1, fifo_idx);
//  }
  CAN_putTxFrame(&hfdcan1, &can_tx_frame);
  usb_evt_happened = 1;
}

void APP_handleCANMessage() {
  CAN_getRxFrame(&hfdcan1, &can_rx_frame);

  // prepare the USB frame
  usb_tx_buffer[0] = PYTHONCAN_START_OF_FRAME;

  usb_tx_buffer[1] = 0x00U;  // Timestamp
  usb_tx_buffer[2] = 0x00U;
  usb_tx_buffer[3] = 0x00U;
  usb_tx_buffer[4] = 0x00U;

  usb_tx_buffer[5] = can_rx_frame.size;  // DLC

  usb_tx_buffer[6] = READ_BITS(can_rx_frame.id, 0xFFU);  // ID
  usb_tx_buffer[7] = READ_BITS(can_rx_frame.id >> 8U, 0xFFU);
  usb_tx_buffer[8] = READ_BITS(can_rx_frame.id >> 16U, 0xFFU);
  usb_tx_buffer[9] = READ_BITS(can_rx_frame.id >> 24U, 0xFFU);

  usb_tx_size = 10;

  for (uint16_t i=0; i<can_rx_frame.size; i+=1) {
    usb_tx_buffer[10+i] = can_rx_frame.data[i];
  }
  usb_tx_size += can_rx_frame.size + 1;

  usb_tx_buffer[10+can_rx_frame.size] = PYTHONCAN_END_OF_FRAME;

  CDC_Transmit_FS(usb_tx_buffer, usb_tx_size);
  can_evt_happened = 1;
}

```



usbd\_cdc\_if.c:

```c
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  usb_rx_size = (uint16_t) *Len;

  // clear the receive buffer
  memset(usb_rx_buffer, 0, USB_BUFFER_SIZE);

  // copy the received data to the receive buffer
  memcpy(usb_rx_buffer, Buf, usb_rx_size);

  // clear the original buffer
  memset(Buf, 0, usb_rx_size);

  APP_handleUSBMessage();

  return (USBD_OK);
  /* USER CODE END 6 */
}

```





