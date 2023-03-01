# USB - FS

## Clock Settings

USB requires a relatively precise 48MHz clock supplied to the peripheral module. Thus, it requires us to use external crystal as the clock source.

In the RCC section, enable HSE.

<figure><img src="../../.gitbook/assets/image (5) (1).png" alt=""><figcaption></figcaption></figure>

Configure the clock tree to supply 48MHz to USB clock domain.





## As Virtual COM Port (COM)

1.Enable USB FS support

<figure><img src="../../.gitbook/assets/image (2) (2) (3).png" alt=""><figcaption></figcaption></figure>

Enable USB\_DEVICE

Select class as "Communication Device Class (Vistual Port Com)"

<figure><img src="../../.gitbook/assets/image (4) (8).png" alt=""><figcaption></figcaption></figure>

Generate code



In addition to the default folders, it will also generate "USB\_DEVICE" folder.

<figure><img src="../../.gitbook/assets/image (1) (2).png" alt=""><figcaption></figcaption></figure>

In the usbd\_cdc\_if.c file, we have the receive and transmit functions

Every message received will invoke the `static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)` function.

And we can use the `uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)` function to transmit data.



To build a simple echo program, we can put the transmit in the receive handler

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



Or a hello world program:

app.c

```c
APP_main() {
  char str[128];
  sprintf(str, "hello\n");
  CDC_Transmit_FS((uint8_t *)str, strlen(str));
  HAL_Delay(1000);
}
```

<figure><img src="../../.gitbook/assets/image (3) (2).png" alt=""><figcaption></figcaption></figure>







main.c:

```c

//    sprintf(str, "hello world\n");
//    sprintf((char *)usb_tx_buffer, "hello USB\n");
//    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 1000);
//
//    CDC_Transmit_FS(usb_tx_buffer, strlen((char *)usb_tx_buffer));
//
//    if (usb_rx_size != 0) {
//      sprintf((char *)usb_tx_buffer, "USB RX: ");
//      while (CDC_Transmit_FS(usb_tx_buffer, strlen((char *)usb_tx_buffer)) != USBD_OK) {}
//      while (CDC_Transmit_FS(usb_rx_buffer, strlen((char *)usb_rx_buffer)) != USBD_OK) {}
//      sprintf((char *)usb_tx_buffer, "\n");
//      while (CDC_Transmit_FS(usb_tx_buffer, strlen((char *)usb_tx_buffer)) != USBD_OK) {}
//      usb_rx_size = 0;
//    }
//    HAL_Delay(1000);




```



usbd\_cdc\_if.c:

```c
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  usb_rx_size = (uint16_t) *Len;
  memset(usb_rx_buffer, 0, 256);
  memcpy(usb_rx_buffer, Buf, usb_rx_size);
  memset(Buf, 0, usb_rx_size);
  return (USBD_OK);
  /* USER CODE END 6 */
}

```





