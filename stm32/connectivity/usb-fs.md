# 15. USB FS

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





