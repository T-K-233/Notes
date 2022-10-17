# Changing STM32 Default Boot Option

By factory default, STM will select the boot mode from BOOT0 and BOOT1 pins. However, we may want to use those pins for other IO functions.

Fortunately, for STM32G4 series, we can configure the default boot method by setting the correct value in the Flash option bytes.



<figure><img src="../../.gitbook/assets/image (115).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (132).png" alt=""><figcaption></figcaption></figure>

The Flash option bytes are locked during normal operation, and there's a special sequence that we need to execute in order to unlock it.

<figure><img src="../../.gitbook/assets/image (133).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (130).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (111).png" alt=""><figcaption></figcaption></figure>

We use HAL functions to achieve this.



By default, the value stored in the option byte is `0xFFEFF8AA` which means that the device will boot from boot pin's selection.

<figure><img src="../../.gitbook/assets/image (127).png" alt=""><figcaption></figcaption></figure>

In order to force it boot from flash, we need to clear the nSWBOOT0 pin, thus changing it to `0xFBEFF8AA`.



Here's the code:

```c
void APP_init() {
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}

  HAL_FLASH_Unlock();
  HAL_FLASH_OB_Unlock();

  FLASH->OPTR = 0xFBEFF8AA;  // default to boot from flash

  SET_BITS(FLASH->CR, FLASH_CR_OPTSTRT);

  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}

  HAL_FLASH_Lock();
  HAL_FLASH_OB_Launch();  // reload the new settings
}

void APP_main() {
}
```



And sometimes, it's possible that the boot pin is tied to high, and the device will boot to SRAM. Our code, however, is always assuming the device is boot into Flash. This will cause us unable to run this changing boot address code.

To temporarily change the code so that it will assume the device starts from SRAM, we need to uncomment the `USER_VECT_TAB_ADDRESS` in `system_stm32g4xx.c`. This way, the vector table will be allocated to the correct address, and the reset handler can direct the program counter to correct location.&#x20;

<figure><img src="../../.gitbook/assets/image (141).png" alt=""><figcaption></figcaption></figure>

After changing, we need to power cycle the MCU to make it take effect.&#x20;
