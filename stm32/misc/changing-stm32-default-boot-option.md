# Changing STM32 Default Boot Option

## STM32F1

TBD



## STM32G4

By factory default, STM32G4 MCU will select the boot mode from BOOT0 and BOOT1 pins. However, we may want to use those pins for other IO functions.

For STM32G4 series, we can configure the default boot method by setting the correct value in the Flash option bytes.



### Configuration Fields

<figure><img src="../../.gitbook/assets/image (115).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (132).png" alt=""><figcaption></figcaption></figure>

The Flash option bytes are locked during normal operation, and there's a special sequence that we need to execute in order to unlock it.

<figure><img src="../../.gitbook/assets/image (133).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (130).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (111).png" alt=""><figcaption></figcaption></figure>

We use HAL functions to achieve this.



By default, the value stored in the option byte is `0xFFEFF8AA`, which means that the device will boot from boot pin's selection.

<figure><img src="../../.gitbook/assets/image (127).png" alt=""><figcaption></figcaption></figure>

In order to force it boot from flash, we need to clear the nSWBOOT0 pin, thus changing it to `0xFBEFF8AA`.



### Code



```c
/**
 * Procedure following G431 User Manual Section 4.4.2 Option bytes programming
 *
 */
void APP_initFlashOption() {
  // 1. Unlock the FLASH_CR with the LOCK clearing sequence
  // Check that no Flash memory operation is on going by checking the BSY bit in the Flash status register (FLASH_SR).
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}
  HAL_FLASH_Unlock();

  // 2. Unlock the FLASH Option Byte with the LOCK clearing sequence
  // Check that no Flash memory operation is on going by checking the BSY bit in the Flash status register (FLASH_SR).
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}
  HAL_FLASH_OB_Unlock();

  // 3. program OPTR
  FLASH->OPTR = 0xFBEFF8AAU;  // default to boot from flash

  // 4. Set the Options Start bit OPTSTRT in the Flash control register (FLASH_CR).
  SET_BITS(FLASH->CR, FLASH_CR_OPTSTRT);

  // 4.1 clear status register
  SET_BITS(FLASH->SR, FLASH_SR_OPTVERR | FLASH_SR_RDERR);

  // 5. Wait for the BSY bit to be cleared.
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}

  // 6. Lock Flash
  // If LOCK is set by software, OPTLOCK is automatically set too
  HAL_FLASH_Lock();

  // 7. reload the new settings
  // seems this line will cause error when put before FLASH_Lock(), which will then corrupt all Flash settings
  // so putting it here
  // can also comment out this line and just power cycle to update the flash settings
  HAL_FLASH_OB_Launch();

  while (1) {
    char str[64];
    sprintf(str, "Flash option change finished. Please power cycle the device.\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
    HAL_Delay(100);
  }
}

void APP_init() {
  // only include this when first-time programming the chip
  APP_initFlashOption();
}
```



### Errata

And sometimes, it's possible that the boot pin is tied to high, and the device will boot to SRAM. Our code, however, is always assuming the device is boot into Flash. This will cause us unable to run this changing boot address code.

To temporarily change the code so that it will assume the device starts from SRAM, we need to uncomment the `USER_VECT_TAB_ADDRESS` in `system_stm32g4xx.c`. This way, the vector table will be allocated to the correct address, and the reset handler can direct the program counter to correct location.&#x20;

<figure><img src="../../.gitbook/assets/image (141).png" alt=""><figcaption></figcaption></figure>

After changing, we need to power cycle the MCU to make it take effect.&#x20;



## STM32F4

TBD

