# MIT Motor Controller Code Analyze

{% embed url="https://github.com/bgkatz/motorcontrol" %}

## Clock Tree

<figure><img src="../.gitbook/assets/image (2) (3) (1).png" alt=""><figcaption></figcaption></figure>

## NVIC Settings

<figure><img src="../.gitbook/assets/image (3) (1) (5).png" alt=""><figcaption></figcaption></figure>

NVIC priority is configured in the code

```c
HAL_NVIC_SetPriority(PWM_ISR, 0x0,0x0); // commutation > communication
HAL_NVIC_SetPriority(CAN_ISR, 0x01, 0x01);
```



## PowerStage PWM Generation

<figure><img src="../.gitbook/assets/image (3) (5) (1).png" alt=""><figcaption></figcaption></figure>

Using DIV/1 prescaler, 0x8CA (2250) period, and center aligned mode -> PWM frequency is 80kHz

With the repetition counter set to 1, the software interrupt frequency is 40kHz.



<figure><img src="../.gitbook/assets/image (31) (2).png" alt=""><figcaption></figcaption></figure>

It handles the motor phase inversion at the last stage, before issuing value to TIM.





## Current Sampling

<figure><img src="../.gitbook/assets/image (4) (7) (1).png" alt=""><figcaption></figcaption></figure>

It's using 2 phase current shunt, and using blocking ADC call to sample current.



## Encoder

<figure><img src="../.gitbook/assets/image (1) (4) (2).png" alt=""><figcaption></figcaption></figure>

It's using blocking encoder read/write.



Using this line to fence off too-fast CS pin control:

```c
HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 1, 100);
while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
```

This shouldn't be necessary when using blocking SPI transaction mode...



## The Commutation Loop

```c
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
	//HAL_GPIO_WritePin(LED, GPIO_PIN_SET );	// Useful for timing

	/* Sample ADCs */
	analog_sample(&controller);

	/* Sample position sensor */
	ps_sample(&comm_encoder, DT);

	/* Run Finite State Machine */
	run_fsm(&state);

	/* Check for CAN messages */
	can_tx_rx();

	/* increment loop count */
	controller.loop_count++;
	//HAL_GPIO_WritePin(LED, GPIO_PIN_RESET );

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}
```









