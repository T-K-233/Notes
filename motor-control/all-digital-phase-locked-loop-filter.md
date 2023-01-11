# Recoil Documentation



On power up, the motor controller will initialize itself. The sequence are:

1. Set mode to MODE\_ERROR with error code of ERROR\_INITIALIZE\_ERROR. This is in case if anything interrupts the initialization, then the system won't start. Also it will load the CAN\_ID and FIRMWARE\_VERSION.
2. Set up CAN filter.
3. Start CAN. Now the motor controller will respond to CAN communication.
4. Initialize virtual Motor.
5. Initialize Encoder.
6. Initialize PowerStage.
7. Initialize CurrentController.
8. Initialize PositionController.
9. Load configuration from Flash.
10. Configure Encoder filter bandwidth.
11. Start PowerStage TIM PWM and ADC sampling.
12. Start OpAmp.
13. Start all base TIM.
14. Start ADC conversion.
15. Configure LED state.
16. Calibrate current sampling offset.
17. Set mode to MODE\_IDLE, and error code to ERROR\_NO\_ERROR. Exit initialization.





