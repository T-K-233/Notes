# New Controller Board Soldering & Power-on Checklist

### Soldering Procedure Checklist

* [ ] Add solder to Encoder pin #7 (opposite to dot).
* [ ] Attach Encoder to pin #7. Make sure that the encoder is centered without offset (rotational offset is acceptable).
* [ ] Add solder to Encoder pin #1 to secure the encoder position.
* [ ] Add flux to both sides.
* [ ] Solder each joint. Pay special attention to the two GND pins is also soldered properly (pin #5, #13).
* [ ] Flip the board.
* [ ] Add solder to 5 power pads.
* [ ] Add solder to motor phase leads.
* [ ] Add flux to pads.
* [ ] Solder motor phase leads.
* [ ] Solder power leads. Power cables should be 120mm long.



#### Soldering Done Checklist

* [ ] Make sure Encoder pins are securely soldered. Otherwise the position readout will be a constant 3.1415.
* [ ] Make sure VCC / Phase B lead is not touching the thermistor. Otherwise this will feed 24V to STM32.

<figure><img src="../.gitbook/assets/image (3).jpg" alt=""><figcaption></figcaption></figure>



### Programming Checklist



With fresh boards: set optimizations to -O0 when performing Flash option overwrite. The program should be able to boot to main loop (with idle program: blue LED flashes slowly). And then, change optimization back to -O2 and flash again.



* [ ] Uncomment the corresponding MOTORPROFILE macro define.
* [ ] Set CAN ID.
* [ ] Set `FIRST_TIME_BOOTUP` to 1.&#x20;
* [ ] Set `LOAD_*_FROM_FLASH`all to 0. Set `SAFETY_WATCHDOG_ENABLED` to 0.
* [ ] Upload program (may need to update ST Link).
* [ ] Set `FIRST_TIME_BOOTUP` to 0.
* [ ] In the initialization part, enable DAMPING mode. In the main loop, print out encoder and bus voltage.
* [ ] Upload program. Turn on motor power supply. Check serial monitor, examine if encoder reading and bus voltage reading are correct. Turn motor by hand, examine if the motor is in damping mode.
* [ ] Turn off motor power supply.
* [ ] In the initialization part, enable CALIBRATION mode.
* [ ] Upload program and run calibration sequence.
* [ ] Set `LOAD_*_FROM_FLASH`all to 1.
* [ ] (optional) Set mode to CURRENT CONTROL and print out current data. Upload program. Examine current waveform and if motor is running correctly.
* [ ] Set mode to POSITION and uncomment the position setpoint program. Upload program
* [ ] Set `SAFETY_WATCHDOG_ENABLED` to 0.
*





Make sure that the current gain is not too high.
