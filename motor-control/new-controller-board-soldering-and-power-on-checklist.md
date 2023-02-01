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



* [ ] Change current loop PID gain to match motor spec.
* [ ] Set CAN ID.
* [ ] Set `#define INITIAL_PROG 1`.
* [ ] Flash the program.
* [ ] Power-cycle the board.
* [ ] Clear`#define INITIAL_PROG 0`.
* [ ] Enable calibration sequence. Enable position demo if wanted.
* [ ] Flash the program.



Make sure that the current gain is not too high.
