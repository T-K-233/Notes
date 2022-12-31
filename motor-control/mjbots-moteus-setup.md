# MJBOTS Moteus setup

### Install Python Package

```bash
pip3 install moteus_gui
```

### Calibration

With motor controller powered and CAN connected to USB adapter, run&#x20;

```bash
py -m moteus.moteus_tool --target 1 --calibrate
```

![](<../.gitbook/assets/image (120).png>)

![](<../.gitbook/assets/image (15) (1).png>)



### Position Mode

run&#x20;

```bash
py -m moteus_gui.tview --devices=1
```



Set `servopos` limits to appropriate values

![](<../.gitbook/assets/image (86).png>)

And set&#x20;

![](<../.gitbook/assets/image (59).png>)

```bash
conf set servopos.position_min -3
conf set servopos.position_max 0.5
conf set servo.max_current_A 5
```



```bash
conf set servo.default_accel_limit 1
conf set servo.max_velocity 5
```



To save config to EEPROM:

```bash
conf write
```

and to read from:

```bash
conf load
```



```bash
d pos 0 0 0.1
```



```bash
d stop
```



