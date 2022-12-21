# ESP32 Cam

## Version Requirements

Unfortunately, there are some version combinations that cannot work.

The working one is

|                     |             |   |
| ------------------- | ----------- | - |
| Arduino IDE         | 1.8.19      |   |
| esp32 board package | 1.0.6       |   |
| ESP32-CAM Board     | ESP32-S     |   |
| Camera Module       | DC-26 40-V3 |   |

## Add ESP32 Boards to Arduino IDE



Open Arduino, select File -> Preferences.

![](<../.gitbook/assets/image (78).png>)



Add the following URL to the "Additional Boards Manager URLs" section:

```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

![](<../.gitbook/assets/image (46).png>)



Restart Arduino.



Go to Tools -> Board -> Boards Manager...

![](<../.gitbook/assets/image (49).png>)



Search and install the esp32.

![](<../.gitbook/assets/image (24) (1).png>)



After installation, select "".



![](<../.gitbook/assets/image (137).png>)



## Configure Example Code

![](<../.gitbook/assets/image (80).png>)

Change the define to use `CAMERA_MODEL_AI_THINKER`.

![](<../.gitbook/assets/image (52).png>)

## Wiring

![](<../.gitbook/assets/image (1) (1).jpg>)

![](<../.gitbook/assets/image (2) (1).jpg>)

Note: do NOT use the GND pin near U0T pin (the one labeled "GND/R"). That pin does not connect to ground for some of the boards.&#x20;



After plugging in the debugger, select the corresponding COM port. No need to change other settings.

![](<../.gitbook/assets/image (129).png>)

![](<../.gitbook/assets/image (94).png>)



After upload, remove the IO0 jumper wire, open Serial Monitor, and click reset button, located at the back of the board.

