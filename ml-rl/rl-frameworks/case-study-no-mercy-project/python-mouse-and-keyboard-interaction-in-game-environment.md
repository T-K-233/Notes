# Python Mouse and Keyboard Interaction in Game Environment





## Getting mouse movements

We are interested in getting mouse movement in game.

The following script is executed for reading mouse information

```bash
import time

import win32api

while True:
    # print mouse movement
    movement = win32api.GetCursorPos()
    
    print(movement)
        
    time.sleep(0.5)

```



Differ from typical applications, games will acquire the mouse and force it to be at center of the screen. Therefore, without mouse movements, the reading will always be (960, 540) on a 1080P screen.



The following log shows first moving the mouse down, back to center, moving to right, and then back to center. +X is right, and +Y is down.



<figure><img src="../../../.gitbook/assets/image (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>



## Sending mouse movements

There exist a linear mapping between the mouse movement and the angle of rotation of the in-game viewpoint. The mapping varies based on the mouse sensitivity setting in game, but is independent of the game window resolution.

By default, the mouse sensitivity is set to 15%.

<figure><img src="../../../.gitbook/assets/image (1) (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>

Under this setting, the mapping is as follows:

Mouse sensitivity: 15%

Screen resolution: 1920 x 1080

Game resolution: any

<table><thead><tr><th width="123"></th><th width="133">Angle Range</th><th width="274">Mouse Movement Value Range</th><th>100% Ratio</th></tr></thead><tbody><tr><td>Yaw (X)</td><td>360</td><td>3636.X</td><td>1.515</td></tr><tr><td>Pitch (Y)</td><td>180</td><td>1800</td><td>1.500</td></tr></tbody></table>



Converting from angle to mouse movement value:

`value = angle * (X_OR_Y_RATIO / MOUSE_SENSITIVITY)`





