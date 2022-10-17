# ODrive Setup





![](<../.gitbook/assets/image (37).png>)

{% embed url="https://discourse.odriverobotics.com/t/odrivetool-could-not-open-usb-device/7230/13" %}

Just change the Native Interface to libusb-win32 driver solves the problem.

![](<../.gitbook/assets/image (10).png>)

![](<../.gitbook/assets/image (48).png>)

![](<../.gitbook/assets/image (128) (1).png>)



{% embed url="https://www.aliexpress.com/item/2251832533667878.html?gatewayAdapt=4itemAdapt" %}

```python
odrv0.axis0.motor.config.pole_pairs = 7

odrv0.axis0.motor.config.torque_constant = 0.024323529411764706

odrv0.axis0.encoder.config.cpr = 2400
```
