# Setting up Recoil USB-CAN Adapter - Ubuntu

The device driver for the adapter is already included in the linux driver libraries.&#x20;

The device should appear on the `/dev/ttyACM0` port.&#x20;



## Change port access

By default, the port is only accessible to sudo users. Run the following command to create or edit the config file

```bash
sudoedit /etc/udev/rules.d/50-ttyacm.rules
```



In the opened editor, add the following lines:

```bash
KERNEL=="ttyACM[0-9]*",MODE="0666"
```



After saving, unplug and re-plug the device. Now it should be accessible to all users.



