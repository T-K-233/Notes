# Using 3DConnexion Mouse on Ubuntu with Python

```bash
sudo apt install libhidapi-dev
```

```bash
pip install pyspacemouse
```



Create UDev rule for hidraw:

```bash
sudo nano /etc/udev/rules.d/50-hidraw.rules
```

{% code title="50-hidraw.rules" %}
```bash
SUBSYSTEM=="hidraw", KERNEL=="hidraw*", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c626", GROUP:="users", MODE:="0666"
```
{% endcode %}

And then

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```





```bash
sudo usermod -aG plugdev $USER
newgrp plugdev
```



