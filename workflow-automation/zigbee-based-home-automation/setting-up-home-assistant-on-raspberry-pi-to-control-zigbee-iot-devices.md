# Setting up Home Assistant on Raspberry Pi to Control Zigbee IoT Devices

In this article, we will introduce how to set up Home Assistant OS on a Raspberry Pi 3 B+ and configure Zigbee2MQTT to connect and control the Zigbee based IoT devices. The entire process takes around 1 hour to finish.

***



This is fairly simple and straight forward, thanks to the nice support provided by the Raspberry Pi Imager.



## Hardware Platform

An pretty old Raspberry Pi 3 B+ is used.&#x20;

Some people reported that the Zigbee dongle does not work well with USB3.0 ports due to interference, so maybe using 3 B+ with all USB 2.0 ports is a good option?



## Setting up Pi

Follow this tutorial on flashing the firmware

{% embed url="https://www.home-assistant.io/installation/raspberrypi" %}

After opening [homeassistant.local:8123](http://homeassistant.local:8123/), it takes some time to set everything up

<figure><img src="../../.gitbook/assets/image (231).png" alt=""><figcaption></figcaption></figure>



## Setting up Home Assistant

Follow the prompt on the webpage to set up admin account, home location, and data sharing settings.



## Setting up MQTT Integration

In Settings -> integrations, add MQTT integration

<figure><img src="../../.gitbook/assets/image (5).png" alt=""><figcaption></figcaption></figure>

Select MQTT

<figure><img src="../../.gitbook/assets/image (1) (1).png" alt=""><figcaption></figcaption></figure>

We will use the provided mosquitto broker to connect the MQTT to Home Assistant service.

<figure><img src="../../.gitbook/assets/image (2) (1).png" alt=""><figcaption></figcaption></figure>



It takes a while to install the integration service. After installation, it will automatically start the service.

<figure><img src="../../.gitbook/assets/image (3) (1).png" alt=""><figcaption></figcaption></figure>





## Setting up Zigbee2MQTT

Install the addon following this tutorial

{% embed url="https://github.com/zigbee2mqtt/hassio-zigbee2mqtt#installation" %}

Click Add

<figure><img src="../../.gitbook/assets/image (4) (1).png" alt=""><figcaption></figcaption></figure>



After adding, refresh the page, and it should appear in the Add-on Store list under section "Home Assistant Add-on: Zigbee2MQTT"

Click into the panel, and click Install.

After installation, start the service.



In this case, it fails to find the Zigbee dongle.

<figure><img src="../../.gitbook/assets/image (5) (1).png" alt=""><figcaption></figcaption></figure>

Then we need to manually tell it.



The dongle path can be seen at Settings -> System -> Hardware -> All Hardware

<figure><img src="../../.gitbook/assets/image (6).png" alt=""><figcaption></figcaption></figure>



Put this information in the configuration, save and restart the addon.

```yaml
port: /dev/serial/by-id/usb-ITead_Sonoff_Zigbee_3.0_USB_Dongle_Plus_aec3a1c88f19ec1197ff37cc47486eb0-if00-port0
adapter: zstack
```

<figure><img src="../../.gitbook/assets/image (7).png" alt=""><figcaption></figcaption></figure>



Now it should be started correctly

<figure><img src="../../.gitbook/assets/image (8).png" alt=""><figcaption></figcaption></figure>



## Adding Zigbee Devices

In the Addon panel, click the "Open WebUI" button.

Click the "Permit Join (All)" to allow pairing with devices.

<figure><img src="../../.gitbook/assets/image (9).png" alt=""><figcaption></figcaption></figure>



Now enable pairing mode on the IoT devices, it should then be paired to the service fairly quickly.

I observed that the pairing time is much faster than when using ZHA.





## Configuring Apple HomeKit

{% embed url="https://www.home-assistant.io/integrations/homekit/" %}







