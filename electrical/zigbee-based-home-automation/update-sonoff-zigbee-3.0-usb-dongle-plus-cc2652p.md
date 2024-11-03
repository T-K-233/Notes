# Update Sonoff Zigbee 3.0 USB Dongle Plus (CC2652P)



Follow this video

[https://www.youtube.com/watch?v=KBAGWBWBATg\&t=233s](https://www.youtube.com/watch?v=KBAGWBWBATg\&t=233s)





Encountered this error&#x20;

```bash
>Reading file: D:/Downloads/CC1352P2_CC2652P_launchpad_coordinator_20240710.hex.
>Unknown record type: 3.
```



<figure><img src="../../.gitbook/assets/image (229).png" alt=""><figcaption></figcaption></figure>

Solution here

{% embed url="https://github.com/Koenkk/Z-Stack-firmware/issues/397#issuecomment-1455614843" %}

Use this command to convert the file to binary

{% code overflow="wrap" %}
```bash
.\srec_cat.exe "CC1352P2_CC2652P_launchpad_coordinator_20240710.hex" -intel -o CC1352P2_CC2652P_launchpad_coordinator_20240710.bin -binary
```
{% endcode %}

here's the converted file

{% file src="../../.gitbook/assets/CC1352P2_CC2652P_launchpad_coordinator_20240710.bin" %}



Then program again with the binary file. Leave the start address at 0x00000000



<figure><img src="../../.gitbook/assets/image (230).png" alt=""><figcaption></figcaption></figure>







