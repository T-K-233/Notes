# Lab Automation

Most of the lab equipments support Ethernet connection, and it is possible to programmatically control the equipments with [Standard Commands for Programmable Instruments](https://en.wikipedia.org/wiki/Standard\_Commands\_for\_Programmable\_Instruments) (SCPI) protocol.



Here's a Python wrapper for the SCPI protocol. Various instruments will be added to this library.

{% embed url="https://github.com/uncertainty-cc/SCPI-Python" %}

> _Note: Most of the instruments use TCP socket connection on port 5025, but this is not specified by the spec, and may differ._



### SIGLENT SPD3303X-E

port: 5025

[https://siglentna.com/wp-content/uploads/dlm\_uploads/2022/11/SPD3303X\_QuickStart\_E02A.pdf](https://siglentna.com/wp-content/uploads/dlm\_uploads/2022/11/SPD3303X\_QuickStart\_E02A.pdf)

#### Example

```python
import time

from cc.scpi import SiglentSPD3303X

device = SiglentSPD3303X("128.32.62.100")
device.connect()

print(device.getInstrumentIdentification())

device.setVoltage(0.1, device.Channel.CH1)
device.enableOutput(device.Channel.CH1)

time.sleep(1)

print(device.getVoltage(device.Channel.CH1))
print(device.getCurrent(device.Channel.CH1))
print(device.getPower(device.Channel.CH1))


device.disableOutput(device.Channel.CH1)
```



### Keysight 81134A

port: 5025

[https://www.keysight.com/us/en/assets/9018-02477/programming-guides/9018-02477.pdf](https://www.keysight.com/us/en/assets/9018-02477/programming-guides/9018-02477.pdf)

#### Example

```python
import time

from cc.scpi import Keysight81134A

device = Keysight81134A("128.32.62.102")
device.connect()

print(device.getInstrumentIdentification())


device.enableOutput(Keysight81134A.Channel.CH1)
device.disableOutput(Keysight81134A.Channel.CH1)
```





EagleX Bench Initialize Script

```python
import time
```



