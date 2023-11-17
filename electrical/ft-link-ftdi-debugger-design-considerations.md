# FT-LINK FTDI Debugger Design Considerations

Signal Mapping

|            |                     |                       |
| ---------- | ------------------- | --------------------- |
| **ADBUS0** | **TCK**             |                       |
| **ADBUS1** | **TDI**             |                       |
| **ADBUS2** | **TDO**             |                       |
| **ADBUS3** | **TMS**             |                       |
| ADBUS4     | JTAG Buffer Control |                       |
| ADBUS5     | Target present      |                       |
| ADBUS6     | TSRST in            |                       |
| ADBUS7     | RTCK                |                       |
|            |                     |                       |
| **ACBUS0** | **TRST**            |                       |
| **ACBUS1** | **TSRST**           |                       |
| ACBUS2     | TRST buffer enable  |                       |
| ACBUS3     | JTAG LED            |                       |
|            |                     |                       |
| **BDBUS0** | **TXD**             | connect to device RXD |
| **BDBUS1** | **RXD**             | connect to device TXD |
| BDBUS2     |                     |                       |
| BDBUS3     |                     |                       |
| BDBUS4     | CK\_RST             |                       |
|            |                     |                       |
| BCBUS2     | RXLED               |                       |
| BCBUS3     | TXLED               |                       |









<figure><img src="../.gitbook/assets/image.png" alt=""><figcaption></figcaption></figure>



<figure><img src="../.gitbook/assets/image (1).png" alt=""><figcaption></figcaption></figure>



<figure><img src="../.gitbook/assets/image (2).png" alt=""><figcaption></figcaption></figure>





### PMOD Header

|            |             |             |
| ---------- | ----------- | ----------- |
| Square Pad | 1 - 1 TDO   | 7 - 5 TDI   |
|            | 2 - 2 nTRST | 8 - 6 TMS   |
|            | 3 - 3 TCK   | 9 - 7 nSRST |
|            | 4 - 4 TXD   | 10 - 8 RXD  |
| GND row    | 5 - GND     | 11 - GND    |
| VCC row    | 6 - VCC     | 12 - VCC    |

