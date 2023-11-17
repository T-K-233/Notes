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

FT header

|            |             |             |
| ---------- | ----------- | ----------- |
| Square Pad | 1 - 1 TDO   | 7 - 5 TDI   |
|            | 2 - 2 nTRST | 8 - 6 TMS   |
|            | 3 - 3 TCK   | 9 - 7 nSRST |
|            | 4 - 4 TXD   | 10 - 8 RXD  |
| GND Row    | 5 - GND     | 11 - GND    |
| VCC Row    | 6 - VCC     | 12 - VCC    |



TileLink header

|            |                 |                   |
| ---------- | --------------- | ----------------- |
| Square Pad | 1 - TL\_CLK     | 7 - CLK\_EXT      |
|            | 2 - TL\_IN\_VAL | 8 - TL\_OUT\_VAL  |
|            | 3 - TL\_IN\_RDY | 9 - TL\_OUT\_RDY  |
|            | 4 - TL\_IN\_DAT | 10 - TL\_OUT\_DAT |
| GND Row    | 5 - GND         | 11 - GND          |
| VCC Row    | 6 - VCC         | 12 - VCC          |

