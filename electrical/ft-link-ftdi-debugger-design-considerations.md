# FT-LINK FTDI Debugger Design Considerations

Signal Mapping for FT2232H

<table><thead><tr><th></th><th>Pin #</th><th width="127"></th><th></th></tr></thead><tbody><tr><td><strong>ADBUS0</strong></td><td>16</td><td><strong>TCK</strong></td><td></td></tr><tr><td><strong>ADBUS1</strong></td><td>17</td><td><strong>TDI</strong></td><td></td></tr><tr><td><strong>ADBUS2</strong></td><td>18</td><td><strong>TDO</strong></td><td></td></tr><tr><td><strong>ADBUS3</strong></td><td>19</td><td><strong>TMS</strong></td><td></td></tr><tr><td>ADBUS4</td><td>21</td><td>JTAG Buffer Control</td><td></td></tr><tr><td>ADBUS5</td><td>22</td><td>Target present</td><td></td></tr><tr><td>ADBUS6</td><td>23</td><td>TSRST in</td><td></td></tr><tr><td>ADBUS7</td><td>24</td><td>RTCK</td><td></td></tr><tr><td></td><td></td><td></td><td></td></tr><tr><td><strong>ACBUS0</strong></td><td>26</td><td><strong>TRST</strong></td><td></td></tr><tr><td><strong>ACBUS1</strong></td><td>27</td><td><strong>TSRST</strong></td><td></td></tr><tr><td>ACBUS2</td><td>28</td><td>TRST buffer enable</td><td></td></tr><tr><td>ACBUS3</td><td>29</td><td>JTAG LED</td><td></td></tr><tr><td></td><td></td><td></td><td></td></tr><tr><td><strong>BDBUS0</strong></td><td>38</td><td><strong>TXD</strong></td><td>connect to device RXD</td></tr><tr><td><strong>BDBUS1</strong></td><td>39</td><td><strong>RXD</strong></td><td>connect to device TXD</td></tr><tr><td>BDBUS2</td><td>40</td><td>RTS</td><td></td></tr><tr><td>BDBUS3</td><td>41</td><td>CTS</td><td></td></tr><tr><td>BDBUS4</td><td>43</td><td>(CK_RST)</td><td></td></tr><tr><td></td><td></td><td></td><td></td></tr><tr><td>BCBUS3</td><td>54</td><td>RXLED</td><td>Active low</td></tr><tr><td>BCBUS4</td><td>55</td><td>TXLED</td><td>Active low</td></tr></tbody></table>









<figure><img src="../.gitbook/assets/image (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>



<figure><img src="../.gitbook/assets/image (1) (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>



<figure><img src="../.gitbook/assets/image (2) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>





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

