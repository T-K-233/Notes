# Arty 35T / 100T UART Pins

From the [official constraint file](https://github.com/Digilent/digilent-xdc/blob/master/Arty-A7-35-Master.xdc), the UART interface is labeled `uart_rxd_out` and `uart_txd_in`

![](<../../.gitbook/assets/image (110).png>)

Note that here, Diligent is labeling the rxd / txd pair with respect to the USB adapter IC, and in / out direction with respect to the DUT (Arty FPGA chip itself).&#x20;

Thus, `uart_rxd_out` correspond to DUT's TX, while `uart_txd_in` correspond to DUT's RX pin.



As a quick reference, the `z1top.v` should use the following port definition:

```verilog
module z1top #(
  // ...
) (
  input         CLK100MHZ,
  input         ck_rst,
  
  output        uart_rxd_out,
  input         uart_txd_in,
  // ...
i);
```
