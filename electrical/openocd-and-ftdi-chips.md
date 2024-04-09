# OpenOCD and FTDI Chips



To let OpenOCD correctly drive the FTDI chip, we need to tell OpenOCD the hardware configuration of the FTDI chip through the config file:

```properties
### FTDI interface configuration ###
ftdi_vid_pid                0x0403 0x6010
ftdi_channel                0
ftdi_layout_init            0x0888 0x8B8B

```



Reference: [https://openocd.org/doc/html/Debug-Adapter-Configuration.html](https://openocd.org/doc/html/Debug-Adapter-Configuration.html)





We will analyze the config file line by line





```properties
ftdi_vid_pid                0x0403 0x6010
```

This line tells OpenOCD to find the USB device with the signature of corresponding vendor ID (VID) and product ID (PID).

0x0403: [Future Technology Devices International Limited](https://the-sz.com/products/usbid/index.php?v=0x0403)

0x6010: FT2232C/D/H Dual UART/FIFO IC



```properties
ftdi_channel                0
```

Selects the channel of the FTDI device to use for MPSSE operations. Here we select channel A, which is the first channel (channel 0).



```properties
ftdi_layout_init            0x0800 0x0800
```

Defines the initial data and IO direction of the GPIOs. The mapping is as follows:

| 0x | bit \[15..8] | bit \[7..0]  |
| -- | ------------ | ------------ |
|    | ACBUS\[7..0] | ADBUS\[7..0] |

As an simple example, to initialize the JTAG LED on FT-LINK, which is an open-drain LED connected to ACBUS3 pin, we use the following setting

```properties
ftdi_layout_init            0x0800 0x0800   # this initialize LED off
```

```properties
ftdi_layout_init            0x0000 0x0800   # this initialize LED on
```



We can also define the JTAG\_LED variable with the following command

```properties
ftdi_layout_signal JTAG_LED -oe 0x0800
```



Then, in the init routine, we can use the variable to reference this pin and blink the LED

```properties
init

ftdi_set_signal JTAG_LED 0    # LED on

sleep 1000

ftdi_set_signal JTAG_LED z    # LED off

sleep 1000

ftdi_set_signal JTAG_LED 0    # LED on
```



note that since we are only enabling the output driver by the `-oe` flag, the chip can only drive it open-drain, and therefore we are only allow to set the signal state to either low (`0`) or high-Z (`z`).

If we try to set it to high (`1`), we will get the following error:

```bash
Error: interface can't drive 'JTAG_LED' high
```



If we have some normal signal that we want to drive both high and low, we can use the `-data` flag instead:

```properties
ftdi_layout_signal JTAG_LED -data 0x0800
```



```properties
init

ftdi_set_signal JTAG_LED 0    # LED on

sleep 1000

ftdi_set_signal JTAG_LED 1    # LED off

sleep 1000

ftdi_set_signal JTAG_LED 0    # LED on
```



If we somehow want all the states, we do the following

```properties
ftdi_layout_signal JTAG_LED -data 0x0800 -oe 0x0800
```















