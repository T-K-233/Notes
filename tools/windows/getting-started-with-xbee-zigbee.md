# Getting Started with XBee (ZigBee)

I’m using the XBee S2C module from Amazon:

<figure><img src="../../.gitbook/assets/image (22) (1) (3).png" alt=""><figcaption></figcaption></figure>

## 1. Wiring <a href="#efa0" id="efa0"></a>

First, we need to wire up the XBee. Originally, I was going to use an Arduino UNO to do the USB-TTL conversion. However, I had no luck with it. Not sure about the reason. Maybe because I’m using a non-official clone, which has a different onboard USB-TTL chip?

Then I realized that I can just use a plain USB-TTL converter to establish the link between the XBee module with the computer. I’m using the Rath RA-LINK module, originally designed for the GD32VF series MCU. We will just use the serial converter function of this debugger.

<figure><img src="../../.gitbook/assets/image (15) (3) (1).png" alt=""><figcaption></figcaption></figure>

An additional benefit of this setup is that both end are using the 2.0mm pitch Dupont connector. (Viva la metric unit!)

Here is the wiring diagram:

<figure><img src="../../.gitbook/assets/image (24) (2) (1).png" alt=""><figcaption></figcaption></figure>

## 2. Download XCTU

Download XCTU from [here](https://hub.digi.com/support/products/xctu/?path=/support/asset/xctu-v-659-windows-x86x64/).

<figure><img src="../../.gitbook/assets/image (23) (2).png" alt=""><figcaption></figcaption></figure>

Insert the USB debugger into the computer and click this button.

<img src="../../.gitbook/assets/image (26) (2).png" alt="" data-size="line">



In the pop-up menu, we will select the corresponding COM port.

<figure><img src="../../.gitbook/assets/image (21) (2).png" alt=""><figcaption></figcaption></figure>

It will automatically search for the device.

<figure><img src="../../.gitbook/assets/image (17) (2).png" alt=""><figcaption></figcaption></figure>

And then display its settings

<figure><img src="../../.gitbook/assets/image (14) (2).png" alt=""><figcaption></figcaption></figure>

We will make this first device the station (coordinator), and thus we modify the following fields:

**ID** (Pan ID): 2333

**CE** (Coordinator Enable): Enabled \[1]

**DL** (Destination Address Low): FFFF

**NI** (Node Identifier): XBee-Station

**BD** (Baud Rate): 115200 \[7]

And update the configuration by clicking this button

<img src="../../.gitbook/assets/image (19) (1).png" alt="" data-size="line">



Then the station node is configured successfully.

Next, we perform the same thing for the car node, but this time with the following settings:

**ID** (Pan ID): 2333

**JV** (Channel Verification): Enabled \[1]

**DL** (Destination Address Low): 0

**NI** (Node Identifier): XBee-Car

**BD** (Baud Rate): 115200 \[7]

Now we are ready to test the connections.

Press this button to switch to the terminal window

<img src="../../.gitbook/assets/image (27) (1).png" alt="" data-size="line">



And click this button to open the serial port

<img src="../../.gitbook/assets/image (28).png" alt="" data-size="line">



Now when we type in one terminal, we can see the contents appear in another terminal, indicating that the connection is established

<figure><img src="../../.gitbook/assets/image (18) (2).png" alt=""><figcaption></figcaption></figure>



P.S. if the wrong configuration is entered in XBee parameters, XCTU will try to perform a revert on the chip

<figure><img src="../../.gitbook/assets/image (25) (2).png" alt=""><figcaption></figcaption></figure>



## 3. STM32 Code <a href="#f1b8" id="f1b8"></a>

Because XBee uses simple TX and RX to transmit data, we can simply program the STM32 to transmit and listen to data on the [USART port](../../stm32/connectivity/uart.md).

\


