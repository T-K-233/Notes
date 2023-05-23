# Vivado Generate Flash Config .mcs File From Bitstream

## 1.

Connect the FPGA device, open vivado

## 2.

In hardware manager, right-click the connected device ("xc7a35t" in this case), select "Add Configuration Memory Device..."

<figure><img src="../../.gitbook/assets/image (3) (1) (3).png" alt=""><figcaption></figcaption></figure>

## 3.

Select the correct Flash device mounted on the FPGA board.

For Arty board bought from Amazon recently, the Flash chip should be "s25fl128xxxxxx0"

<figure><img src="../../.gitbook/assets/image (2) (5) (1).png" alt=""><figcaption></figcaption></figure>

The exact chip can be found using this method from Arty's [User Manual](https://digilent.com/reference/programmable-logic/arty-a7/reference-manual?redirect=1#quad-spi\_flash).

<figure><img src="../../.gitbook/assets/image (1) (2) (1).png" alt=""><figcaption></figcaption></figure>

Click OK.&#x20;

If it asks if you want to program the device now, click No.

## 4.

Go to top tool bar, select Tools-> Generate Memory Configuration File...

<figure><img src="../../.gitbook/assets/image (4) (4) (1).png" alt=""><figcaption></figcaption></figure>



In the pop-up window, put the desired output filename in the "Filename" section.

For Chipyard generated bitstreams, we need to select "SPIx4" as the interface width.

Then, specify the bitstream we want to load to the Flash and click "Ok".

<figure><img src="../../.gitbook/assets/image (11) (2) (1).png" alt=""><figcaption></figcaption></figure>



## 5.

Go back to Hardware Manager, right-click the memory device, and then select "Program Configuration Memory Device..."

<figure><img src="../../.gitbook/assets/image (10) (2) (1).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (13) (2) (1).png" alt=""><figcaption></figcaption></figure>

The flashing progress will take about 1 minute to complete.



After flashing, the FPGA will automatically load the bitstream stored in the Flash on power-up, or every time the PROG button is pressed. The DONE LED will show if the bitstream has finished loading.
