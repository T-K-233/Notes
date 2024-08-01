# Configuring Vivado DDR MIG on Arty 35T



The configuration parameters are taken from the Arty 100T Reference Manual

<figure><img src="../.gitbook/assets/image (221).png" alt=""><figcaption></figcaption></figure>



In the MIG configuration tab, choose "Create Design". Check "AXI4 Interface" if the target design requires AXI4. Otherwise, a FIFO-like native interface will be used.&#x20;

![](<../.gitbook/assets/image (9) (1) (1).png>)

![](<../.gitbook/assets/image (30) (1).png>)

![](<../.gitbook/assets/image (14) (1) (1).png>)

For the Memory Part, "MT41K128M16XX-15E" should be chosen, according to [this link](https://forum.digilent.com/topic/2709-which-memory-chip-on-the-arty/).

![](<../.gitbook/assets/image (29) (1).png>)

![](<../.gitbook/assets/image (125).png>)

![](<../.gitbook/assets/image (57).png>)



<figure><img src="../.gitbook/assets/image (222).png" alt=""><figcaption></figcaption></figure>





![](<../.gitbook/assets/image (38).png>)

![](<../.gitbook/assets/image (101) (1).png>)

![](<../.gitbook/assets/image (112).png>)



Download the pin configuration file from [here](https://github.com/Digilent/Arty/tree/master/Resources/Arty\_MIG\_DDR3?\_ga=2.161862332.183027093.1669435108-1257577490.1663646590).

In this page, first click the "Read XDC/UCF" button and load the "Arty\_C\_mig.ucf" file.&#x20;

Then, click "Validate". The window should show that the pinout is valid.

The Next button should then be enabled.

![](<../.gitbook/assets/image (9) (1) (2).png>)



<figure><img src="../.gitbook/assets/image (223).png" alt=""><figcaption></figcaption></figure>



![](<../.gitbook/assets/image (106).png>)

Configuration Parameters:

```bash


Vivado Project Options:
   Target Device                   : xc7a35ti-csg324
   Speed Grade                     : -1L
   HDL                             : verilog
   Synthesis Tool                  : VIVADO

If any of the above options are incorrect,   please click on "Cancel", change the CORE Generator Project Options, and restart MIG.

MIG Output Options:
   Module Name                     : mig_7series_0
   No of Controllers               : 1
   Selected Compatible Device(s)   : --

FPGA Options:
   System Clock Type               : No Buffer
   Reference Clock Type            : No Buffer
   Debug Port                      : OFF
   Internal Vref                   : enabled
   IO Power Reduction              : ON
   XADC instantiation in MIG       : Enabled

Extended FPGA Options:
   DCI for DQ,DQS/DQS#,DM          : enabled
   Internal Termination (HR Banks) : 50 Ohms
    



/*******************************************************/
/*                  Controller 0                       */
/*******************************************************/
Controller Options :
   Memory                        : DDR3_SDRAM
   Interface                     : NATIVE
   Design Clock Frequency        : 3000 ps (333.33 MHz)
   Phy to Controller Clock Ratio : 4:1
   Input Clock Period            : 5999 ps
   CLKFBOUT_MULT (PLL)           : 8
   DIVCLK_DIVIDE (PLL)           : 1
   VCC_AUX IO                    : 1.8V
   Memory Type                   : Components
   Memory Part                   : MT41K128M16XX-15E
   Equivalent Part(s)            : --
   Data Width                    : 16
   ECC                           : Disabled
   Data Mask                     : enabled
   ORDERING                      : Strict

AXI Parameters :
   Data Width                    : 128
   Arbitration Scheme            : RD_PRI_REG
   Narrow Burst Support          : 0
   ID Width                      : 4

Memory Options:
   Burst Length (MR0[1:0])          : 8 - Fixed
   Read Burst Type (MR0[3])         : Sequential
   CAS Latency (MR0[6:4])           : 5
   Output Drive Strength (MR1[5,1]) : RZQ/6
   Controller CS option             : Enable
   Rtt_NOM - ODT (MR1[9,6,2])       : RZQ/6
   Rtt_WR - Dynamic ODT (MR2[10:9]) : Dynamic ODT off
   Memory Address Mapping           : BANK_ROW_COLUMN


Bank Selections:
	Bank: 34
		Byte Group T0:	DQ[0-7]
		Byte Group T1:	DQ[8-15]
		Byte Group T2:	Address/Ctrl-0
		Byte Group T3:	Address/Ctrl-1

System_Control: 
	SignalName: sys_rst
		PadLocation: No connect  Bank: Select Bank
	SignalName: init_calib_complete
		PadLocation: No connect  Bank: Select Bank
	SignalName: tg_compare_error
		PadLocation: No connect  Bank: Select Bank


```







