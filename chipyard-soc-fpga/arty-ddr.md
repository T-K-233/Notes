# Arty DDR

![](<../.gitbook/assets/image (9) (1) (1).png>)

![](<../.gitbook/assets/image (30).png>)

![](<../.gitbook/assets/image (14) (1).png>)

![](<../.gitbook/assets/image (29).png>)

![](<../.gitbook/assets/image (125).png>)

![](<../.gitbook/assets/image (57).png>)

![](<../.gitbook/assets/image (38).png>)

![](<../.gitbook/assets/image (101) (1).png>)

![](<../.gitbook/assets/image (112).png>)

![](<../.gitbook/assets/image (9) (1).png>)

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
   System Clock Type               : Single-Ended
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
   Interface                     : AXI
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
   Data Width                    : 32
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

System_Clock: 
	SignalName: sys_clk_i
		PadLocation: D5  Bank: 35

System_Control: 
	SignalName: sys_rst
		PadLocation: No connect  Bank: Select Bank
	SignalName: init_calib_complete
		PadLocation: No connect  Bank: Select Bank
	SignalName: tg_compare_error
		PadLocation: No connect  Bank: Select Bank



    



```







