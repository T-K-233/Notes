# Table of contents

* [Home](README.md)

## STM32

* [Getting Started - STM32 Edition](stm32/getting-started-stm32-edition/README.md)
  * [Setting up STM32CubeIDE](stm32/getting-started-stm32-edition/setting-up-stm32cubeide.md)
  * [Going Through A Starter Project](stm32/getting-started-stm32-edition/going-through-a-starter-project.md)
  * [Changing STM32CubeIDE Settings](stm32/getting-started-stm32-edition/changing-stm32cubeide-settings.md)
* [Misc](stm32/misc/README.md)
  * [Using Nucleo STLink to Flash Off-board Chips](stm32/misc/using-nucleo-stlink-to-flash-off-board-chips.md)
  * [Changing STM32 Default Boot Option](stm32/misc/changing-stm32-default-boot-option.md)
  * [Flash Option Byte Recovery](stm32/misc/flash-option-byte-recovery.md)
  * [Some Performance Measurements](stm32/misc/some-performance-measurements.md)
* [System Core](stm32/system-core/README.md)
  * [GPIO](stm32/system-core/gpio.md)
  * [External Interrupt](stm32/system-core/external-interrupt.md)
* [Analog](stm32/analog/README.md)
  * [ADC](stm32/analog/adc.md)
  * [DAC](stm32/analog/dac.md)
* [Timers](stm32/timers/README.md)
  * [RTC](stm32/timers/rtc.md)
  * [TIM](stm32/timers/tim.md)
* [Connectivity](stm32/connectivity/README.md)
  * [UART](stm32/connectivity/uart.md)
  * [USART](stm32/connectivity/usart.md)
  * [USART - CAN Dongle (Fixed Size Serializer with Robust Timeout Handling)](stm32/connectivity/usart-can-dongle-fixed-size-serializer-with-robust-timeout-handling.md)
  * [CAN](stm32/connectivity/can.md)
  * [I2C](stm32/connectivity/i2c.md)
  * [SPI](stm32/connectivity/spi.md)
  * [SPI - SD Card](stm32/ntc-temperature-sense-resistor-value-calculation/spi-sd-card.md)
  * [Ethernet](stm32/connectivity/ethernet.md)
  * [USB - FS](stm32/ntc-temperature-sense-resistor-value-calculation/usb-fs.md)
  * [USB - HS](stm32/ntc-temperature-sense-resistor-value-calculation/usb-hs.md)
* [Middleware](stm32/middleware/README.md)
  * [FreeRTOS](stm32/middleware/freertos.md)
* [Software Pack](stm32/software-pack/README.md)
  * [STMicroelectronics.X-CUBE-AI - Sine Approximator](stm32/software-pack/stmicroelectronics.x-cube-ai-sine-approximator.md)

## Chipyard SoC / FPGA

* [Quick Navigation Page](chipyard-soc-fpga/quick-navigation-page.md)
* [Chipyard Environment Setup](chipyard-soc-fpga/chipyard-environment-setup/README.md)
  * [Installing Chipyard - Ubuntu](chipyard-soc-fpga/installing-chipyard.md)
  * [Installing Chipyard - Windows Subsystem Linux](chipyard-soc-fpga/chipyard-environment-setup/installing-chipyard-windows-subsystem-linux.md)
* [Xilinx Vivado Install on Ubuntu 22.04](chipyard-soc-fpga/xilinx-vivado-install-on-ubuntu-22.04.md)
* [Setup RISC-V Development Environment on Windows](chipyard-soc-fpga/setup-risc-v-development-environment-on-windows.md)
* [Arty 35T / 100T UART Pins](chipyard-soc-fpga/arty-35t-100t-uart-pins.md)
* [Arty 35T Using DDR DRAM](chipyard-soc-fpga/arty-35t-using-ddr-dram.md)
* [Vivado Generate Flash Config .mcs File From Bitstream](chipyard-soc-fpga/vivado-generate-flash-config-.mcs-file-from-bitstream.md)
* [Debugging](chipyard-soc-fpga/debugging/README.md)
  * [GDB debugging OsciArty](chipyard-soc-fpga/debugging/gdb-debugging-osciarty.md)
  * [Booting BearlyML With External SPI Flash](chipyard-soc-fpga/debugging/booting-bearlyml-with-external-spi-flash.md)
* [Prototyping the SoC on FPGA](chipyard-soc-fpga/prototyping-the-soc-on-fpga.md)
* [Build EagleX Linux Image](chipyard-soc-fpga/build-eaglex-linux-image.md)
* [Setting Up SD / microSD Card for vcu118 Linux Image](chipyard-soc-fpga/setting-up-sd-microsd-card-for-vcu118-linux-image.md)
* [FT LINK JTAG Debugger Driver Setup](chipyard-soc-fpga/ft-link-jtag-debugger-driver-setup.md)
* [RISC-V: Baremetal From The Ground Up (Chipyard Edition)](chipyard-soc-fpga/risc-v-baremetal-from-the-ground-up-chipyard-edition.md)

## Motor Control

* [Recoil FOC Motor Controller](motor-control/recoil-foc-motor-controller/README.md)
  * [0x00. Theory of Operation](motor-control/recoil-foc-motor-controller/0x00.-theory-of-operation.md)
  * [0x01. Components](motor-control/recoil-foc-motor-controller/0x01.-components.md)
  * [0x02. Implementation](motor-control/recoil-foc-motor-controller/0x02.-implementation.md)
  * [0x03. Application](motor-control/recoil-foc-motor-controller/0x03.-application.md)
* [Recoil Documentation](motor-control/all-digital-phase-locked-loop-filter.md)
* [New Controller Board Soldering & Power-on Checklist](motor-control/new-controller-board-soldering-and-power-on-checklist.md)
* [MJBOTS Moteus setup](motor-control/mjbots-moteus-setup.md)
* [Failed Attempt on Acceleration- and Velocity-Limited Trajectory Generation](motor-control/failed-attempt-on-acceleration-and-velocity-limited-trajectory-generation.md)
* [Moteus Code Analyze](motor-control/moteus-code-analyze.md)
* [MIT Motor Controller Code Analyze](motor-control/mit-motor-controller-code-analyze.md)
* [ODrive Setup](motor-control/odrive-setup.md)
* [NTC Temperature Sense Resistor Value Calculation](motor-control/ntc-temperature-sense-resistor-value-calculation.md)

## Tools

* [Ubuntu 22.04 Standard Installation Procedure](tools/page-6.md)
* [Windows Ubuntu Dual Boot Issues](tools/windows-ubuntu-dual-boot-issues.md)
* [Linux Mounting SD Card](tools/linux-mount-sd-card.md)
* [Install and Configure GlobalProtect UC Berkeley VPN Service on Ubuntu 22.04](tools/install-and-configure-globalprotect-uc-berkeley-vpn-service-on-ubuntu-22.04.md)
* [Using JADENS Thermal Label Printer](tools/using-jadens-thermal-label-printer.md)
* [Connecting the SIGLENT SDS1104X-U Oscilloscope to Computer](tools/connecting-the-siglent-sds1104x-u-oscilloscope-to-computer.md)
* [Using Oscilloscope: x1 vs x10](tools/using-oscilloscope-x1-vs-x10.md)
* [Microsoft Visual Studio Create Software Signature](tools/microsoft-visual-studio-create-software-signature.md)
* [Python Logging Utility](tools/python-logging-utility.md)

## ML/AI

* [OpenAI gym + Mujoco Setup](setup/setting-up-aws-mujoco-training-machine.md)
* [NVIDIA Omniverse Isaac Sim Setup](ml-ai/nvidia-omniverse-isaac-sim-setup.md)
* [ROS 2 Humble Hawksbill Setup on Ubuntu](ml-ai/ros-2-humble-hawksbill-setup-on-ubuntu.md)
* [ROS 2 Humble Hawksbill Setup on Windows 10](ml-ai/ros-2-humble-hawksbill-setup-on-windows-10.md)
* [Gazebo Setup](ml-ai/gazebo-setup.md)

## Setup

* [Github-Related Info](setup/github-related-info.md)
* [Raspberry Pi Setup](setup/raspberry-pi-setup.md)
* [Clang-Format Style Config](setup/clang-format-style-config.md)
* [Getting Started with XBee (ZigBee)](setup/getting-started-with-xbee-zigbee.md)

## Mechanical

* [MAD Cycloidal Actuator](mechanical/mad-cycloidal-actuator.md)
* [Fixing the Unitree A1 Robot Dog](mechanical/fixing-the-unitree-a1-robot-dog.md)

## Electrical

* [A Note on the Polarity of the Famous TT Motor](electrical/a-note-on-the-polarity-of-the-famous-tt-motor.md)
* [Wiring Convention](electrical/wiring-convention.md)
* [MCU Pinmap Convention](electrical/mcu-pinmap-convention.md)
* [PCB Design and Manufacturing Conventions](electrical/pcb-design-and-manufacturing-conventions.md)
* [ESP32 Cam](electrical/esp32-cam.md)
* [LiPo Safety](electrical/lipo-safety.md)

## Network

* [Digital-Twin Communication System](network/digital-twin-communication-system.md)
* [NewLine Serialization Method](network/newline-serialization-method.md)
* [Home Network Setup](network/home-network-setup.md)

## 3D Modeling

* [Blender Python Related](3d-modeling/blender-python-related.md)

***

* [Page 1](page-1.md)

## Maintainance Log

* [RISC-V Toolbox Website](maintainance-log/risc-v-toolbox-website.md)

## UW

* [Page 2](uw/page-2.md)
* [Remote Controlled Humanoid Robot Whitepaper And Design Notes](uw/remote-controlled-humanoid-robot-whitepaper-and-design-notes/README.md)
  * [Note on Face Design and Manufacture](uw/remote-controlled-humanoid-robot-whitepaper-and-design-notes/note-on-face-design-and-manufacture.md)

## Finance

* [Finance](finance/tax-stuff.md)
* [Page 3](finance/page-3.md)

## Life

* [Some Interview Questions](life/some-interview-questions.md)
* [Health Insurance](life/health-insurance.md)

## Group 1

* [Page 5](group-1/page-5.md)
