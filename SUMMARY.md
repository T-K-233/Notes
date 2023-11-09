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
  * [STM32 Flash Option Byte Recovery](stm32/misc/stm32-flash-option-byte-recovery.md)
  * [STM32 Systick and Timeout in Interrupt Routines](stm32/misc/stm32-systick-and-timeout-in-interrupt-routines.md)
  * [Telesky ST-Link V2 Upgrade Firmware](stm32/misc/telesky-st-link-v2-upgrade-firmware.md)
  * [Some Performance Measurements on STM32 MCUs](stm32/misc/some-performance-measurements-on-stm32-mcus.md)
* [System Core](stm32/system-core/README.md)
  * [GPIO](stm32/system-core/gpio.md)
  * [External Interrupt](stm32/system-core/external-interrupt.md)
* [Analog](stm32/analog/README.md)
  * [ADC](stm32/analog/adc.md)
  * [OPAMP](stm32/analog/opamp.md)
  * [DAC](stm32/analog/dac.md)
* [Timers](stm32/timers/README.md)
  * [RTC](stm32/timers/rtc.md)
  * [TIM](stm32/timers/tim.md)
* [Connectivity](stm32/connectivity/README.md)
  * [UART](stm32/connectivity/uart.md)
  * [USART](stm32/connectivity/usart.md)
  * [USART - CAN Dongle (Fixed Size Serializer with Robust Timeout Handling)](stm32/connectivity/usart-can-dongle-fixed-size-serializer-with-robust-timeout-handling.md)
  * [CAN](stm32/connectivity/can.md)
  * [FDCAN](stm32/connectivity/fdcan.md)
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

* [RISC-V: Baremetal From The Ground Up (Chipyard Edition)](chipyard-soc-fpga/risc-v-baremetal-from-the-ground-up-chipyard-edition.md)
* [Setting up Chipyard](chipyard-soc-fpga/chipyard-environment-setup/README.md)
  * [Setting up Chipyard - Windows Subsystem Linux](chipyard-soc-fpga/chipyard-environment-setup/installing-chipyard-windows-subsystem-linux.md)
  * [Setting up Chipyard - Ubuntu](chipyard-soc-fpga/installing-chipyard.md)
  * [Setting Up Chipyard - BWRC Machines](chipyard-soc-fpga/setting-up-chipyard/setting-up-chipyard-bwrc-machines.md)
  * [Setting up Metals (Scala Language Server)](chipyard-soc-fpga/setting-up-metals-scala-language-server.md)
* [Setting up RISC-V Toolchain](chipyard-soc-fpga/setting-up-risc-v-toolchain/README.md)
  * [Setting up RISC-V Toolchain - Windows](chipyard-soc-fpga/setting-up-risc-v-toolchain/setting-up-risc-v-toolchain-windows.md)
  * [Setting up RISC-V Toolchain - Ubuntu](chipyard-soc-fpga/setting-up-risc-v-toolchain/setting-up-risc-v-toolchain-ubuntu.md)
  * [Setting up RV32 Toolchain on BWRC](finance/page-3.md)
  * [Setting up RISC-V Toolchain - Mac](chipyard-soc-fpga/setting-up-riscv-toolchain/setting-up-risc-v-toolchain-mac.md)
* [Running Chipyard Simulation](chipyard-soc-fpga/running-chipyard-simulation/README.md)
  * [Running Chipyard Simulation - Ubuntu](chipyard-soc-fpga/running-chipyard-simulation/running-chipyard-simulation-ubuntu.md)
  * [Running Chipyard Simulation - BWRC](chipyard-soc-fpga/running-chipyard-simulation/running-chipyard-simulation-bwrc.md)
* [Prototyping Chipyard SoC on FPGA](chipyard-soc-fpga/prototyping-chipyard-soc-on-fpga/README.md)
  * [Prototyping Chipyard SoC on FPGA - Ubuntu](chipyard-soc-fpga/prototyping-chipyard-soc-on-fpga/prototyping-chipyard-soc-on-fpga.md)
  * [Installing Xilinx Vivado - Ubuntu 22.04](chipyard-soc-fpga/prototyping-chipyard-soc-on-fpga/installing-xilinx-vivado-on-ubuntu-22.04.md)
  * [Arty 35T / 100T UART Pins](chipyard-soc-fpga/prototyping-chipyard-soc-on-fpga/arty-35t-100t-uart-pins.md)
  * [Arty 35T Using DDR DRAM](chipyard-soc-fpga/prototyping-chipyard-soc-on-fpga/arty-35t-using-ddr-dram.md)
  * [Vivado Generate Flash Config .mcs File From Bitstream](chipyard-soc-fpga/vivado-generate-flash-config-.mcs-file-from-bitstream.md)
* [Setting up FT-LINK Debugger](chipyard-soc-fpga/build-eaglex-linux-image/README.md)
  * [Setting up FT-LINK Debugger - Windows](chipyard-soc-fpga/build-eaglex-linux-image/ft-link-jtag-debugger-driver-setup.md)
  * [Setting up FT-LINK Debugger - Ubuntu](chipyard-soc-fpga/build-eaglex-linux-image/setting-up-ft-link-debugger-ubuntu.md)
* [Debugging the SoC](chipyard-soc-fpga/debugging/README.md)
  * [Page 2](chipyard-soc-fpga/debugging-the-soc/page-2.md)
  * [Debugging OsciArty with JTAG and command line GDB](chipyard-soc-fpga/debugging/gdb-debugging-osciarty.md)
  * [Debugging BearlyML with JTAG and GDB](chipyard-soc-fpga/debugging/jtag-and-gdb-debugging-bearlyml.md)
  * [Booting BearlyML With External SPI Flash](chipyard-soc-fpga/debugging/booting-bearlyml-with-external-spi-flash.md)
* [Setting Up SD / microSD Card for vcu118 Linux Image](chipyard-soc-fpga/setting-up-sd-microsd-card-for-vcu118-linux-image.md)
* [Page](chipyard-soc-fpga/page.md)

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
* [Setting up Recoil USB-CAN Adapter](motor-control/setting-up-recoil-usb-can-adapter/README.md)
  * [Setting up Recoil USB-CAN Adapter - Ubuntu](motor-control/setting-up-recoil-usb-can-adapter/setting-up-recoil-usb-can-adapter-ubuntu.md)
  * [Setting up Recoil USB-CAN Adapter - Windows](motor-control/setting-up-recoil-usb-can-adapter/setting-up-recoil-usb-can-adapter-windows.md)
* [NTC Temperature Sense Resistor Value Calculation](motor-control/ntc-temperature-sense-resistor-value-calculation.md)

## ML/RL

* [Zoo of RL Algorithms](ml-rl/zoo-of-rl-algorithms.md)
* [ROS 2](ml-rl/ros-2/README.md)
  * [Setting up ROS 2 Humble Hawksbill on Ubuntu](ml-rl/ros-2/setting-up-ros-2-humble-hawksbill-on-ubuntu.md)
  * [Setting up ROS 2 Humble Hawksbill on Windows 10](ml-rl/ros-2/setting-up-ros-2-humble-hawksbill-on-windows-10.md)
* [Google Colab](ml-rl/google-colab/README.md)
  * [Colab Resource Options](ml-rl/google-colab/colab-resource-options.md)
  * [so-vits-svc 4.0: Colab Flow](ml-rl/google-colab/so-vits-svc-4.0-colab-flow.md)
* [OpenAI gym + Mujoco Setup](ml-rl/setting-up-aws-mujoco-training-machine.md)
* [NVIDIA Omniverse Isaac Sim Setup](ml-rl/nvidia-omniverse-isaac-sim-setup.md)
* [NVIDIA Isaac Gym Setup](ml-rl/nvidia-isaac-gym-setup.md)
* [NVIDIA Isaac Gym URDF Import Notes](ml-rl/nvidia-isaac-gym-urdf-import-notes.md)
* [Gazebo Setup](ml-rl/gazebo-setup.md)
* [OnShape to URDF](ml-rl/onshape-to-urdf.md)
* [so-vits-svc 4.2](ml-rl/so-vits-svc-4.2-runtime.md)
* [Setting up MineDojo Environment](ml-rl/setting-up-minedojo-environment.md)

## Tools

* [Windows](tools/windows/README.md)
  * [Install WSL 2](tools/windows/install-wsl-2.md)
  * [Install Make on Windows](tools/windows/install-make-on-windows.md)
  * [Remove EFI disk partition](tools/windows/remove-efi-disk-partition.md)
  * [SAI Color Flip/Color Inversion](tools/windows/sai-color-flip-color-inversion.md)
  * [Microsoft Visual Studio Create Software Signature](tools/windows/microsoft-visual-studio-create-software-signature.md)
  * [Connecting the SIGLENT SDS1104X-U Oscilloscope to Computer](tools/windows/connecting-the-siglent-sds1104x-u-oscilloscope-to-computer.md)
  * [Using JADENS Thermal Label Printer](tools/windows/using-jadens-thermal-label-printer.md)
  * [Getting Started with XBee (ZigBee)](tools/windows/getting-started-with-xbee-zigbee.md)
* [Ubuntu](tools/ubuntu/README.md)
  * [Ubuntu 22.04 Standard Installation Procedure](tools/ubuntu/ubuntu-22.04-standard-installation-procedure.md)
  * [Linux Mounting SD Card](tools/ubuntu/linux-mounting-sd-card.md)
  * [Windows Ubuntu Dual Boot Issues](tools/ubuntu/windows-ubuntu-dual-boot-issues.md)
  * [Test Disk Read/Write Speed](tools/ubuntu/test-disk-read-write-speed.md)
  * [Configure USB Access Permissions (udev rules) on Ubuntu](tools/ubuntu/configure-usb-access-permissions-on-ubuntu.md)
  * [Screen Commands](tools/ubuntu/screen-commands.md)
  * [Disabling the "\<Application> is not responding." System Message on Ubuntu](tools/ubuntu/disabling-the-less-than-application-greater-than-is-not-responding.-system-message-on-ubuntu.md)
  * [Install and Configure GlobalProtect UC Berkeley VPN Service on Ubuntu 22.04](tools/ubuntu/install-and-configure-globalprotect-uc-berkeley-vpn-service-on-ubuntu-22.04.md)
* [Lab Automation](tools/lab-automation.md)
* [Github-Related Info](tools/github-related-info.md)
* [Python](tools/python/README.md)
  * [Publish Python Package to PyPi](tools/python/publish-python-package-to-pypi.md)
  * [Python Logging Utility](tools/python/python-logging-utility.md)
  * [Python converting bettwen JSON and XML](tools/python/python-converting-bettwen-json-and-xml.md)
  * [Retrieve Github user avatar with Github API](tools/python/retrieve-github-user-avatar-with-github-api.md)
* [SSH](tools/ssh.md)
* [Raspberry Pi Setup](tools/raspberry-pi-setup.md)
* [Clang-Format Style Config](tools/clang-format-style-config.md)
* [CrazyFlie Setting Up](tools/crazyflie-setting-up.md)
* [Using Oscilloscope: x1 vs x10](tools/using-oscilloscope-x1-vs-x10.md)
* [Using the BWRC 3D Printer](tools/using-the-bwrc-3d-printer.md)
* [Using the Leica Microscope at BWRC](tools/using-the-leica-microscope-at-bwrc.md)
* [Pair XBoxController to Raspberry Pi with Bluetooth](tools/pair-xboxcontroller-to-raspberry-pi-with-bluetooth.md)

## Mechanical

* [MAD Cycloidal Actuator](mechanical/mad-cycloidal-actuator.md)
* [Fixing the Unitree A1 Robot Dog](mechanical/fixing-the-unitree-a1-robot-dog.md)
* [3D Printer Profile](mechanical/3d-printer-profile.md)

## Electrical

* [A Note on the Polarity of the Famous TT Motor](electrical/a-note-on-the-polarity-of-the-famous-tt-motor.md)
* [Wiring Convention](electrical/wiring-convention.md)
* [MCU Pinmap Convention](electrical/mcu-pinmap-convention.md)
* [PCB Design and Manufacturing Conventions](electrical/pcb-design-and-manufacturing-conventions.md)
* [ESP32 Cam](electrical/esp32-cam.md)
* [LiPo Safety](electrical/lipo-safety.md)
* [AS5600 Modification](electrical/as5600-modification.md)

## AR/VR/XR

* [Digital-Twin Communication System](network/digital-twin-communication-system.md)
* [Unreal Engine Communicate with SteamVR](network/unreal-engine-communicate-with-steamvr-and-external-python.md)
* [A Note on Coordinate Systems](ar-vr-xr/a-note-on-coordinate-systems.md)
* [Unreal Engine Socket Communication](network/unreal-engine-socket-communication.md)
* [NewLine Serialization Method](network/newline-serialization-method.md)
* [Blender Python Related](3d-modeling/blender-python-related.md)

## 3D Modeling

* [3D Print Tolerancing](3d-modeling/3d-print-tolerancing.md)
* [Blender to OnShape Workflow](3d-modeling/blender-to-onshape-workflow.md)

## Maintainance Log

* [RISC-V Toolbox Website](maintainance-log/risc-v-toolbox-website.md)

## UW

* [AI233 Design Notes](uw/ai233-design-notes/README.md)
  * [Robot Body Ratio Issue](uw/ai233-design-notes/robot-body-ratio-issue.md)
  * [Note on Face Design and Manufacture](uw/ai233-design-notes/note-on-face-design-and-manufacture.md)
* [Network Setup](uw/network-setup.md)

## Finance

* [Finance](finance/tax-stuff.md)
* [UC Berkeley Reimbursement](finance/uc-berkeley-reimbursement.md)

## Life

* [Some Interview Questions](life/some-interview-questions.md)
* [Health Insurance](life/page-4.md)

## Group 1
