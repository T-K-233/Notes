# Recoil Documentation

## Initialization

We have a bunch of stuff to initialize. The most important aspect is the sequence of these initializations ---- there are some dependencies, and violating them creates a very serious safety hazard.



0\.

For a new chip, we need to set up Flash option bytes first. This is marked by the `FIRST_TIME_BOOTUP` macro flag.

When the flag is set, we run the `APP_initFlashOption()` function, which sets the boot method to boot from Flash.

After setting up, we set the LED to indicate the operation is finished, and then halt into an infinite loop to wait for firmware reflash.



1\.

Otherwise, we enter the normal `MotorController_init()` routine.

It will first set mode to `MODE_ERROR` with error code `ERROR_INITIALIZE_ERROR`. This is in case anything interrupts the initialization, then the system won't start. It will also load the `CAN_ID` and `FIRMWARE_VERSION`.



2\.&#x20;

Then, we initialize all the components. Here, we will use initialize all the volatile variables to appropriate values, and set non-volatile variables to some user values. The idea is that if `LOAD_CONFIG_FROM_FLASH` or `LOAD_CALIBRATION_FROM_FLASH` is set, then the flash loader will override these user values for the selected fields. Otherwise it will use these preset values.



`LOAD_CALIBRATION_FROM_FLASH` controls whether the controller uses the Motor.flux\_angle\_offset from flash.

`LOAD_CONFIG_FROM_FLASH` controls whether the controller uses other parameters from flash.



If any one of the flags is not set, then we also write the Flash with the most up to date settings.



3\.

Then we start with initializing the CAN communication and setting up CAN filters.

CAN interrupt will be available.

The device will start to be responsive to CAN comm, and device status will be MODE\_ERROR and ERROR\_INITIALIZE\_ERROR.



4\.

Start opamp and ADC. The conversion will not start yet, because it is triggered by PowerStage timer.



6\.

Start periphery timers. PowerStage timer is NOT enabled at this stage.

Because watchdog timer is started, the CAN bus communication watchdog will start ticking from here.

Watchdog timer interrupt will be available.



7\.

Configure default LED state.

The LED will be constant white.



8\.

Start PowerStage timer.



9\.

Calibrate current sampling offset.



10\.&#x20;

Set mode to MODE\_IDLE, and error code to ERROR\_NO\_ERROR. Exit initialization.



## State Machine

Currently we support the following modes:

**Disabled (sleep)**

In this mode, the PowerStage will not be enabled (ENABLE pin pulled low for DRV835x, and PWM disabled for BESC). Only communication will be running. The motor controller will boot up in this mode. After initialization, the mode will be set to IDLE by the initialize() function.

This is a safe mode.

Note: Disabled is NOT used as a user mode for now.

**Idle**

PWM no output, all FETs are disabled, the motor is in floating mode.

This is a safe mode.

**Damping**

PWM output LOW, all lower FETs are open, the motor is in damping mode.

**Calibration**

This is the only blocking mode that is executed by the main routine.

**Current**

Closed-loop current control. User sets the desired value using `iq_target` and `id_target`

**Torque**

Closed-loop torque control. User sets the desired value using `torque_target`.



### State Switching

On bootup, the controller will be in DISABLED mode (by default the variable initialized to 0, which is DISABLED; the first line of initialize() will also reset this to ensure this state).

If no error occured during initialization, the state will be changed to IDLE by the last line of initialize().



State switching is handled by the commutation loop running at 20kHz. The minimal safety requirement is that the commutation loop should always be executed in time and cannot be halted. Otherwise, the state cannot be switched correctly, risking error signal passed to PowerStage.





## Update (Commutation) Routine





