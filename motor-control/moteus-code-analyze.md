# Moteus Code Analyze

## Current Sampling

[article](https://jpieper.com/2020/08/24/stm32g4-adc-and-low-torque-operation/)



## Position Sampling

[article](https://jpieper.com/2021/05/10/filtering-encoder-values-in-moteus/)

[encoder calibration](https://jpieper.com/2019/01/26/encoder-autocalibration/)

Using All-Digital Phase Locked Loop filter



Handled in `motor_position.h`

`pll_filter_hz` = 400



filter.kp = 2 \* 400 \* 2pi

filter.ki = (400 \* 2pi) \*\* 2



position += dt \* filter.kp \* error

velocity += dt \* filter.ki \* error



## Position Control with Vel and Acc Limit

[article](https://jpieper.com/2022/04/08/velocity-and-acceleration-limited-trajectories/)

User will issue position control command

```
d pos 0 0 0.1
```

<figure><img src="../.gitbook/assets/image.png" alt=""><figcaption></figcaption></figure>

which will be handled by the `ISR_DoPosition()` function in `bldc_servo.cc`

```c
  void ISR_DoPosition(const SinCos& sin_cos, CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    PID::ApplyOptions apply_options;
    apply_options.kp_scale = data->kp_scale;
    apply_options.kd_scale = data->kd_scale;

    ISR_DoPositionCommon(sin_cos, data, apply_options, data->max_torque_Nm,
                         data->feedforward_Nm, data->velocity);
  }
```



The service routine first will call `BldcServoPosition::UpdateCommand()` in `bldc_servo_position.h`



If the trajectory is not done, it will call `UpdateTrajectory()`

which then will call `DoVelocityAndAccelLimits`



inside the funciton,

`a` is the acceleration limit

`v0` is the current velocity

`vf` is the target velocity

`position_relative_raw` is the target position

dx is the position error

dv is the velocity error

acceleration is calculated as:

```python
a = accel_limit

if abs(v0) > velocity_limit:
  a = sign(-v0) * a
else:
  if v0 * dx > 0:
    decel_distance = v0**2 / 2a
    if abs(dx) >= decel_distance:
      if abs(v0) < velocity_limit:
        a = sign(dx) * a
      else:
        a = 0
    else:
      a =sign(-v0) * a
  else:
    a = sign(-v0) * a

```

