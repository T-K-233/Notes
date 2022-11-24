# Failed Attempt on Acceleration- and Velocity-Limited Trajectory Generation



```c

  float dt = 1 / 8000.;
  float accel = 30;

  float k_p = 0.002;
  float k_d = 0.001;

  float dx = controller->position_target - controller->position_measured;
  controller->dx = dx;
  float decel_distance = (controller->velocity_measured * controller->velocity_measured) / (2 * accel);
  controller->decel_distance = decel_distance;
  if (controller->velocity_measured * dx > 0) {
    if (fabs(dx) > 1.2 * decel_distance) { // accelerate
      accel = copysignf(accel, dx);
    }
    else { // decelerate
      accel = copysignf(accel, -controller->velocity_measured);
    }
  }
  else {
    accel = copysignf(accel, -controller->velocity_measured);
  }


  controller->velocity_setpoint += accel * dt;
  controller->position_setpoint += controller->velocity_setpoint * dt;
//  controller->position_setpoint = clampf(controller->position_setpoint, -2*M_PI, 2*M_PI);

  if (mode != MODE_TORQUE) {
    controller->torque_setpoint = k_p * (controller->position_setpoint - controller->position_measured) + k_d * (controller->velocity_setpoint - controller->velocity_measured);

    controller->accel = accel;

    controller->torque_setpoint = clampf(
        controller->torque_setpoint,
        -controller->torque_limit,
        controller->torque_limit);


//    controller->torque_setpoint = (controller->position_controller.torque_setpoint * (float)controller->motor.kv_rating) / 8.3;
//    controller->current_controller.i_d_target = 0;

//    controller->torque_setpoint =
//        controller->torque_target - controller->torque_measured;
  }
  else {
    controller->torque_setpoint = controller->torque_target;
  }
```



<figure><img src="../.gitbook/assets/image.png" alt=""><figcaption></figcaption></figure>

The actuator will oscillate around the target position

