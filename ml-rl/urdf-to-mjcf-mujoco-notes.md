# URDF to MJCF Mujoco Notes

1\. urdf to mjcf

Follow this link to convert urdf to mjcf: [https://github.com/wangcongrobot/dual\_ur5\_husky\_mujoco](https://github.com/wangcongrobot/dual\_ur5\_husky\_mujoco)





2\. mjcf modifications

reference path:&#x20;

x-robot-mjpy/assets/robot.xml



add mujoco option

add default tag (can copy from reference)

add floor (can copy from reference)

make sure the root body is base, and base has a free joint



add inertial to base body (can copy from reference)

values: position take from urdf, the interia tag of link=base value

mujoco inertia order:

```
ixx, iyy, izz, ixy, ixz, iyz
```





manually pick the collision parts, add `class="collision"` attribute

add actuator motor

add sensor









