# Case Study: A Dive Into Unitree-Mujoco





```python
# ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml"
mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

```



## Base and Joint Data

The state of the robot can be accessed via the `mj_data.qpos` and `mj_data.qvel` fields.

These two fields are two `numpy.ndarray` that stores both the base state as well as the joint state.

The shape of `qpos` is `3 + 4 + n_dof`, which composes of the base xyz position, base rotation in quaternion, and all the joint rotation positions.

The shape of `qvel` is `3 + 3 + n_dof`. The only difference from the qpos is that the angular rotation is expressed in euler angle, instead of quaternions.

```bash
mj_data.qpos  # (base_x, base_y, base_z, base_qx, base_qy, base_qz, base_qw, joint...)
mj_data.qvel  # (base_vx, base_vy, base_vz, base_vrx, base_vry, base_vrz, jointv...)

```



## Joint Ordering

Different from IsaacLab, which orders the joint in a breath-first search fashion and ordered by capital letter, Mujoco uses a depth-first search. Therefore, the joint order will be different. Here's an example for the A1 robot.



Mujoco Mapping:

```python
 0: FL_Hip
 1: FR_Hip
 2: RL_Hip
 3: RR_Hip
 4: FL_Thigh
 5: FR_Thigh
 6: RL_Thigh
 7: RR_Thigh
 8: FL_Calf
 9: FR_Calf
10: RL_Calf
11: RR_Calf
```



IsaacLab Mapping

```python
 0: FL_Hip
 1: FL_Thigh
 2: FL_Calf
 3: FR_Hip
 4: FR_Thigh
 5: FR_Calf
 6: RL_Hip
 7: RL_Thigh
 8: RL_Calf
 9: RR_Hip
10: RR_Thigh
11: RR_Calf
```















