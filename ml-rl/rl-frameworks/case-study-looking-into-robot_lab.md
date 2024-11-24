# Case Study: Looking into robot\_lab

robot\_lab provides both the normal training pipeline and also the AMP version. In this article, we will focuse on the AMP training pipeline.



## Retarget Motion

```bash
python exts/robot_lab/robot_lab/third_party/amp_utils/scripts/retarget_kp_motions.py
```



First, we need to define the robot dimensions and mocap data attributes

<figure><img src="../../.gitbook/assets/image (239).png" alt=""><figcaption></figcaption></figure>



These FK chains are used for solving the foot local positions

<figure><img src="../../.gitbook/assets/image (240).png" alt=""><figcaption></figcaption></figure>



### Create PyBullet simulation

first, we create pybullet simulation



### Process mocap data

we iterate through all the mocap data specified in `config.MOCAP_MOTIONS`&#x20;

for each mocap data, we need to reset the pybullet simulation, re-add the ground and robot urdf, and then set the robot pose to be the init pose.



For following section, we will use example reference motion`dog_walk00_joint_pos.txt`



### Load mocap data

Then, we load the reference motion txt.

```python
JOINT_POS_FILENAME = "/home/tk/Downloads/robot_lab/exts/robot_lab/robot_lab/third_party/amp_utils/datasets/keypoint_datasets/ai4animation/dog_walk00_joint_pos.txt"
joint_pos_data = np.loadtxt(JOINT_POS_FILENAME, delimiter=",")
```

81 floating point numbers on each line

then, we clip out the unused frames, after concat

```
# joint_pos_data: (40, 81), float64
```



reshape to each position tracker's position

```python
joint_pos_data = joint_pos_data.reshape(joint_pos_data.shape[0], -1, POS_SIZE)
# joint_pos_data: (40, 27, 3), float64
```

We then know that there are 27 mocap markers used in this motion sequence



### Marker coordiate transform

process\_ref\_joint\_pos\_data



```python
        # 对数据进行坐标变换
        for i in range(joint_pos_data.shape[0]):
            joint_pos_data[i] = process_ref_joint_pos_data(joint_pos_data[i])
```

```python
# 1. Rotates the point around the reference coordinate system
curr_pos = pose3d.QuaternionRotatePoint(curr_pos, REF_COORD_ROT)
# REF_COORD_ROT = rotation of 90 degrees around X axis (0.5 * pi)

# 2. Applies a root rotation to align the motion
curr_pos = pose3d.QuaternionRotatePoint(curr_pos, REF_ROOT_ROT) 
# REF_ROOT_ROT = rotation of ~85 degrees around Z axis (0.47 * pi)

# 3. Scales the position and adds an offset
curr_pos = curr_pos * config.REF_POS_SCALE + REF_POS_OFFSET
```

The purpose of these transformations is to match the frame between robot and mocap data by:

* Convert from the motion capture coordinate system to the simulation coordinate system (first rotation)
* Orient the motion in the desired direction (second rotation)
* Scale and offset the positions to match the robot's size and starting position



### Adjust feet tip position

<figure><img src="../../.gitbook/assets/image (241).png" alt=""><figcaption></figcaption></figure>



### Retarget Motion





```python
# retargeted_frame: (39, 61)
```



```python
curr_pose: 31
  root_pos: 3
  root_rot: 4
  joint_pose: 12
  tar_toe_pos_local: 12
LINEAR_VEL_SIZE: 3
ANGULAR_VEL_SIZE: 3
JOINT_POS_SIZE: 12
TAR_TOE_VEL_LOCAL_SIZE: 12
```






