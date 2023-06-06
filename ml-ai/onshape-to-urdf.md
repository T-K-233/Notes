# OnShape to URDF

## Overview

We will be using the onshape-to-robot plugin to read robot model from Onshape, and export them to a urdf file.

{% embed url="https://github.com/rhoban/onshape-to-robot/" %}

## Installation

```bash
sudo apt install openscad meshlab
sudo apt install libcanberra-gtk-module
```

```bash
pip install onshape-to-robot
```



Also install ROS2 environment following this guide:

{% content-ref url="ros-2/setting-up-ros-2-humble-hawksbill-on-ubuntu.md" %}
[setting-up-ros-2-humble-hawksbill-on-ubuntu.md](ros-2/setting-up-ros-2-humble-hawksbill-on-ubuntu.md)
{% endcontent-ref %}



## Set up Onshape API access key

Set up an API access key at [OnShape developer portal](https://dev-portal.onshape.com/keys).



Store the API keys in `./scripts/onshape_api_key.sh`

```sh
# Obtained at https://dev-portal.onshape.com/keys
export ONSHAPE_API=https://cad.onshape.com
export ONSHAPE_ACCESS_KEY=Your_Access_Key
export ONSHAPE_SECRET_KEY=Your_Secret_Key
```



On every new terminal opened, run

```bash
source ./scripts/onshape_api_key.sh
```



## Set up the ROS2 workspace

> This procedure only needs to be performed when initially setting up a new workspace and package.

Create the package

```bash
mkdir -p ~/Desktop/humanoid-urdf/src
cd ~/Desktop/humanoid-urdf/src
ros2 pkg create --build-type ament_cmake humanoid_v1
```

Create additional folders

```bash
cd ~/Desktop/humanoid-urdf/src/humanoid_v1
mkdir -p config launch maps meshes models params rviz urdf worlds
```

Test build

```bash
cd ~/Desktop/humanoid-urdf
colcon build
```



If the package is built without error, try the following command.&#x20;

```
colcon_cd humanoid_v1
```

colcon\_cd should navigate to our package directory `~/Desktop/humanoid-urdf/src/humanoid_v1`



### Create Config Files

We need to create these files in the ROS package.

```bash
touch ~/Desktop/humanoid-urdf/src/humanoid_v1/launch/humanoid_v1.launch.py

touch ~/Desktop/humanoid-urdf/src/humanoid_v1/rviz/rviz_basic_settings.rviz

touch ~/Desktop/humanoid-urdf/src/humanoid_v1/urdf/robot.urdf.xacro
```



`humanoid_v1.launch.py`:&#x20;

```python
# Author: Addison Sears-Collins
# Date: September 14, 2021
# Description: Launch a robot URDF file using Rviz.
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Set the path to this package.
  pkg_share = FindPackageShare(package="humanoid_v1").find("humanoid_v1")

  # Set the path to the RViz configuration settings
  default_rviz_config_path = os.path.join(pkg_share, "rviz/rviz_basic_settings.rviz")

  # Set the path to the URDF file
  default_urdf_model_path = os.path.join(pkg_share, "urdf/robot.urdf.xacro")

  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  gui = LaunchConfiguration("gui")
  urdf_model = LaunchConfiguration("urdf_model")
  rviz_config_file = LaunchConfiguration("rviz_config_file")
  use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
  use_rviz = LaunchConfiguration("use_rviz")
  use_sim_time = LaunchConfiguration("use_sim_time")

  # Declare the launch arguments  
  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name="urdf_model", 
    default_value=default_urdf_model_path, 
    description="Absolute path to robot urdf file")
    
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name="rviz_config_file",
    default_value=default_rviz_config_path,
    description="Full path to the RVIZ config file to use")
    
  declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
    name="gui",
    default_value="True",
    description="Flag to enable joint_state_publisher_gui")
  
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name="use_robot_state_pub",
    default_value="True",
    description="Whether to start the robot state publisher")

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name="use_rviz",
    default_value="True",
    description="Whether to start RVIZ")
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name="use_sim_time",
    default_value="True",
    description="Use simulation (Gazebo) clock if true")
   
  # Specify the actions

  # Publish the joint state values for the non-fixed joints in the URDF file.
  start_joint_state_publisher_cmd = Node(
    condition=UnlessCondition(gui),
    package="joint_state_publisher",
    executable="joint_state_publisher",
    name="joint_state_publisher")

  # A GUI to manipulate the joint state values
  start_joint_state_publisher_gui_node = Node(
    condition=IfCondition(gui),
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui",
    name="joint_state_publisher_gui")

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{"use_sim_time": use_sim_time, 
    "robot_description": Command(["xacro ", urdf_model])}],
    arguments=[default_urdf_model_path])

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="screen",
    arguments=["-d", rviz_config_file])
  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_joint_state_publisher_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)

  # Add any actions
  ld.add_action(start_joint_state_publisher_cmd)
  ld.add_action(start_joint_state_publisher_gui_node)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)

  return ld


```



rviz

```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /RobotModel1
        - /TF1
        - /TF1/Frames1
      Splitter Ratio: 0.5
    Tree Height: 617
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
        base_footprint:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        base_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        hip_x_l_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        hip_x_r_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        hip_y_l_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        hip_y_r_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        hip_z_l_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        hip_z_r_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        wheel_1_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        wheel_2:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        front_caster:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        gps_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        imu_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        lidar_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: false
        base_link:
          Value: false
        hip_x_l_link:
          Value: true
        hip_x_r_link:
          Value: true
        front_caster:
          Value: false
        gps_link:
          Value: false
        imu_link:
          Value: false
        lidar_link:
          Value: false
      Marker Scale: 1
      Name: TF
      Show Arrows: false
      Show Axes: true
      Show Names: true
      Tree:
        base_footprint:
          base_link:
            drivewhl_l_link:
              {}
            drivewhl_r_link:
              {}
            front_caster:
              {}
            gps_link:
              {}
            imu_link:
              {}
            lidar_link:
              {}
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 4.434264183044434
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0.31193429231643677
        Y: 0.11948385089635849
        Z: -0.4807402193546295
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.490397572517395
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 1.0503965616226196
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002f4fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002f4000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002f4fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002f4000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d006501000000000000045000000000000000000000023f000002f400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1200
  X: 246
  Y: 77
```



urdf

```
<robot name="onshape">

    <link name="base_link">
    </link>
    <joint name="base_anchor_joint" type="fixed">
      <parent link="base_link"/>
      <child link="body_1_link"/>
    </joint>
    <link name="body_1_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://biped_v1/meshes/body.stl" />
            </geometry>
            <material name="body_material">
                <color rgba="0.8 0.8 0.8 1.0" />
            </material>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://biped_v1/meshes/body.stl" />
            </geometry>
            <material name="body_material">
                <color rgba="0.8 0.8 0.8 1.0" />
            </material>
        </collision>
        <inertial>
            <origin xyz="0.0375 0.0253843 0.0225241" rpy="0 0 0" />
            <mass value="2.23874" />
            <inertia ixx="0.00206023" ixy="4.48497e-10" ixz="6.47006e-11" iyy="0.00150691"
                iyz="-0.00028761" izz="0.00254354" />
        </inertial>
    </link>

    <link name="wheel_1_link">
        <visual>
            <origin xyz="0.02 -0.0325 -0.025" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://biped_v1/meshes/wheel.stl" />
            </geometry>
            <material name="wheel_material">
                <color rgba="0.262745 0.282353 0.301961 1.0" />
            </material>
        </visual>
        <collision>
            <origin xyz="0.02 -0.0325 -0.025" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://biped_v1/meshes/wheel.stl" />
            </geometry>
            <material name="wheel_material">
                <color rgba="0.262745 0.282353 0.301961 1.0" />
            </material>
        </collision>
        <inertial>
            <origin xyz="0.004 0 -3.46945e-18" rpy="0 0 0" />
            <mass value="0.335419" />
            <inertia ixx="0.000345298" ixy="0" ixz="0" iyy="0.000174367" iyz="0" izz="0.000174367" />
        </inertial>
    </link>

    <joint name="wheel_1_joint" type="revolute">
        <origin xyz="-0.02 0.0325 0.025" rpy="0 -0 0" />
        <parent link="body_1_link" />
        <child link="wheel_1_link" />
        <axis xyz="-1 0 0" />
        <limit upper="1.0472" lower="-1.0472" effort="20" velocity="10.0"/>
    </joint>

    <link name="wheel_2">
        <visual>
            <origin xyz="0.012 -0.0325 -0.025" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://biped_v1/meshes/wheel.stl" />
            </geometry>
            <material name="wheel_material">
                <color rgba="0.262745 0.282353 0.301961 1.0" />
            </material>
        </visual>
        <collision>
            <origin xyz="0.012 -0.0325 -0.025" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://biped_v1/meshes/wheel.stl" />
            </geometry>
            <material name="wheel_material">
                <color rgba="0.262745 0.282353 0.301961 1.0" />
            </material>
        </collision>
        <inertial>
            <origin xyz="-0.004 6.93889e-18 -3.46945e-18" rpy="0 0 0" />
            <mass value="0.335419" />
            <inertia ixx="0.000345298" ixy="0" ixz="0" iyy="0.000174367" iyz="0" izz="0.000174367" />
        </inertial>
    </link>

    <joint name="wheel2_speed" type="revolute">
        <origin xyz="0.095 0.0325 0.025" rpy="0 -0 0" />
        <parent link="body_1_link" />
        <child link="wheel_2" />
        <axis xyz="-1 -0 -0" />
        <joint_properties friction="0.0" />
        <limit upper="1.0472" lower="-1.0472" effort="20" velocity="10.0"/>
    </joint>

    <link name="wheel3_link">
        <inertial>
          <mass value="0.9"/>
          <origin xyz="0 0 0"/>
          <inertia ixx=".004" ixy="1.445e-11" ixz="-1.922e-4" iyy="0.004" iyz="1.954e-11" izz="2.696e-4"/>
        </inertial>
      </link>
      <joint name="wheel3_joint" type="revolute">
        <parent link="body_1_link"/>
        <child link="wheel3_link"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <dynamics damping="0.01" friction="0.1"/>
        <limit upper="1.0472" lower="-1.0472" effort="20" velocity="10.0"/>
        <axis xyz="1 0 0"/>
      </joint>
</robot>
    
```





### Change Files

Change `package.xml` by adding these lines:

```markup

  <buildtool_depend>ament_cmake</buildtool_depend>

<!-- insert the following lines -->
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz</exec_depend>
  <exec_depend>xacro</exec_depend>
<!-- end of insertion -->

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

```



Change `CMakeLists.txt` by adding these lines:

```cmake
# find_package(<dependency> REQUIRED)

# insert the following lines
install(
  DIRECTORY config launch maps meshes models params rviz src urdf worlds
  DESTINATION share/${PROJECT_NAME}
)
# end of insertion

if(BUILD_TESTING)
```



### Test Build

```bash
make ros-build
```

colcon build

source install/setup.sh

ros2 launch humanoid\_v1 humanoid\_v1.launch.py



If the following error is raised, we need to install the gui package separately, suggested [here](https://answers.ros.org/question/344992/missing-joint\_state\_publisher\_gui-when-l-run-displaylaunch/).

```bash
[ERROR] [launch]: Caught exception in launch (see debug for traceback): "package 'joint_state_publisher_gui' not found, searching: ['/home/tk/Desktop/humanoid-urdf/install/humanoid_v1', '/opt/ros/humble']"
```

```bash
sudo apt install ros-humble-joint-state-publisher-gui
```



might also need

```
sudo apt install ros-humble-xacro
```



if running into this error:

```bash
[rviz2-3] /opt/ros/humble/lib/rviz2/rviz2: symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
[ERROR] [rviz2-3]: process has died [pid 8882, exit code 127, cmd '/opt/ros/humble/lib/rviz2/rviz2 -d /home/tk/Desktop/humanoid-urdf/install/humanoid_v1/share/humanoid_v1/rviz/rviz_basic_settings.rviz --ros-args -r __node:=rviz2'].
[joint_state_publisher_gui-1] /usr/bin/python3: symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
[ERROR] [joint_state_publisher_gui-1]: process has died [pid 8878, exit code 127, cmd '/opt/ros/humble/lib/joint_state_publisher_gui/joint_state_publisher_gui --ros-args -r __node:=joint_state_publisher_gui'].
```



highly possible that this is a VSCode terminal issue. Do

```bash
unset GTK_PATH
```







## Export Onshape robot

```bash
make urdf-build
```

This command will dump the urdf and related asset files in `./onshape/humanoid_v1` directory, and automatically copy the necessary files to the ROS2 package.



To clear the previous build, run

```bash
make urdf-clean
```

```
onshape-to-robot humanoid_v1
```





## Change the exported urdf



For exported urdf, we need to make several changes

### 1. Add base\_link

```xml
<link name="base_link">
</link>
<joint name="base_anchor_joint" type="fixed">
<parent link="base_link"/>
<child link="body_1"/>
</joint>
```

### 2. Change mesh path setting

```
package://mesh.stl -> package://humanoid_v1/meshes/mesh.stl
```

### 3. Add position limit



### 4. Collision

```
% scale(1000) import("extrusion_100mm.stl");

translate([65, 0, 0])
cube([20, 100, 20], center=true);

```





## Build

To force refresh the static files, do

```bash
rm -rf ~/Desktop/humanoid-urdf/install/humanoid_v1/share/
```

and then

```
colcon build && ros2 launch humanoid_v1 humanoid_v1.launch.py
```







```bash
sudo apt install openscad
```

```bash
onshape-to-robot-edit-shape
```

