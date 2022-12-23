# Gazebo Setup

With ROS 2 Humble setup, run

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```





## Workspace Setup

{% embed url="https://automaticaddison.com/how-to-create-a-workspace-ros-2-foxy-fitzroy/" %}

Create workspace folder

```bash
mkdir -p ~/Desktop/test_ws/src
cd ~/Desktop/test_ws/src
```



Add pre-existing packages into this folder. We will add the ros\_tutorials repository to our workspace. It contains the following packages: roscpp\_tutorials, rospy\_tutorials, ros\_tutorials, and turtlesim.

```bash
git clone https://github.com/ros/ros_tutorials.git -b humble-devel
```



Resolve dependencies

```bash
cd ~/Desktop/test_ws/
rosdep install -i --from-path src --rosdistro humble -y
```



### Build the workspace

Make sure colcon is installed. Colcon is a tool used to build software packages.

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```



```bash
colcon build
```

<figure><img src="../.gitbook/assets/image (2).png" alt=""><figcaption></figcaption></figure>

If the above error occurs when building turtlesim, run

```bash
pip install empy
pip install lark
```

and then rebuild

```bash
rm -rf ~/Desktop/test_ws/build/
rm -rf ~/Desktop/test_ws/install/
colcon build
```







### Source the workspace

```bash
source ~/Desktop/test_ws/install/setup.bash
```





## URDF&#x20;

