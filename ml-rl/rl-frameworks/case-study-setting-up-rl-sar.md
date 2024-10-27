# Case Study: Setting up RL-SAR



Clone repo

```bash
mkdir -p ~/Desktop/ros_ws/src/
cd ~/Desktop/ros_ws/src/

git clone https://github.com/fan-ziqi/rl_sar.git
```



Install system dependencies

```bash
sudo apt install liblcm-dev libyaml-cpp-dev
```



Install ROS dependencies

```bash
sudo apt install \
  ros-noetic-teleop-twist-keyboard \
  ros-noetic-controller-interface \
  ros-noetic-gazebo-ros-control \
  ros-noetic-joint-state-controller \
  ros-noetic-effort-controllers \
  ros-noetic-joint-trajectory-controller
```



Install PyTorch Cpp binding

```bash
mkdir -p ~/Documents/
cd ~/Documents/
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
rm ~/Documents/libtorch/libtorch-*.zip
cd -
```

{% code title="~/.bashrc" %}
```bash
# PyTorch C++ Binding
export TORCH_DIR=/home/tk/Documents/libtorch/
```
{% endcode %}



```bash
export Torch_DIR=/home/tk/Documents/libtorch/
```



Build source

```bash
# cd ./ros_ws/
catkin build
```



## Sim2Sim



```bash
source ./devel/setup.bash
```



```bash
roslaunch rl_sar gazebo_a1_isaacgym.launch
```



```bash
rosrun rl_sar rl_sim
```



## Sim2Real



```bash
rosrun rl_sar rl_real_a1
```









