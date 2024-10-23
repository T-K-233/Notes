# Case Study: Setting up RL-SAR



```bash
git clone https://github.com/fan-ziqi/rl_sar.git
```



```bash
sudo apt install ros-humble-teleop-twist-keyboard
sudo apt install ros-humble-controller-interface
sudo apt install ros-humble-gazebo-ros2-control
#sudo apt install ros-humble-joint-state-controller
sudo apt install ros-humble-effort-controllers
sudo apt install ros-humble-joint-trajectory-controller
```



```bash
mkdir -p ~/Documents/libtorch/
cd ~/Documents/libtorch/
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./

```

```
export TORCH_DIR=/home/tk/Documents/libtorch/
```





```bash
sudo apt install liblcm-dev libyaml-cpp-dev
```



