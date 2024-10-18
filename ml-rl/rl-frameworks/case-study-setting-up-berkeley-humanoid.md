# Case Study: Setting up Berkeley Humanoid

## System

Running with the following system configuration:

* Ubuntu 22.04
* NVIDIA Driver 535.183.06
* CUDA 12.2
* Python 3.10
* IsaacLab 4.2.0



### Install Dependencies

```bash
sudo apt install ninja-build
sudo apt install build-essential
```



```bash
sudo apt install ros-humble-pinocchio
```



```bash
# pip install onnxruntime
```

```bash
cd /tmp
wget https://github.com/microsoft/onnxruntime/releases/download/v1.7.0/onnxruntime-linux-x64-1.7.0.tgz
tar xf onnxruntime-linux-x64-1.7.0.tgz
mkdir -p ~/.local/bin ~/.local/include/onnxruntime ~/.local/lib ~/.local/share/cmake/onnxruntime
rsync -a /tmp/onnxruntime-linux-x64-1.7.0/include/ ~/.local/include/onnxruntime
rsync -a /tmp/onnxruntime-linux-x64-1.7.0/lib/ ~/.local/lib
wget https://raw.githubusercontent.com/leggedrobotics/ocs2/refs/heads/main/ocs2_mpcnet/ocs2_mpcnet_core/misc/onnxruntime/cmake/onnxruntimeConfig.cmake -O ~/.local/share/cmake/onnxruntime/onnxruntimeConfig.cmake
wget https://raw.githubusercontent.com/leggedrobotics/ocs2/refs/heads/main/ocs2_mpcnet/ocs2_mpcnet_core/misc/onnxruntime/cmake/onnxruntimeVersion.cmake -O ~/.local/share/cmake/onnxruntime/onnxruntimeVersion.cmake
cd -
```



```bash
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH
```



## Setting up environment

Create the project directory and clone the required repositories. In this example, we will be working in the `./berkeley-humanoid` folder.

```bash
mkdir ./berkeley-humanoid/
cd ./berkeley-humanoid/
git clone https://github.com/isaac-sim/IsaacLab.git
git clone https://github.com/HybridRobotics/isaac_berkeley_humanoid.git
git clone git@github.com:qiayuanl/legged_control2.git
```



Create conda environment

```bash
conda create -yn humanoid-isaaclab python=3.10
conda activate humanoid-isaaclab
```



Install Python package dependencies for IsaacSim

{% code overflow="wrap" %}
```bash
pip install isaacsim-rl isaacsim-replicator isaacsim-extscache-physics isaacsim-extscache-kit-sdk isaacsim-extscache-kit isaacsim-app --extra-index-url https://pypi.nvidia.com
```
{% endcode %}



Go to the IsaacLab directory and install RSL-RL along with IsaacLab

```bash
cd ./IsaacLab
./isaaclab.sh --install rsl_rl
cd ..
```

Accept the EULA when prompted.



Go to the Berkeley Humanoid extension and install this module

```bash
cd ./isaac_berkeley_humanoid/exts/berkeley_humanoid
pip install -e .
cd ../../..
```



## Training the policy

```bash
cd ./isaac_berkeley_humanoid/
```



```bash
python ./scripts/rsl_rl/train.py --task Velocity-Rough-Berkeley-Humanoid-v0
```

For first-time running the script, it might take a few minutes for IsaacLab to load all the configuration and compile the shaders.



Running in headless mode

```bash
python ./scripts/rsl_rl/train.py --task Velocity-Rough-Berkeley-Humanoid-v0 --headless
```



Test out the trained policy

```bash
python ./scripts/rsl_rl/play.py --task Velocity-Rough-Berkeley-Humanoid-Play-v0
```



### Sim2sim transfer



source ros dependencies

```bash
source /opt/ros/humble/setup.sh
source /usr/share/colcon_cd/function/colcon_cd.sh
```





```bash
mkdir -p ./ros2_ws/src/
cd ./ros2_ws/src/
```



```bash
# cd ./ros2_ws/src/
git clone git@github.com:qiayuanl/berkeley_humanoid_description.git

# cd ./ros2_ws/src/berkeley_humanoid_description
git checkout feature/ament_cmake
```



```bash
# cd ./ros2_ws/
colcon build --packages-select berkeley_humanoid_description
```

```bash
source ./install/setup.bash
```



```bash
ros2 launch berkeley_humanoid_description view_robot.launch.py
```



```bash
ros2 launch berkeley_humanoid_description gazebo.launch.py
```





{% code overflow="wrap" %}
```bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo --event-handlers console_direct+ --packages-up-to legged_rl_controller
```
{% endcode %}





