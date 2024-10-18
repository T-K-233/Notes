# Case Study: Setting up Berkeley Humanoid

## System

Running with the following system configuration:

* Ubuntu 22.04
* NVIDIA Driver 535.183.06
* CUDA 12.2
* Python 3.10
* IsaacLab 4.2.0



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





