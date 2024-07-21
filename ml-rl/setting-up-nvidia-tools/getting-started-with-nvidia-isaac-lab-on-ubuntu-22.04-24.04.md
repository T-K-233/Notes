# Getting Started with NVIDIA Isaac Lab on Ubuntu 22.04 / 24.04



## Installation

The official instruction is pretty straight forward: [https://isaac-sim.github.io/IsaacLab/source/setup/installation/pip\_installation.html](https://isaac-sim.github.io/IsaacLab/source/setup/installation/pip\_installation.html)



Prepare build tools

```bash
sudo apt install cmake build-essential
```



Create conda environment

```bash
conda create -yn isaaclab python=3.10
```

```bash
conda activate isaaclab
```



Install Python packages

```bash
pip install torch
```

{% code overflow="wrap" %}
```bash
pip install isaacsim-rl isaacsim-replicator isaacsim-extscache-physics isaacsim-extscache-kit-sdk isaacsim-extscache-kit isaacsim-app --extra-index-url https://pypi.nvidia.com
```
{% endcode %}



Clone repository and install rsl\_rl component

```bash
cd ~/Desktop/
git clone git@github.com:isaac-sim/IsaacLab.git
cd ./IsaacLab/
```

```bash
./isaaclab.sh --install rsl_rl
```



Verify installation

```bash
./isaaclab.sh -p source/standalone/tutorials/00_sim/create_empty.py
```



## List Environments

```bash
./isaaclab.sh -p source/standalone/environments/list_envs.py
```



## Example Train and Play script

{% code overflow="wrap" %}
```bash
python source/standalone/workflows/rsl_rl/train.py --task Isaac-Velocity-Flat-G1-v0 --run_name example_run
```
{% endcode %}



{% code overflow="wrap" %}
```bash
python source/standalone/workflows/rsl_rl/play.py --task Isaac-Velocity-Flat-G1-v0 --num_envs 4
```
{% endcode %}







