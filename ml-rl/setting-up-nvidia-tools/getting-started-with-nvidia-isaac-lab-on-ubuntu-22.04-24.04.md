# Getting Started with NVIDIA Isaac Lab on Ubuntu 22.04 / 24.04



## Installation

The official instruction is pretty straight forward: [https://isaac-sim.github.io/IsaacLab/source/setup/installation/pip\_installation.html](https://isaac-sim.github.io/IsaacLab/source/setup/installation/pip_installation.html)



Prepare build tools

```bash
sudo apt install cmake build-essential
```



Clone repository

```bash
git clone git@github.com:isaac-sim/IsaacLab.git
```

```bash
cd ./IsaacLab/
```



Create conda environment

```bash
conda create -yn isaaclab python=3.10

# alternatively,
# conda create -yp ./.conda-env/ python=3.10
```

```bash
conda activate isaaclab
```



Install Python packages

{% code overflow="wrap" %}
```bash
pip install isaacsim[all,extscache]==4.5.0 --extra-index-url https://pypi.nvidia.com
```
{% endcode %}



Install rsl\_rl component

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
python scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Flat-G1-v0 --run_name example_run
```
{% endcode %}

Train headless and record video

{% code overflow="wrap" %}
```bash
python scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Flat-G1-v0 --video --video_length 100 --video_interval 2000 --headless
```
{% endcode %}





{% code overflow="wrap" %}
```bash
python scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Flat-G1-v0 --num_envs 4
```
{% endcode %}



{% code overflow="wrap" %}
```bash
./isaaclab.sh -p source/standalone/workflows/rsl_rl/train.py --task Isaac-Reach-Franka-v0 --headless
# run script for playing with 32 environments
./isaaclab.sh -p source/standalone/workflows/rsl_rl/play.py --task Isaac-Reach-Franka-v0 --num_envs 32 --load_run run_folder_name --checkpoint model.pt
# run script for recording video of a trained agent (requires installing `ffmpeg`)
./isaaclab.sh -p source/standalone/workflows/rsl_rl/play.py --task Isaac-Reach-Franka-v0 --headless --video --video_length 200
```
{% endcode %}





## Errata

If encounter this error

{% code overflow="wrap" %}
```bash
.isaac.core/omni/isaac/core/robots`: errno=28/No space left on device
2024-08-20 07:19:57 [9,094ms] [Error] [carb] Failed to create change watch for `/home/tk/Desktop/IsaacLab/.conda-env/lib/python3.10/site-packages/isaacsim/exts/omni.isaac.core/omni/isaac/core/simulation_context`: errno=28/No space left on device
2024-08-20 07:19:57 [9,094ms] [Error] [carb] Failed to create change watch for `/home/tk/Desktop/IsaacLab/.conda-env/lib/python3.10/site-packages/isaacsim/exts/omni.isaac.core/omni/isaac/core/objects`: errno=28/No space left on device
2024-08-20 07:19:57 [9,094ms] [Error] [carb] Failed to create change watch for `/home/tk/Desktop/IsaacLab/.conda-env/lib/python3.10/site-packages/isaacsim/exts/omni.isaac.core/omni/isaac/core/scenes`: errno=28/No space left on device
2024-08-20 07:19:57 [9,094ms] [Error] [carb] Failed to create change watch for `/home/tk/Desktop/IsaacLab/.conda-env/lib/python3.10/site-packages/isaacsim/exts/omni.isaac.core/omni/isaac/core/world`: errno=28/No space left on device
```
{% endcode %}



solution:&#x20;

[https://docs.omniverse.nvidia.com/dev-guide/latest/linux-troubleshooting.html#q11-getting-many-failures-similar-to-failed-to-create-change-watch-for-xxx-errno-28-no-space-left-on-device](https://docs.omniverse.nvidia.com/dev-guide/latest/linux-troubleshooting.html#q11-getting-many-failures-similar-to-failed-to-create-change-watch-for-xxx-errno-28-no-space-left-on-device)



## Set up in Headless Mode



```bash
rm _isaac_sim
(base) chiyufeng@a27:/rscratch/tk/Desktop/G1Workspace/IsaacLab$ ln -s /rscratch/tk/Documents/isaac-sim-4.1.0 _isaac_sim
(base) chiyufeng@a27:/rscratch/tk/Desktop/G1Workspace/IsaacLab$ export ISAACSIM_PATH="/rscratch/tk/Documents/isaac-sim-4.1.0"
(base) chiyufeng@a27:/rscratch/tk/Desktop/G1Workspace/IsaacLab$ export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
(base) chiyufeng@a27:/rscratch/tk/Desktop/G1Workspace/IsaacLab$ ${ISAACSIM_PYTHON_EXE} -c "print('Isaac Sim configuration is now complete.')"
```







## Helpful Tips

### Logging Directory

A bunch of omni Python module uses the `omni.log.info()` method to log data.

The logging directory is at

{% code overflow="wrap" %}
```bash
$CONDA_PREFIX/lib/python3.10/site-packages/omni/logs/Kit/Isaac-Sim/4.2/
```
{% endcode %}



### Robot Joint Order

The joint are searched BFS, and joints at the same depth level are ordered alphabetically.

For example, this is the G1 robot joint order

<figure><img src="../../.gitbook/assets/image (242).png" alt=""><figcaption></figcaption></figure>

### File Watch Limit

If running into file watch limit error like the one below:

```bash
[carb] Failed to create change watch for `xxxxx`: errno=28/No space left on device
```



Can increase the system file watch limit by running

```bash
echo fs.inotify.max_user_watches=1048576 | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```



and examine the change:

```bash
cat /proc/sys/fs/inotify/max_user_watches
```



{% embed url="https://dev.to/rubiin/ubuntu-increase-inotify-watcher-file-watch-limit-kf4" %}

