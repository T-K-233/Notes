# NVIDIA Isaac Gym Setup

## 1. Download Isaac Gym Installation File

Go to the Isaac Gym website.

{% embed url="https://developer.nvidia.com/isaac-gym" %}

Login using the NVIDIA account. Click "Join now".

<figure><img src="../.gitbook/assets/image (167).png" alt=""><figcaption></figcaption></figure>



Click "Member area".

<figure><img src="../.gitbook/assets/image (168).png" alt=""><figcaption></figcaption></figure>

Check the "I Agree To ..." checkbox, and in the expanded section, click the button to download.

<figure><img src="../.gitbook/assets/image (169).png" alt=""><figcaption></figcaption></figure>



Extract the downloaded "IsaacGym\_Preview\_4\_Package.tar.gz" file. Then, move the folder to a known location.

<figure><img src="../.gitbook/assets/image (173).png" alt=""><figcaption></figcaption></figure>



## 2. Set up Conda Environment

Isaac Gym requires Python version <3.9, >=3.6. Here, we will be using Python 3.8.

```bash
conda create --name gym python=3.8
conda activate gym
```

> **Note**: Set the environment name to be descriptive. "gym" here is just an example, and ideally it should be your project name.

```bash
cd <path-to-isaacgym-folder>/python
pip install -e .
```



Create a new file called "env.sh" and write the following content in the file

```bash
# env.sh
export LD_LIBRARY_PATH=/home/tk/Documents/mambaforge/envs/gym/lib/
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
```



## 3. Running Example

On every new terminal, do

```bash
source env.sh
```



To start off, we can run the included examples.

```bash
cd <path-to-isaacgym-folder>/python/examples
```

```bash
python joint_monkey.py --asset_id 0
```





