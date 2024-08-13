# \[Deprecated] Setting up NVIDIA Isaac Gym on Ubuntu 22.04 / 20.04

{% hint style="warning" %}
**2024-07-20 Update:**

Isaac Gym is deprecated. Please refer to NVIDIA Isaac Lab note: [getting-started-with-nvidia-isaac-lab-on-ubuntu-22.04-24.04.md](getting-started-with-nvidia-isaac-lab-on-ubuntu-22.04-24.04.md "mention")
{% endhint %}



## 1. Download Isaac Gym Installation File

Go to the Isaac Gym website.

{% embed url="https://developer.nvidia.com/isaac-gym" %}

Login using the NVIDIA account. Click "Join now".

<figure><img src="../../.gitbook/assets/image (167).png" alt=""><figcaption></figcaption></figure>



Click "Member area".

<figure><img src="../../.gitbook/assets/image (168).png" alt=""><figcaption></figcaption></figure>

Check the "I Agree To ..." checkbox, and in the expanded section, click the button to download.

<figure><img src="../../.gitbook/assets/image (169).png" alt=""><figcaption></figcaption></figure>



Extract the downloaded "IsaacGym\_Preview\_4\_Package.tar.gz" file. Then, move the folder to a known location.

<figure><img src="../../.gitbook/assets/image (174).png" alt=""><figcaption></figcaption></figure>



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



```bash
pip install protobuf==3.20
```



Create a new file (here we put it under the isaac gym installation directory: `~/Documents/isaacgym/env.sh`) with the following content in the file.

{% code title="env.sh " %}
```bash
# solve libpython3.8.so.1.0 not found issue
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib/:$LD_LIBRARY_PATH

# solve "generated code is out of date and must be regenerated with protoc >= 3.19.0" issue
# export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
```
{% endcode %}



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



<figure><img src="../../.gitbook/assets/image (175).png" alt=""><figcaption></figcaption></figure>

