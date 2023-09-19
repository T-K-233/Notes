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



Extract the zip file to folder.





## 2. Set up Conda Environment

Isaac Gym requires Python version <3.9, >=3.6. Here, we will be using Python 3.8.

```bash
conda create --name calm python=3.8
```



```bash
conda activate calm
cd ./python
pip install -e .

```



```bash
export LD_LIBRARY_PATH=/home/tk/Documents/mambaforge/envs/calm/lib/
```

