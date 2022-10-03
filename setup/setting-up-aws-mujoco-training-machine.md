# OpenAI gym + Mujoco Setup

### 1. Create conda environment

```
conda env create --name biped

conda activate biped
```

### 2. Install gym and mujoco



```
pip install gym[mujoco]
```



It's possible that we may also need to install some system packages

```
sudo apt install libosmesa6-dev libgl1-mesa-glx libglfw3
sudo apt install patchelf
```





