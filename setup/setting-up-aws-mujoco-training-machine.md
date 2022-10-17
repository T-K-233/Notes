# OpenAI gym + Mujoco Setup

### 1. Create conda environment

```bash
conda env create --name biped

conda activate biped
```

### 2. Install gym and mujoco



```bash
pip install gym[mujoco]
```



It's possible that we may also need to install some system packages

```bash
sudo apt install libosmesa6-dev libgl1-mesa-glx libglfw3
sudo apt install patchelf
```





