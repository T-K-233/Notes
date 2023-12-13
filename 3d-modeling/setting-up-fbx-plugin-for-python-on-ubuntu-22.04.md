# Setting up FBX Plugin for Python on Ubuntu 22.04

## 0. Prerequisite

Python == 3.10.X



## 1. Download FBX Python SDK

Go to [https://www.autodesk.com/products/fbx/overview](https://www.autodesk.com/products/fbx/overview). Click the "Get FBX SDK" button.

In the opened page, navigate to "FBX Python SDK" section and download SDK for Linux.



<figure><img src="../.gitbook/assets/image (191).png" alt=""><figcaption></figcaption></figure>



Extract the downloaded file.

<figure><img src="../.gitbook/assets/image (192).png" alt=""><figcaption></figcaption></figure>



Grant executable permission to the binary installer.

<figure><img src="../.gitbook/assets/image (193).png" alt=""><figcaption></figcaption></figure>



## 2. Install

Create a location where the FBX plugin is going to be installed at. In this case, we use `/home/tk/Documents/fbx/`.



Then, run the binary and provide the installation path as the first argument.

```bash
./fbx202034_fbxpythonsdk_linux /home/tk/Documents/fbx/
```

<figure><img src="../.gitbook/assets/image (194).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../.gitbook/assets/image (195).png" alt=""><figcaption></figcaption></figure>



## 3. Install to Python

```bash
cd /home/tk/Documents/fbx/
```

```bash
conda activate urdf
```

```bash
pip install ./fbx-2020.3.4-cp310-cp310-manylinux1_x86_64.whl
```





