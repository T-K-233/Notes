# Setting up ExecuTorch on Ubuntu 22.04

Note: system must >= 22.04. Otherwise the GLIBC version will cause error.



Roughly follow tutorial [here](https://pytorch.org/executorch/stable/getting-started-setup.html) and [here](https://pytorch.org/executorch/main/getting-started-setup.html#environment-setup).



## Set up Environment

```bash
git clone --branch v0.1.0 https://github.com/pytorch/executorch.git

cd ./executorch/
git submodule sync
git submodule update --init

conda create -yn executorch python=3.10.0
conda activate executorch

pip install cmake

./install_requirements.sh
```



Likely will encounter the following error when running the last command:

<figure><img src="../.gitbook/assets/image (210).png" alt=""><figcaption></figcaption></figure>

In this case, do the following

```bash
cd ./third-party/flatbuffers/
git checkout v24.3.25
cd ./cmake-out/
make clean
cd ../../../

./build/install_flatc.sh
export PATH="/home/tk/Desktop/executorch/third-party/flatbuffers/cmake-out:${PATH}"
./build/install_flatc.sh
```



<figure><img src="../.gitbook/assets/image (212).png" alt=""><figcaption></figcaption></figure>



## Generate a Sample ExecuTorch program

Create simple program

{% code title="export_add.py" %}
```python
import torch
from torch.export import export
from executorch.exir import to_edge

# Start with a PyTorch model that adds two input tensors (matrices)
class Add(torch.nn.Module):
  def __init__(self):
    super(Add, self).__init__()

  def forward(self, x: torch.Tensor, y: torch.Tensor):
      return x + y

# 1. torch.export: Defines the program with the ATen operator set.
aten_dialect = export(Add(), (torch.ones(1), torch.ones(1)))

# 2. to_edge: Make optimizations for Edge devices
edge_program = to_edge(aten_dialect)

# 3. to_executorch: Convert the graph to an ExecuTorch program
executorch_program = edge_program.to_executorch()

# 4. Save the compiled .pte program
with open("add.pte", "wb") as file:
    file.write(executorch_program.buffer)

print("Exported to add.pte")

```
{% endcode %}



```bash
python3 export_add.py
```





## Building Runtime

```bash
pip3 install zstd
```



```bash
sudo apt install clang
sudo apt install lld
```



buck2 might complain about inode watch file number etc. One possible solution according to [thread](https://github.com/facebook/buck2/issues/471#issuecomment-1822996843) is to use watchman for the file watcher.

```bash
sudo apt install watchman
```

and then edit `./buckconfig`:

{% code title=".buckconfig" %}
```bash
...

[parser]
  target_platform_detector_spec = target:root//...->prelude//platforms:default target:shim//...->prelude//platforms:default

[buck2]
file_watcher = watchman

```
{% endcode %}



Install buck



Download buck2 from [here](https://github.com/facebook/buck2/releases/tag/2023-07-18)

{% code overflow="wrap" %}
```bash
zstd -cdq ~/Downloads/buck2-x86_64-unknown-linux-gnu.zst > ~/Downloads/buck2
sudo mv ~/Downloads/buck2 /usr/bin/buck2
chmod +x /usr/bin/buck2
```
{% endcode %}



Building the program

{% code overflow="wrap" %}
```bash
(rm -rf cmake-out && mkdir cmake-out && cd cmake-out && cmake ..)

cmake --build cmake-out --target executor_runner -j9
```
{% endcode %}



```bash
./cmake-out/executor_runner --model_path add.pte
```















