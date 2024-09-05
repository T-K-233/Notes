# Setting up ExecuTorch on Ubuntu 22.04

Follow PyTorch official tutorial [here](https://pytorch.org/executorch/stable/getting-started-setup.html).



Platform: A12.millennium



## Set up Environment

```bash
mkdir ./executorch-workspace/
```



```bash
cd ./executorch-workspace/
conda create -yp ./.conda-env python=3.10.0
conda activate executorch
```



```bash
git clone --branch v0.3.0 https://github.com/pytorch/executorch.git
cd ./executorch/
```



```bash
git submodule sync
git submodule update --init
```



```bash
./install_requirements.sh
```





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

```
{% endcode %}



```bash
python3 export_add.py
```





## Building Runtime



Building the program

{% code overflow="wrap" %}
```bash
(rm -rf cmake-out && mkdir cmake-out && cd cmake-out && cmake ..)

cmake --build cmake-out --target executor_runner -j9
```
{% endcode %}



```bash
./cmake-out/executor_runner --model_path ./add.pte
```















