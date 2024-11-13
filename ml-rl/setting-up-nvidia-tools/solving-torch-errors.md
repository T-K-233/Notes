# Solving Torch Errors



## Undefined \`\_\_nvJitLinkComplete\_12\_4\`

### Error

{% code overflow="wrap" %}
```bash
>>> import torch
  File "/home/tk/Documents/mambaforge/envs/torch/lib/python3.10/site-packages/torch/__init__.py", line 367, in <module>
    from torch._C import *  # noqa: F403
ImportError: /home/tk/Documents/mambaforge/envs/depth-pro/lib/python3.10/site-packages/torch/lib/../../nvidia/cusparse/lib/libcusparse.so.12: undefined symbol: __nvJitLinkComplete_12_4, version libnvJitLink.so.12
```
{% endcode %}

### Solution

Add the `nvjtlink` directory of the corresponding python package to link library path.

{% code overflow="wrap" %}
```bash
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib/python3.10/site-packages/nvidia/nvjitlink/lib/:$LD_LIBRARY_PATH
```
{% endcode %}

