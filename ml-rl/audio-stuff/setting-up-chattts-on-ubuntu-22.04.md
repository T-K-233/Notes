# Setting up ChatTTS on Ubuntu 22.04



[https://github.com/2noise/ChatTTS](https://github.com/2noise/ChatTTS)



```bash
git clone https://github.com/2noise/ChatTTS
cd ChatTTS
```



```bash
conda create -yn chattts python=3.10
conda activate chattts
pip install -r requirements.txt
```



by webui

```bash
python examples/web/webui.py
```

by cmd

```bash
python examples/cmd/run.py "Your text 1." "Your text 2."
```













