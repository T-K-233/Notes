# GPTSoVITS



On A24



### Clone repo

```bash
cd /rscratch/tk/Desktop/
git clone https://github.com/RVC-Boss/GPT-SoVITS.git
cd ./GPT-SoVITS/
```



### Create Conda environment

```bash
conda create -p ./.conda-env/ python=3.9
```

```bash
conda activate ./.conda-env/

# conda install -c conda-forge gcc
# conda install -c conda-forge gxx
# conda install ffmpeg cmake
# conda install pytorch==2.1.1 torchvision==0.16.1 torchaudio==2.1.1 pytorch-cuda=11.8 -c pytorch -c nvidia
pip install -r requirements.txt
```



### Install required models and tools

```bash
mkdir -p ./GPT_SoVITS/pretrained_models
mkdir -p ./tools/damo_asr/models
```

```bash
cd ./GPT_SoVITS/pretrained_models
git clone https://huggingface.co/lj1995/GPT-SoVITS
cd ../..
mv ./GPT_SoVITS/pretrained_models/GPT-SoVITS/* ./GPT_SoVITS/pretrained_models/
rm -rf ./GPT_SoVITS/pretrained_models/GPT-SoVITS/
```

```bash
cd ./tools/damo_asr/models
git clone https://www.modelscope.cn/damo/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch.git
git clone https://www.modelscope.cn/damo/speech_fsmn_vad_zh-cn-16k-common-pytorch.git
git clone https://www.modelscope.cn/damo/punc_ct-transformer_zh-cn-common-vocab272727-pytorch.git
cd ../../..
```

```bash
cd ./tools/uvr5
git clone https://huggingface.co/Delik/uvr5_weights
git config core.sparseCheckout true
cd ../..
```



optional







### Dataset labeling

#### 0c-Chinese ASR tool

input folder path

```bash
/rscratch/tk/Desktop/GPT-SoVITS/raw/bora
```

output folder path&#x20;

```bash
/rscratch/tk/Desktop/GPT-SoVITS/output/asr_opt
```

ASR model

Faster Whisper - large-v3



#### 0d-Speech to text proofreading tool

.list annotation file path

```bash
/rscratch/tk/Desktop/GPT-SoVITS/output/asr_opt/bora.list
```



### Training

#### 1A-Dataset formatting

Experiment/model name: borav2



\*Text labelling file

```bash
/rscratch/tk/Desktop/GPT-SoVITS/output/asr_opt/bora.list
```

\*Audio dataset folder

```bash
leave empty
```



### Inference



sample

{% code overflow="wrap" %}
```
There is a limited number of registers in every processor. Risk five architecture specifies thirty two registers.
```
{% endcode %}







