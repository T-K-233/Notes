# GPTSoVITS



On A24



```bash
cd /rscratch/tk/Desktop/
git clone https://github.com/RVC-Boss/GPT-SoVITS.git
cd ./GPT-SoVITS/
```



```bash
conda create -p ./.conda-env/ python=3.9
```



```bash
conda activate GPTSoVits
bash install.sh
```



```bash
mkdir -p ./GPT_SoVITS/pretrained_models
mkdir -p ./tools/damo_asr/models
```



```bash
cd ./GPT_SoVITS/pretrained_models
git clone https://huggingface.co/lj1995/GPT-SoVITS
cd ../..
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
mv ./GPT_SoVITS/pretrained_models/GPT-SoVITS/* ./GPT_SoVITS/pretrained_models/

```





