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







## Model Architecture



{% code overflow="wrap" %}
```bash
> dict_s2.keys()
dict_keys(['weight', 'config', 'info'])


> dict_s2["weight"].keys()
odict_keys(['enc_p.ssl_proj.weight', 'enc_p.ssl_proj.bias', 'enc_p.encoder_ssl.attn_layers.0.emb_rel_k', 'enc_p.encoder_ssl.attn_layers.0.emb_rel_v', 'enc_p.encoder_ssl.attn_layers.0.conv_q.weight', 'enc_p.encoder_ssl.attn_layers.0.conv_q.bias', 'enc_p.encoder_ssl.attn_layers.0.conv_k.weight', 'enc_p.encoder_ssl.attn_layers.0.conv_k.bias', 'enc_p.encoder_ssl.attn_layers.0.conv_v.weight', 'enc_p.encoder_ssl.attn_layers.0.conv_v.bias', 'enc_p.encoder_ssl.attn_layers.0.conv_o.weight', 'enc_p.encoder_ssl.attn_layers.0.conv_o.bias', 'enc_p.encoder_ssl.attn_layers.1.emb_rel_k', 'enc_p.encoder_ssl.attn_layers.1.emb_rel_v', 'enc_p.encoder_ssl.attn_layers.1.conv_q.weight', 'enc_p.encoder_ssl.attn_layers.1.conv_q.bias', 'enc_p.encoder_ssl.attn_layers.1.conv_k.weight', 'enc_p.encoder_ssl.attn_layers.1.conv_k.bias', 'enc_p.encoder_ssl.attn_layers.1.conv_v.weight', 'enc_p.encoder_ssl.attn_layers.1.conv_v.bias', 'enc_p.encoder_ssl.attn_layers.1.conv_o.weight', 'enc_p.encoder_ssl.attn_layers.1.conv_o.bias', 'enc_p.encoder_ssl.attn_layers.2.emb_rel_k', 'enc_p.encoder_ssl.attn_layers.2.emb_rel_v', 'enc_p.encoder_ssl.attn_layers.2.conv_q.weight', 'enc_p.encoder_ssl.attn_layers.2.conv_q.bias', 'enc_p.encoder_ssl.attn_layers.2.conv_k.weight', 'enc_p.encoder_ssl.attn_layers.2.conv_k.bias', 'enc_p.encoder_ssl.attn_layers.2.conv_v.weight', 'enc_p.encoder_ssl.attn_layers.2.conv_v.bias', 'enc_p.encoder_ssl.attn_layers.2.conv_o.weight', 'enc_p.encoder_ssl.attn_layers.2.conv_o.bias', 'enc_p.encoder_ssl.norm_layers_1.0.gamma', 'enc_p.encoder_ssl.norm_layers_1.0.beta', 'enc_p.encoder_ssl.norm_layers_1.1.gamma', 'enc_p.encoder_ssl.norm_layers_1.1.beta', 'enc_p.encoder_ssl.norm_layers_1.2.gamma', 'enc_p.encoder_ssl.norm_layers_1.2.beta', 'enc_p.encoder_ssl.ffn_layers.0.conv_1.weight', 'enc_p.encoder_ssl.ffn_layers.0.conv_1.bias', 'enc_p.encoder_ssl.ffn_layers.0.conv_2.weight', 'enc_p.encoder_ssl.ffn_layers.0.conv_2.bias', 'enc_p.encoder_ssl.ffn_layers.1.conv_1.weight', 'enc_p.encoder_ssl.ffn_layers.1.conv_1.bias', 'enc_p.encoder_ssl.ffn_layers.1.conv_2.weight', 'enc_p.encoder_ssl.ffn_layers.1.conv_2.bias', 'enc_p.encoder_ssl.ffn_layers.2.conv_1.weight', 'enc_p.encoder_ssl.ffn_layers.2.conv_1.bias', 'enc_p.encoder_ssl.ffn_layers.2.conv_2.weight', 'enc_p.encoder_ssl.ffn_layers.2.conv_2.bias', 'enc_p.encoder_ssl.norm_layers_2.0.gamma', 'enc_p.encoder_ssl.norm_layers_2.0.beta', 'enc_p.encoder_ssl.norm_layers_2.1.gamma', 'enc_p.encoder_ssl.norm_layers_2.1.beta', 'enc_p.encoder_ssl.norm_layers_2.2.gamma', 'enc_p.encoder_ssl.norm_layers_2.2.beta', 'enc_p.encoder_text.attn_layers.0.emb_rel_k', 'enc_p.encoder_text.attn_layers.0.emb_rel_v', 'enc_p.encoder_text.attn_layers.0.conv_q.weight', 'enc_p.encoder_text.attn_layers.0.conv_q.bias', 'enc_p.encoder_text.attn_layers.0.conv_k.weight', 'enc_p.encoder_text.attn_layers.0.conv_k.bias', 'enc_p.encoder_text.attn_layers.0.conv_v.weight', 'enc_p.encoder_text.attn_layers.0.conv_v.bias', 'enc_p.encoder_text.attn_layers.0.conv_o.weight', 'enc_p.encoder_text.attn_layers.0.conv_o.bias', 'enc_p.encoder_text.attn_layers.1.emb_rel_k', 'enc_p.encoder_text.attn_layers.1.emb_rel_v', 'enc_p.encoder_text.attn_layers.1.conv_q.weight', 'enc_p.encoder_text.attn_layers.1.conv_q.bias', 'enc_p.encoder_text.attn_layers.1.conv_k.weight', 'enc_p.encoder_text.attn_layers.1.conv_k.bias', 'enc_p.encoder_text.attn_layers.1.conv_v.weight', 'enc_p.encoder_text.attn_layers.1.conv_v.bias', 'enc_p.encoder_text.attn_layers.1.conv_o.weight', 'enc_p.encoder_text.attn_layers.1.conv_o.bias', 'enc_p.encoder_text.attn_layers.2.emb_rel_k', 'enc_p.encoder_text.attn_layers.2.emb_rel_v', 'enc_p.encoder_text.attn_layers.2.conv_q.weight', 'enc_p.encoder_text.attn_layers.2.conv_q.bias', 'enc_p.encoder_text.attn_layers.2.conv_k.weight', 'enc_p.encoder_text.attn_layers.2.conv_k.bias', 'enc_p.encoder_text.attn_layers.2.conv_v.weight', 'enc_p.encoder_text.attn_layers.2.conv_v.bias', 'enc_p.encoder_text.attn_layers.2.conv_o.weight', 'enc_p.encoder_text.attn_layers.2.conv_o.bias', 'enc_p.encoder_text.attn_layers.3.emb_rel_k', 'enc_p.encoder_text.attn_layers.3.emb_rel_v', 'enc_p.encoder_text.attn_layers.3.conv_q.weight', 'enc_p.encoder_text.attn_layers.3.conv_q.bias', 'enc_p.encoder_text.attn_layers.3.conv_k.weight', 'enc_p.encoder_text.attn_layers.3.conv_k.bias', 'enc_p.encoder_text.attn_layers.3.conv_v.weight', 'enc_p.encoder_text.attn_layers.3.conv_v.bias', 'enc_p.encoder_text.attn_layers.3.conv_o.weight', 'enc_p.encoder_text.attn_layers.3.conv_o.bias', 'enc_p.encoder_text.attn_layers.4.emb_rel_k', 'enc_p.encoder_text.attn_layers.4.emb_rel_v', 'enc_p.encoder_text.attn_layers.4.conv_q.weight', 'enc_p.encoder_text.attn_layers.4.conv_q.bias', 'enc_p.encoder_text.attn_layers.4.conv_k.weight', 'enc_p.encoder_text.attn_layers.4.conv_k.bias', 'enc_p.encoder_text.attn_layers.4.conv_v.weight', 'enc_p.encoder_text.attn_layers.4.conv_v.bias', 'enc_p.encoder_text.attn_layers.4.conv_o.weight', 'enc_p.encoder_text.attn_layers.4.conv_o.bias', 'enc_p.encoder_text.attn_layers.5.emb_rel_k', 'enc_p.encoder_text.attn_layers.5.emb_rel_v', 'enc_p.encoder_text.attn_layers.5.conv_q.weight', 'enc_p.encoder_text.attn_layers.5.conv_q.bias', 'enc_p.encoder_text.attn_layers.5.conv_k.weight', 'enc_p.encoder_text.attn_layers.5.conv_k.bias', 'enc_p.encoder_text.attn_layers.5.conv_v.weight', 'enc_p.encoder_text.attn_layers.5.conv_v.bias', 'enc_p.encoder_text.attn_layers.5.conv_o.weight', 'enc_p.encoder_text.attn_layers.5.conv_o.bias', 'enc_p.encoder_text.norm_layers_1.0.gamma', 'enc_p.encoder_text.norm_layers_1.0.beta', 'enc_p.encoder_text.norm_layers_1.1.gamma', 'enc_p.encoder_text.norm_layers_1.1.beta', 'enc_p.encoder_text.norm_layers_1.2.gamma', 'enc_p.encoder_text.norm_layers_1.2.beta', 'enc_p.encoder_text.norm_layers_1.3.gamma', 'enc_p.encoder_text.norm_layers_1.3.beta', 'enc_p.encoder_text.norm_layers_1.4.gamma', 'enc_p.encoder_text.norm_layers_1.4.beta', 'enc_p.encoder_text.norm_layers_1.5.gamma', 'enc_p.encoder_text.norm_layers_1.5.beta', 'enc_p.encoder_text.ffn_layers.0.conv_1.weight', 'enc_p.encoder_text.ffn_layers.0.conv_1.bias', 'enc_p.encoder_text.ffn_layers.0.conv_2.weight', 'enc_p.encoder_text.ffn_layers.0.conv_2.bias', 'enc_p.encoder_text.ffn_layers.1.conv_1.weight', 'enc_p.encoder_text.ffn_layers.1.conv_1.bias', 'enc_p.encoder_text.ffn_layers.1.conv_2.weight', 'enc_p.encoder_text.ffn_layers.1.conv_2.bias', 'enc_p.encoder_text.ffn_layers.2.conv_1.weight', 'enc_p.encoder_text.ffn_layers.2.conv_1.bias', 'enc_p.encoder_text.ffn_layers.2.conv_2.weight', 'enc_p.encoder_text.ffn_layers.2.conv_2.bias', 'enc_p.encoder_text.ffn_layers.3.conv_1.weight', 'enc_p.encoder_text.ffn_layers.3.conv_1.bias', 'enc_p.encoder_text.ffn_layers.3.conv_2.weight', 'enc_p.encoder_text.ffn_layers.3.conv_2.bias', 'enc_p.encoder_text.ffn_layers.4.conv_1.weight', 'enc_p.encoder_text.ffn_layers.4.conv_1.bias', 'enc_p.encoder_text.ffn_layers.4.conv_2.weight', 'enc_p.encoder_text.ffn_layers.4.conv_2.bias', 'enc_p.encoder_text.ffn_layers.5.conv_1.weight', 'enc_p.encoder_text.ffn_layers.5.conv_1.bias', 'enc_p.encoder_text.ffn_layers.5.conv_2.weight', 'enc_p.encoder_text.ffn_layers.5.conv_2.bias', 'enc_p.encoder_text.norm_layers_2.0.gamma', 'enc_p.encoder_text.norm_layers_2.0.beta', 'enc_p.encoder_text.norm_layers_2.1.gamma', 'enc_p.encoder_text.norm_layers_2.1.beta', 'enc_p.encoder_text.norm_layers_2.2.gamma', 'enc_p.encoder_text.norm_layers_2.2.beta', 'enc_p.encoder_text.norm_layers_2.3.gamma', 'enc_p.encoder_text.norm_layers_2.3.beta', 'enc_p.encoder_text.norm_layers_2.4.gamma', 'enc_p.encoder_text.norm_layers_2.4.beta', 'enc_p.encoder_text.norm_layers_2.5.gamma', 'enc_p.encoder_text.norm_layers_2.5.beta', 'enc_p.text_embedding.weight', 'enc_p.mrte.cross_attention.conv_q.weight', 'enc_p.mrte.cross_attention.conv_q.bias', 'enc_p.mrte.cross_attention.conv_k.weight', 'enc_p.mrte.cross_attention.conv_k.bias', 'enc_p.mrte.cross_attention.conv_v.weight', 'enc_p.mrte.cross_attention.conv_v.bias', 'enc_p.mrte.cross_attention.conv_o.weight', 'enc_p.mrte.cross_attention.conv_o.bias', 'enc_p.mrte.c_pre.weight', 'enc_p.mrte.c_pre.bias', 'enc_p.mrte.text_pre.weight', 'enc_p.mrte.text_pre.bias', 'enc_p.mrte.c_post.weight', 'enc_p.mrte.c_post.bias', 'enc_p.encoder2.attn_layers.0.emb_rel_k', 'enc_p.encoder2.attn_layers.0.emb_rel_v', 'enc_p.encoder2.attn_layers.0.conv_q.weight', 'enc_p.encoder2.attn_layers.0.conv_q.bias', 'enc_p.encoder2.attn_layers.0.conv_k.weight', 'enc_p.encoder2.attn_layers.0.conv_k.bias', 'enc_p.encoder2.attn_layers.0.conv_v.weight', 'enc_p.encoder2.attn_layers.0.conv_v.bias', 'enc_p.encoder2.attn_layers.0.conv_o.weight', 'enc_p.encoder2.attn_layers.0.conv_o.bias', 'enc_p.encoder2.attn_layers.1.emb_rel_k', 'enc_p.encoder2.attn_layers.1.emb_rel_v', 'enc_p.encoder2.attn_layers.1.conv_q.weight', 'enc_p.encoder2.attn_layers.1.conv_q.bias', 'enc_p.encoder2.attn_layers.1.conv_k.weight', 'enc_p.encoder2.attn_layers.1.conv_k.bias', 'enc_p.encoder2.attn_layers.1.conv_v.weight', 'enc_p.encoder2.attn_layers.1.conv_v.bias', 'enc_p.encoder2.attn_layers.1.conv_o.weight', 'enc_p.encoder2.attn_layers.1.conv_o.bias', 'enc_p.encoder2.attn_layers.2.emb_rel_k', 'enc_p.encoder2.attn_layers.2.emb_rel_v', 'enc_p.encoder2.attn_layers.2.conv_q.weight', 'enc_p.encoder2.attn_layers.2.conv_q.bias', 'enc_p.encoder2.attn_layers.2.conv_k.weight', 'enc_p.encoder2.attn_layers.2.conv_k.bias', 'enc_p.encoder2.attn_layers.2.conv_v.weight', 'enc_p.encoder2.attn_layers.2.conv_v.bias', 'enc_p.encoder2.attn_layers.2.conv_o.weight', 'enc_p.encoder2.attn_layers.2.conv_o.bias', 'enc_p.encoder2.norm_layers_1.0.gamma', 'enc_p.encoder2.norm_layers_1.0.beta', 'enc_p.encoder2.norm_layers_1.1.gamma', 'enc_p.encoder2.norm_layers_1.1.beta', 'enc_p.encoder2.norm_layers_1.2.gamma', 'enc_p.encoder2.norm_layers_1.2.beta', 'enc_p.encoder2.ffn_layers.0.conv_1.weight', 'enc_p.encoder2.ffn_layers.0.conv_1.bias', 'enc_p.encoder2.ffn_layers.0.conv_2.weight', 'enc_p.encoder2.ffn_layers.0.conv_2.bias', 'enc_p.encoder2.ffn_layers.1.conv_1.weight', 'enc_p.encoder2.ffn_layers.1.conv_1.bias', 'enc_p.encoder2.ffn_layers.1.conv_2.weight', 'enc_p.encoder2.ffn_layers.1.conv_2.bias', 'enc_p.encoder2.ffn_layers.2.conv_1.weight', 'enc_p.encoder2.ffn_layers.2.conv_1.bias', 'enc_p.encoder2.ffn_layers.2.conv_2.weight', 'enc_p.encoder2.ffn_layers.2.conv_2.bias', 'enc_p.encoder2.norm_layers_2.0.gamma', 'enc_p.encoder2.norm_layers_2.0.beta', 'enc_p.encoder2.norm_layers_2.1.gamma', 'enc_p.encoder2.norm_layers_2.1.beta', 'enc_p.encoder2.norm_layers_2.2.gamma', 'enc_p.encoder2.norm_layers_2.2.beta', 'enc_p.proj.weight', 'enc_p.proj.bias', 'dec.conv_pre.weight', 'dec.conv_pre.bias', 'dec.ups.0.bias', 'dec.ups.0.weight_g', 'dec.ups.0.weight_v', 'dec.ups.1.bias', 'dec.ups.1.weight_g', 'dec.ups.1.weight_v', 'dec.ups.2.bias', 'dec.ups.2.weight_g', 'dec.ups.2.weight_v', 'dec.ups.3.bias', 'dec.ups.3.weight_g', 'dec.ups.3.weight_v', 'dec.ups.4.bias', 'dec.ups.4.weight_g', 'dec.ups.4.weight_v', 'dec.resblocks.0.convs1.0.bias', 'dec.resblocks.0.convs1.0.weight_g', 'dec.resblocks.0.convs1.0.weight_v', 'dec.resblocks.0.convs1.1.bias', 'dec.resblocks.0.convs1.1.weight_g', 'dec.resblocks.0.convs1.1.weight_v', 'dec.resblocks.0.convs1.2.bias', 'dec.resblocks.0.convs1.2.weight_g', 'dec.resblocks.0.convs1.2.weight_v', 'dec.resblocks.0.convs2.0.bias', 'dec.resblocks.0.convs2.0.weight_g', 'dec.resblocks.0.convs2.0.weight_v', 'dec.resblocks.0.convs2.1.bias', 'dec.resblocks.0.convs2.1.weight_g', 'dec.resblocks.0.convs2.1.weight_v', 'dec.resblocks.0.convs2.2.bias', 'dec.resblocks.0.convs2.2.weight_g', 'dec.resblocks.0.convs2.2.weight_v', 'dec.resblocks.1.convs1.0.bias', 'dec.resblocks.1.convs1.0.weight_g', 'dec.resblocks.1.convs1.0.weight_v', 'dec.resblocks.1.convs1.1.bias', 'dec.resblocks.1.convs1.1.weight_g', 'dec.resblocks.1.convs1.1.weight_v', 'dec.resblocks.1.convs1.2.bias', 'dec.resblocks.1.convs1.2.weight_g', 'dec.resblocks.1.convs1.2.weight_v', 'dec.resblocks.1.convs2.0.bias', 'dec.resblocks.1.convs2.0.weight_g', 'dec.resblocks.1.convs2.0.weight_v', 'dec.resblocks.1.convs2.1.bias', 'dec.resblocks.1.convs2.1.weight_g', 'dec.resblocks.1.convs2.1.weight_v', 'dec.resblocks.1.convs2.2.bias', 'dec.resblocks.1.convs2.2.weight_g', 'dec.resblocks.1.convs2.2.weight_v', 'dec.resblocks.2.convs1.0.bias', 'dec.resblocks.2.convs1.0.weight_g', 'dec.resblocks.2.convs1.0.weight_v', 'dec.resblocks.2.convs1.1.bias', 'dec.resblocks.2.convs1.1.weight_g', 'dec.resblocks.2.convs1.1.weight_v', 'dec.resblocks.2.convs1.2.bias', 'dec.resblocks.2.convs1.2.weight_g', 'dec.resblocks.2.convs1.2.weight_v', 'dec.resblocks.2.convs2.0.bias', 'dec.resblocks.2.convs2.0.weight_g', 'dec.resblocks.2.convs2.0.weight_v', 'dec.resblocks.2.convs2.1.bias', 'dec.resblocks.2.convs2.1.weight_g', 'dec.resblocks.2.convs2.1.weight_v', 'dec.resblocks.2.convs2.2.bias', 'dec.resblocks.2.convs2.2.weight_g', 'dec.resblocks.2.convs2.2.weight_v', 'dec.resblocks.3.convs1.0.bias', 'dec.resblocks.3.convs1.0.weight_g', 'dec.resblocks.3.convs1.0.weight_v', 'dec.resblocks.3.convs1.1.bias', 'dec.resblocks.3.convs1.1.weight_g', 'dec.resblocks.3.convs1.1.weight_v', 'dec.resblocks.3.convs1.2.bias', 'dec.resblocks.3.convs1.2.weight_g', 'dec.resblocks.3.convs1.2.weight_v', 'dec.resblocks.3.convs2.0.bias', 'dec.resblocks.3.convs2.0.weight_g', 'dec.resblocks.3.convs2.0.weight_v', 'dec.resblocks.3.convs2.1.bias', 'dec.resblocks.3.convs2.1.weight_g', 'dec.resblocks.3.convs2.1.weight_v', 'dec.resblocks.3.convs2.2.bias', 'dec.resblocks.3.convs2.2.weight_g', 'dec.resblocks.3.convs2.2.weight_v', 'dec.resblocks.4.convs1.0.bias', 'dec.resblocks.4.convs1.0.weight_g', 'dec.resblocks.4.convs1.0.weight_v', 'dec.resblocks.4.convs1.1.bias', 'dec.resblocks.4.convs1.1.weight_g', 'dec.resblocks.4.convs1.1.weight_v', 'dec.resblocks.4.convs1.2.bias', 'dec.resblocks.4.convs1.2.weight_g', 'dec.resblocks.4.convs1.2.weight_v', 'dec.resblocks.4.convs2.0.bias', 'dec.resblocks.4.convs2.0.weight_g', 'dec.resblocks.4.convs2.0.weight_v', 'dec.resblocks.4.convs2.1.bias', 'dec.resblocks.4.convs2.1.weight_g', 'dec.resblocks.4.convs2.1.weight_v', 'dec.resblocks.4.convs2.2.bias', 'dec.resblocks.4.convs2.2.weight_g', 'dec.resblocks.4.convs2.2.weight_v', 'dec.resblocks.5.convs1.0.bias', 'dec.resblocks.5.convs1.0.weight_g', 'dec.resblocks.5.convs1.0.weight_v', 'dec.resblocks.5.convs1.1.bias', 'dec.resblocks.5.convs1.1.weight_g', 'dec.resblocks.5.convs1.1.weight_v', 'dec.resblocks.5.convs1.2.bias', 'dec.resblocks.5.convs1.2.weight_g', 'dec.resblocks.5.convs1.2.weight_v', 'dec.resblocks.5.convs2.0.bias', 'dec.resblocks.5.convs2.0.weight_g', 'dec.resblocks.5.convs2.0.weight_v', 'dec.resblocks.5.convs2.1.bias', 'dec.resblocks.5.convs2.1.weight_g', 'dec.resblocks.5.convs2.1.weight_v', 'dec.resblocks.5.convs2.2.bias', 'dec.resblocks.5.convs2.2.weight_g', 'dec.resblocks.5.convs2.2.weight_v', 'dec.resblocks.6.convs1.0.bias', 'dec.resblocks.6.convs1.0.weight_g', 'dec.resblocks.6.convs1.0.weight_v', 'dec.resblocks.6.convs1.1.bias', 'dec.resblocks.6.convs1.1.weight_g', 'dec.resblocks.6.convs1.1.weight_v', 'dec.resblocks.6.convs1.2.bias', 'dec.resblocks.6.convs1.2.weight_g', 'dec.resblocks.6.convs1.2.weight_v', 'dec.resblocks.6.convs2.0.bias', 'dec.resblocks.6.convs2.0.weight_g', 'dec.resblocks.6.convs2.0.weight_v', 'dec.resblocks.6.convs2.1.bias', 'dec.resblocks.6.convs2.1.weight_g', 'dec.resblocks.6.convs2.1.weight_v', 'dec.resblocks.6.convs2.2.bias', 'dec.resblocks.6.convs2.2.weight_g', 'dec.resblocks.6.convs2.2.weight_v', 'dec.resblocks.7.convs1.0.bias', 'dec.resblocks.7.convs1.0.weight_g', 'dec.resblocks.7.convs1.0.weight_v', 'dec.resblocks.7.convs1.1.bias', 'dec.resblocks.7.convs1.1.weight_g', 'dec.resblocks.7.convs1.1.weight_v', 'dec.resblocks.7.convs1.2.bias', 'dec.resblocks.7.convs1.2.weight_g', 'dec.resblocks.7.convs1.2.weight_v', 'dec.resblocks.7.convs2.0.bias', 'dec.resblocks.7.convs2.0.weight_g', 'dec.resblocks.7.convs2.0.weight_v', 'dec.resblocks.7.convs2.1.bias', 'dec.resblocks.7.convs2.1.weight_g', 'dec.resblocks.7.convs2.1.weight_v', 'dec.resblocks.7.convs2.2.bias', 'dec.resblocks.7.convs2.2.weight_g', 'dec.resblocks.7.convs2.2.weight_v', 'dec.resblocks.8.convs1.0.bias', 'dec.resblocks.8.convs1.0.weight_g', 'dec.resblocks.8.convs1.0.weight_v', 'dec.resblocks.8.convs1.1.bias', 'dec.resblocks.8.convs1.1.weight_g', 'dec.resblocks.8.convs1.1.weight_v', 'dec.resblocks.8.convs1.2.bias', 'dec.resblocks.8.convs1.2.weight_g', 'dec.resblocks.8.convs1.2.weight_v', 'dec.resblocks.8.convs2.0.bias', 'dec.resblocks.8.convs2.0.weight_g', 'dec.resblocks.8.convs2.0.weight_v', 'dec.resblocks.8.convs2.1.bias', 'dec.resblocks.8.convs2.1.weight_g', 'dec.resblocks.8.convs2.1.weight_v', 'dec.resblocks.8.convs2.2.bias', 'dec.resblocks.8.convs2.2.weight_g', 'dec.resblocks.8.convs2.2.weight_v', 'dec.resblocks.9.convs1.0.bias', 'dec.resblocks.9.convs1.0.weight_g', 'dec.resblocks.9.convs1.0.weight_v', 'dec.resblocks.9.convs1.1.bias', 'dec.resblocks.9.convs1.1.weight_g', 'dec.resblocks.9.convs1.1.weight_v', 'dec.resblocks.9.convs1.2.bias', 'dec.resblocks.9.convs1.2.weight_g', 'dec.resblocks.9.convs1.2.weight_v', 'dec.resblocks.9.convs2.0.bias', 'dec.resblocks.9.convs2.0.weight_g', 'dec.resblocks.9.convs2.0.weight_v', 'dec.resblocks.9.convs2.1.bias', 'dec.resblocks.9.convs2.1.weight_g', 'dec.resblocks.9.convs2.1.weight_v', 'dec.resblocks.9.convs2.2.bias', 'dec.resblocks.9.convs2.2.weight_g', 'dec.resblocks.9.convs2.2.weight_v', 'dec.resblocks.10.convs1.0.bias', 'dec.resblocks.10.convs1.0.weight_g', 'dec.resblocks.10.convs1.0.weight_v', 'dec.resblocks.10.convs1.1.bias', 'dec.resblocks.10.convs1.1.weight_g', 'dec.resblocks.10.convs1.1.weight_v', 'dec.resblocks.10.convs1.2.bias', 'dec.resblocks.10.convs1.2.weight_g', 'dec.resblocks.10.convs1.2.weight_v', 'dec.resblocks.10.convs2.0.bias', 'dec.resblocks.10.convs2.0.weight_g', 'dec.resblocks.10.convs2.0.weight_v', 'dec.resblocks.10.convs2.1.bias', 'dec.resblocks.10.convs2.1.weight_g', 'dec.resblocks.10.convs2.1.weight_v', 'dec.resblocks.10.convs2.2.bias', 'dec.resblocks.10.convs2.2.weight_g', 'dec.resblocks.10.convs2.2.weight_v', 'dec.resblocks.11.convs1.0.bias', 'dec.resblocks.11.convs1.0.weight_g', 'dec.resblocks.11.convs1.0.weight_v', 'dec.resblocks.11.convs1.1.bias', 'dec.resblocks.11.convs1.1.weight_g', 'dec.resblocks.11.convs1.1.weight_v', 'dec.resblocks.11.convs1.2.bias', 'dec.resblocks.11.convs1.2.weight_g', 'dec.resblocks.11.convs1.2.weight_v', 'dec.resblocks.11.convs2.0.bias', 'dec.resblocks.11.convs2.0.weight_g', 'dec.resblocks.11.convs2.0.weight_v', 'dec.resblocks.11.convs2.1.bias', 'dec.resblocks.11.convs2.1.weight_g', 'dec.resblocks.11.convs2.1.weight_v', 'dec.resblocks.11.convs2.2.bias', 'dec.resblocks.11.convs2.2.weight_g', 'dec.resblocks.11.convs2.2.weight_v', 'dec.resblocks.12.convs1.0.bias', 'dec.resblocks.12.convs1.0.weight_g', 'dec.resblocks.12.convs1.0.weight_v', 'dec.resblocks.12.convs1.1.bias', 'dec.resblocks.12.convs1.1.weight_g', 'dec.resblocks.12.convs1.1.weight_v', 'dec.resblocks.12.convs1.2.bias', 'dec.resblocks.12.convs1.2.weight_g', 'dec.resblocks.12.convs1.2.weight_v', 'dec.resblocks.12.convs2.0.bias', 'dec.resblocks.12.convs2.0.weight_g', 'dec.resblocks.12.convs2.0.weight_v', 'dec.resblocks.12.convs2.1.bias', 'dec.resblocks.12.convs2.1.weight_g', 'dec.resblocks.12.convs2.1.weight_v', 'dec.resblocks.12.convs2.2.bias', 'dec.resblocks.12.convs2.2.weight_g', 'dec.resblocks.12.convs2.2.weight_v', 'dec.resblocks.13.convs1.0.bias', 'dec.resblocks.13.convs1.0.weight_g', 'dec.resblocks.13.convs1.0.weight_v', 'dec.resblocks.13.convs1.1.bias', 'dec.resblocks.13.convs1.1.weight_g', 'dec.resblocks.13.convs1.1.weight_v', 'dec.resblocks.13.convs1.2.bias', 'dec.resblocks.13.convs1.2.weight_g', 'dec.resblocks.13.convs1.2.weight_v', 'dec.resblocks.13.convs2.0.bias', 'dec.resblocks.13.convs2.0.weight_g', 'dec.resblocks.13.convs2.0.weight_v', 'dec.resblocks.13.convs2.1.bias', 'dec.resblocks.13.convs2.1.weight_g', 'dec.resblocks.13.convs2.1.weight_v', 'dec.resblocks.13.convs2.2.bias', 'dec.resblocks.13.convs2.2.weight_g', 'dec.resblocks.13.convs2.2.weight_v', 'dec.resblocks.14.convs1.0.bias', 'dec.resblocks.14.convs1.0.weight_g', 'dec.resblocks.14.convs1.0.weight_v', 'dec.resblocks.14.convs1.1.bias', 'dec.resblocks.14.convs1.1.weight_g', 'dec.resblocks.14.convs1.1.weight_v', 'dec.resblocks.14.convs1.2.bias', 'dec.resblocks.14.convs1.2.weight_g', 'dec.resblocks.14.convs1.2.weight_v', 'dec.resblocks.14.convs2.0.bias', 'dec.resblocks.14.convs2.0.weight_g', 'dec.resblocks.14.convs2.0.weight_v', 'dec.resblocks.14.convs2.1.bias', 'dec.resblocks.14.convs2.1.weight_g', 'dec.resblocks.14.convs2.1.weight_v', 'dec.resblocks.14.convs2.2.bias', 'dec.resblocks.14.convs2.2.weight_g', 'dec.resblocks.14.convs2.2.weight_v', 'dec.conv_post.weight', 'dec.cond.weight', 'dec.cond.bias', 'enc_q.pre.weight', 'enc_q.pre.bias', 'enc_q.enc.in_layers.0.bias', 'enc_q.enc.in_layers.0.weight_g', 'enc_q.enc.in_layers.0.weight_v', 'enc_q.enc.in_layers.1.bias', 'enc_q.enc.in_layers.1.weight_g', 'enc_q.enc.in_layers.1.weight_v', 'enc_q.enc.in_layers.2.bias', 'enc_q.enc.in_layers.2.weight_g', 'enc_q.enc.in_layers.2.weight_v', 'enc_q.enc.in_layers.3.bias', 'enc_q.enc.in_layers.3.weight_g', 'enc_q.enc.in_layers.3.weight_v', 'enc_q.enc.in_layers.4.bias', 'enc_q.enc.in_layers.4.weight_g', 'enc_q.enc.in_layers.4.weight_v', 'enc_q.enc.in_layers.5.bias', 'enc_q.enc.in_layers.5.weight_g', 'enc_q.enc.in_layers.5.weight_v', 'enc_q.enc.in_layers.6.bias', 'enc_q.enc.in_layers.6.weight_g', 'enc_q.enc.in_layers.6.weight_v', 'enc_q.enc.in_layers.7.bias', 'enc_q.enc.in_layers.7.weight_g', 'enc_q.enc.in_layers.7.weight_v', 'enc_q.enc.in_layers.8.bias', 'enc_q.enc.in_layers.8.weight_g', 'enc_q.enc.in_layers.8.weight_v', 'enc_q.enc.in_layers.9.bias', 'enc_q.enc.in_layers.9.weight_g', 'enc_q.enc.in_layers.9.weight_v', 'enc_q.enc.in_layers.10.bias', 'enc_q.enc.in_layers.10.weight_g', 'enc_q.enc.in_layers.10.weight_v', 'enc_q.enc.in_layers.11.bias', 'enc_q.enc.in_layers.11.weight_g', 'enc_q.enc.in_layers.11.weight_v', 'enc_q.enc.in_layers.12.bias', 'enc_q.enc.in_layers.12.weight_g', 'enc_q.enc.in_layers.12.weight_v', 'enc_q.enc.in_layers.13.bias', 'enc_q.enc.in_layers.13.weight_g', 'enc_q.enc.in_layers.13.weight_v', 'enc_q.enc.in_layers.14.bias', 'enc_q.enc.in_layers.14.weight_g', 'enc_q.enc.in_layers.14.weight_v', 'enc_q.enc.in_layers.15.bias', 'enc_q.enc.in_layers.15.weight_g', 'enc_q.enc.in_layers.15.weight_v', 'enc_q.enc.res_skip_layers.0.bias', 'enc_q.enc.res_skip_layers.0.weight_g', 'enc_q.enc.res_skip_layers.0.weight_v', 'enc_q.enc.res_skip_layers.1.bias', 'enc_q.enc.res_skip_layers.1.weight_g', 'enc_q.enc.res_skip_layers.1.weight_v', 'enc_q.enc.res_skip_layers.2.bias', 'enc_q.enc.res_skip_layers.2.weight_g', 'enc_q.enc.res_skip_layers.2.weight_v', 'enc_q.enc.res_skip_layers.3.bias', 'enc_q.enc.res_skip_layers.3.weight_g', 'enc_q.enc.res_skip_layers.3.weight_v', 'enc_q.enc.res_skip_layers.4.bias', 'enc_q.enc.res_skip_layers.4.weight_g', 'enc_q.enc.res_skip_layers.4.weight_v', 'enc_q.enc.res_skip_layers.5.bias', 'enc_q.enc.res_skip_layers.5.weight_g', 'enc_q.enc.res_skip_layers.5.weight_v', 'enc_q.enc.res_skip_layers.6.bias', 'enc_q.enc.res_skip_layers.6.weight_g', 'enc_q.enc.res_skip_layers.6.weight_v', 'enc_q.enc.res_skip_layers.7.bias', 'enc_q.enc.res_skip_layers.7.weight_g', 'enc_q.enc.res_skip_layers.7.weight_v', 'enc_q.enc.res_skip_layers.8.bias', 'enc_q.enc.res_skip_layers.8.weight_g', 'enc_q.enc.res_skip_layers.8.weight_v', 'enc_q.enc.res_skip_layers.9.bias', 'enc_q.enc.res_skip_layers.9.weight_g', 'enc_q.enc.res_skip_layers.9.weight_v', 'enc_q.enc.res_skip_layers.10.bias', 'enc_q.enc.res_skip_layers.10.weight_g', 'enc_q.enc.res_skip_layers.10.weight_v', 'enc_q.enc.res_skip_layers.11.bias', 'enc_q.enc.res_skip_layers.11.weight_g', 'enc_q.enc.res_skip_layers.11.weight_v', 'enc_q.enc.res_skip_layers.12.bias', 'enc_q.enc.res_skip_layers.12.weight_g', 'enc_q.enc.res_skip_layers.12.weight_v', 'enc_q.enc.res_skip_layers.13.bias', 'enc_q.enc.res_skip_layers.13.weight_g', 'enc_q.enc.res_skip_layers.13.weight_v', 'enc_q.enc.res_skip_layers.14.bias', 'enc_q.enc.res_skip_layers.14.weight_g', 'enc_q.enc.res_skip_layers.14.weight_v', 'enc_q.enc.res_skip_layers.15.bias', 'enc_q.enc.res_skip_layers.15.weight_g', 'enc_q.enc.res_skip_layers.15.weight_v', 'enc_q.enc.cond_layer.bias', 'enc_q.enc.cond_layer.weight_g', 'enc_q.enc.cond_layer.weight_v', 'enc_q.proj.weight', 'enc_q.proj.bias', 'flow.flows.0.pre.weight', 'flow.flows.0.pre.bias', 'flow.flows.0.enc.in_layers.0.bias', 'flow.flows.0.enc.in_layers.0.weight_g', 'flow.flows.0.enc.in_layers.0.weight_v', 'flow.flows.0.enc.in_layers.1.bias', 'flow.flows.0.enc.in_layers.1.weight_g', 'flow.flows.0.enc.in_layers.1.weight_v', 'flow.flows.0.enc.in_layers.2.bias', 'flow.flows.0.enc.in_layers.2.weight_g', 'flow.flows.0.enc.in_layers.2.weight_v', 'flow.flows.0.enc.in_layers.3.bias', 'flow.flows.0.enc.in_layers.3.weight_g', 'flow.flows.0.enc.in_layers.3.weight_v', 'flow.flows.0.enc.res_skip_layers.0.bias', 'flow.flows.0.enc.res_skip_layers.0.weight_g', 'flow.flows.0.enc.res_skip_layers.0.weight_v', 'flow.flows.0.enc.res_skip_layers.1.bias', 'flow.flows.0.enc.res_skip_layers.1.weight_g', 'flow.flows.0.enc.res_skip_layers.1.weight_v', 'flow.flows.0.enc.res_skip_layers.2.bias', 'flow.flows.0.enc.res_skip_layers.2.weight_g', 'flow.flows.0.enc.res_skip_layers.2.weight_v', 'flow.flows.0.enc.res_skip_layers.3.bias', 'flow.flows.0.enc.res_skip_layers.3.weight_g', 'flow.flows.0.enc.res_skip_layers.3.weight_v', 'flow.flows.0.enc.cond_layer.bias', 'flow.flows.0.enc.cond_layer.weight_g', 'flow.flows.0.enc.cond_layer.weight_v', 'flow.flows.0.post.weight', 'flow.flows.0.post.bias', 'flow.flows.2.pre.weight', 'flow.flows.2.pre.bias', 'flow.flows.2.enc.in_layers.0.bias', 'flow.flows.2.enc.in_layers.0.weight_g', 'flow.flows.2.enc.in_layers.0.weight_v', 'flow.flows.2.enc.in_layers.1.bias', 'flow.flows.2.enc.in_layers.1.weight_g', 'flow.flows.2.enc.in_layers.1.weight_v', 'flow.flows.2.enc.in_layers.2.bias', 'flow.flows.2.enc.in_layers.2.weight_g', 'flow.flows.2.enc.in_layers.2.weight_v', 'flow.flows.2.enc.in_layers.3.bias', 'flow.flows.2.enc.in_layers.3.weight_g', 'flow.flows.2.enc.in_layers.3.weight_v', 'flow.flows.2.enc.res_skip_layers.0.bias', 'flow.flows.2.enc.res_skip_layers.0.weight_g', 'flow.flows.2.enc.res_skip_layers.0.weight_v', 'flow.flows.2.enc.res_skip_layers.1.bias', 'flow.flows.2.enc.res_skip_layers.1.weight_g', 'flow.flows.2.enc.res_skip_layers.1.weight_v', 'flow.flows.2.enc.res_skip_layers.2.bias', 'flow.flows.2.enc.res_skip_layers.2.weight_g', 'flow.flows.2.enc.res_skip_layers.2.weight_v', 'flow.flows.2.enc.res_skip_layers.3.bias', 'flow.flows.2.enc.res_skip_layers.3.weight_g', 'flow.flows.2.enc.res_skip_layers.3.weight_v', 'flow.flows.2.enc.cond_layer.bias', 'flow.flows.2.enc.cond_layer.weight_g', 'flow.flows.2.enc.cond_layer.weight_v', 'flow.flows.2.post.weight', 'flow.flows.2.post.bias', 'flow.flows.4.pre.weight', 'flow.flows.4.pre.bias', 'flow.flows.4.enc.in_layers.0.bias', 'flow.flows.4.enc.in_layers.0.weight_g', 'flow.flows.4.enc.in_layers.0.weight_v', 'flow.flows.4.enc.in_layers.1.bias', 'flow.flows.4.enc.in_layers.1.weight_g', 'flow.flows.4.enc.in_layers.1.weight_v', 'flow.flows.4.enc.in_layers.2.bias', 'flow.flows.4.enc.in_layers.2.weight_g', 'flow.flows.4.enc.in_layers.2.weight_v', 'flow.flows.4.enc.in_layers.3.bias', 'flow.flows.4.enc.in_layers.3.weight_g', 'flow.flows.4.enc.in_layers.3.weight_v', 'flow.flows.4.enc.res_skip_layers.0.bias', 'flow.flows.4.enc.res_skip_layers.0.weight_g', 'flow.flows.4.enc.res_skip_layers.0.weight_v', 'flow.flows.4.enc.res_skip_layers.1.bias', 'flow.flows.4.enc.res_skip_layers.1.weight_g', 'flow.flows.4.enc.res_skip_layers.1.weight_v', 'flow.flows.4.enc.res_skip_layers.2.bias', 'flow.flows.4.enc.res_skip_layers.2.weight_g', 'flow.flows.4.enc.res_skip_layers.2.weight_v', 'flow.flows.4.enc.res_skip_layers.3.bias', 'flow.flows.4.enc.res_skip_layers.3.weight_g', 'flow.flows.4.enc.res_skip_layers.3.weight_v', 'flow.flows.4.enc.cond_layer.bias', 'flow.flows.4.enc.cond_layer.weight_g', 'flow.flows.4.enc.cond_layer.weight_v', 'flow.flows.4.post.weight', 'flow.flows.4.post.bias', 'flow.flows.6.pre.weight', 'flow.flows.6.pre.bias', 'flow.flows.6.enc.in_layers.0.bias', 'flow.flows.6.enc.in_layers.0.weight_g', 'flow.flows.6.enc.in_layers.0.weight_v', 'flow.flows.6.enc.in_layers.1.bias', 'flow.flows.6.enc.in_layers.1.weight_g', 'flow.flows.6.enc.in_layers.1.weight_v', 'flow.flows.6.enc.in_layers.2.bias', 'flow.flows.6.enc.in_layers.2.weight_g', 'flow.flows.6.enc.in_layers.2.weight_v', 'flow.flows.6.enc.in_layers.3.bias', 'flow.flows.6.enc.in_layers.3.weight_g', 'flow.flows.6.enc.in_layers.3.weight_v', 'flow.flows.6.enc.res_skip_layers.0.bias', 'flow.flows.6.enc.res_skip_layers.0.weight_g', 'flow.flows.6.enc.res_skip_layers.0.weight_v', 'flow.flows.6.enc.res_skip_layers.1.bias', 'flow.flows.6.enc.res_skip_layers.1.weight_g', 'flow.flows.6.enc.res_skip_layers.1.weight_v', 'flow.flows.6.enc.res_skip_layers.2.bias', 'flow.flows.6.enc.res_skip_layers.2.weight_g', 'flow.flows.6.enc.res_skip_layers.2.weight_v', 'flow.flows.6.enc.res_skip_layers.3.bias', 'flow.flows.6.enc.res_skip_layers.3.weight_g', 'flow.flows.6.enc.res_skip_layers.3.weight_v', 'flow.flows.6.enc.cond_layer.bias', 'flow.flows.6.enc.cond_layer.weight_g', 'flow.flows.6.enc.cond_layer.weight_v', 'flow.flows.6.post.weight', 'flow.flows.6.post.bias', 'ref_enc.spectral.0.fc.weight', 'ref_enc.spectral.0.fc.bias', 'ref_enc.spectral.3.fc.weight', 'ref_enc.spectral.3.fc.bias', 'ref_enc.temporal.0.conv1.conv.weight', 'ref_enc.temporal.0.conv1.conv.bias', 'ref_enc.temporal.1.conv1.conv.weight', 'ref_enc.temporal.1.conv1.conv.bias', 'ref_enc.slf_attn.w_qs.weight', 'ref_enc.slf_attn.w_qs.bias', 'ref_enc.slf_attn.w_ks.weight', 'ref_enc.slf_attn.w_ks.bias', 'ref_enc.slf_attn.w_vs.weight', 'ref_enc.slf_attn.w_vs.bias', 'ref_enc.slf_attn.fc.weight', 'ref_enc.slf_attn.fc.bias', 'ref_enc.fc.fc.weight', 'ref_enc.fc.fc.bias', 'ssl_proj.weight', 'ssl_proj.bias', 'quantizer.vq.layers.0._codebook.inited', 'quantizer.vq.layers.0._codebook.cluster_size', 'quantizer.vq.layers.0._codebook.embed', 'quantizer.vq.layers.0._codebook.embed_avg'])

> dict_s2["config"].keys()
dict_keys(['train', 'data', 'model', 's2_ckpt_dir', 'content_module', 'save_weight_dir', 'name', 'pretrain', 'resume_step'])

> dict_s2["info"]
'pretrained_s2G_v1'


```
{% endcode %}







```bash
> vq_model
SynthesizerTrn(
  (enc_p): TextEncoder(
    (ssl_proj): Conv1d(768, 192, kernel_size=(1,), stride=(1,))
    (encoder_ssl): Encoder(
      (drop): Dropout(p=0.1, inplace=False)
      (attn_layers): ModuleList(
        (0-2): 3 x MultiHeadAttention(
          (conv_q): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (conv_k): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (conv_v): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (conv_o): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (drop): Dropout(p=0.1, inplace=False)
        )
      )
      (norm_layers_1): ModuleList(
        (0-2): 3 x LayerNorm()
      )
      (ffn_layers): ModuleList(
        (0-2): 3 x FFN(
          (conv_1): Conv1d(192, 768, kernel_size=(3,), stride=(1,))
          (conv_2): Conv1d(768, 192, kernel_size=(3,), stride=(1,))
          (drop): Dropout(p=0.1, inplace=False)
        )
      )
      (norm_layers_2): ModuleList(
        (0-2): 3 x LayerNorm()
      )
    )
    (encoder_text): Encoder(
      (drop): Dropout(p=0.1, inplace=False)
      (attn_layers): ModuleList(
        (0-5): 6 x MultiHeadAttention(
          (conv_q): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (conv_k): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (conv_v): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (conv_o): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (drop): Dropout(p=0.1, inplace=False)
        )
      )
      (norm_layers_1): ModuleList(
        (0-5): 6 x LayerNorm()
      )
      (ffn_layers): ModuleList(
        (0-5): 6 x FFN(
          (conv_1): Conv1d(192, 768, kernel_size=(3,), stride=(1,))
          (conv_2): Conv1d(768, 192, kernel_size=(3,), stride=(1,))
          (drop): Dropout(p=0.1, inplace=False)
        )
      )
      (norm_layers_2): ModuleList(
        (0-5): 6 x LayerNorm()
      )
    )
    (text_embedding): Embedding(322, 192)
    (mrte): MRTE(
      (cross_attention): MultiHeadAttention(
        (conv_q): Conv1d(512, 512, kernel_size=(1,), stride=(1,))
        (conv_k): Conv1d(512, 512, kernel_size=(1,), stride=(1,))
        (conv_v): Conv1d(512, 512, kernel_size=(1,), stride=(1,))
        (conv_o): Conv1d(512, 512, kernel_size=(1,), stride=(1,))
        (drop): Dropout(p=0.0, inplace=False)
      )
      (c_pre): Conv1d(192, 512, kernel_size=(1,), stride=(1,))
      (text_pre): Conv1d(192, 512, kernel_size=(1,), stride=(1,))
      (c_post): Conv1d(512, 192, kernel_size=(1,), stride=(1,))
    )
    (encoder2): Encoder(
      (drop): Dropout(p=0.1, inplace=False)
      (attn_layers): ModuleList(
        (0-2): 3 x MultiHeadAttention(
          (conv_q): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (conv_k): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (conv_v): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (conv_o): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (drop): Dropout(p=0.1, inplace=False)
        )
      )
      (norm_layers_1): ModuleList(
        (0-2): 3 x LayerNorm()
      )
      (ffn_layers): ModuleList(
        (0-2): 3 x FFN(
          (conv_1): Conv1d(192, 768, kernel_size=(3,), stride=(1,))
          (conv_2): Conv1d(768, 192, kernel_size=(3,), stride=(1,))
          (drop): Dropout(p=0.1, inplace=False)
        )
      )
      (norm_layers_2): ModuleList(
        (0-2): 3 x LayerNorm()
      )
    )
    (proj): Conv1d(192, 384, kernel_size=(1,), stride=(1,))
  )
  (dec): Generator(
    (conv_pre): Conv1d(192, 512, kernel_size=(7,), stride=(1,), padding=(3,))
    (ups): ModuleList(
      (0): ConvTranspose1d(512, 256, kernel_size=(16,), stride=(10,), padding=(3,))
      (1): ConvTranspose1d(256, 128, kernel_size=(16,), stride=(8,), padding=(4,))
      (2): ConvTranspose1d(128, 64, kernel_size=(8,), stride=(2,), padding=(3,))
      (3): ConvTranspose1d(64, 32, kernel_size=(2,), stride=(2,))
      (4): ConvTranspose1d(32, 16, kernel_size=(2,), stride=(2,))
    )
    (resblocks): ModuleList(
      (0): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(256, 256, kernel_size=(3,), stride=(1,), padding=(1,))
          (1): Conv1d(256, 256, kernel_size=(3,), stride=(1,), padding=(3,), dilation=(3,))
          (2): Conv1d(256, 256, kernel_size=(3,), stride=(1,), padding=(5,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(256, 256, kernel_size=(3,), stride=(1,), padding=(1,))
        )
      )
      (1): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(256, 256, kernel_size=(7,), stride=(1,), padding=(3,))
          (1): Conv1d(256, 256, kernel_size=(7,), stride=(1,), padding=(9,), dilation=(3,))
          (2): Conv1d(256, 256, kernel_size=(7,), stride=(1,), padding=(15,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(256, 256, kernel_size=(7,), stride=(1,), padding=(3,))
        )
      )
      (2): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(256, 256, kernel_size=(11,), stride=(1,), padding=(5,))
          (1): Conv1d(256, 256, kernel_size=(11,), stride=(1,), padding=(15,), dilation=(3,))
          (2): Conv1d(256, 256, kernel_size=(11,), stride=(1,), padding=(25,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(256, 256, kernel_size=(11,), stride=(1,), padding=(5,))
        )
      )
      (3): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(128, 128, kernel_size=(3,), stride=(1,), padding=(1,))
          (1): Conv1d(128, 128, kernel_size=(3,), stride=(1,), padding=(3,), dilation=(3,))
          (2): Conv1d(128, 128, kernel_size=(3,), stride=(1,), padding=(5,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(128, 128, kernel_size=(3,), stride=(1,), padding=(1,))
        )
      )
      (4): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(128, 128, kernel_size=(7,), stride=(1,), padding=(3,))
          (1): Conv1d(128, 128, kernel_size=(7,), stride=(1,), padding=(9,), dilation=(3,))
          (2): Conv1d(128, 128, kernel_size=(7,), stride=(1,), padding=(15,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(128, 128, kernel_size=(7,), stride=(1,), padding=(3,))
        )
      )
      (5): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(128, 128, kernel_size=(11,), stride=(1,), padding=(5,))
          (1): Conv1d(128, 128, kernel_size=(11,), stride=(1,), padding=(15,), dilation=(3,))
          (2): Conv1d(128, 128, kernel_size=(11,), stride=(1,), padding=(25,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(128, 128, kernel_size=(11,), stride=(1,), padding=(5,))
        )
      )
      (6): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(64, 64, kernel_size=(3,), stride=(1,), padding=(1,))
          (1): Conv1d(64, 64, kernel_size=(3,), stride=(1,), padding=(3,), dilation=(3,))
          (2): Conv1d(64, 64, kernel_size=(3,), stride=(1,), padding=(5,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(64, 64, kernel_size=(3,), stride=(1,), padding=(1,))
        )
      )
      (7): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(64, 64, kernel_size=(7,), stride=(1,), padding=(3,))
          (1): Conv1d(64, 64, kernel_size=(7,), stride=(1,), padding=(9,), dilation=(3,))
          (2): Conv1d(64, 64, kernel_size=(7,), stride=(1,), padding=(15,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(64, 64, kernel_size=(7,), stride=(1,), padding=(3,))
        )
      )
      (8): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(64, 64, kernel_size=(11,), stride=(1,), padding=(5,))
          (1): Conv1d(64, 64, kernel_size=(11,), stride=(1,), padding=(15,), dilation=(3,))
          (2): Conv1d(64, 64, kernel_size=(11,), stride=(1,), padding=(25,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(64, 64, kernel_size=(11,), stride=(1,), padding=(5,))
        )
      )
      (9): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(32, 32, kernel_size=(3,), stride=(1,), padding=(1,))
          (1): Conv1d(32, 32, kernel_size=(3,), stride=(1,), padding=(3,), dilation=(3,))
          (2): Conv1d(32, 32, kernel_size=(3,), stride=(1,), padding=(5,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(32, 32, kernel_size=(3,), stride=(1,), padding=(1,))
        )
      )
      (10): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(32, 32, kernel_size=(7,), stride=(1,), padding=(3,))
          (1): Conv1d(32, 32, kernel_size=(7,), stride=(1,), padding=(9,), dilation=(3,))
          (2): Conv1d(32, 32, kernel_size=(7,), stride=(1,), padding=(15,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(32, 32, kernel_size=(7,), stride=(1,), padding=(3,))
        )
      )
      (11): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(32, 32, kernel_size=(11,), stride=(1,), padding=(5,))
          (1): Conv1d(32, 32, kernel_size=(11,), stride=(1,), padding=(15,), dilation=(3,))
          (2): Conv1d(32, 32, kernel_size=(11,), stride=(1,), padding=(25,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(32, 32, kernel_size=(11,), stride=(1,), padding=(5,))
        )
      )
      (12): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(16, 16, kernel_size=(3,), stride=(1,), padding=(1,))
          (1): Conv1d(16, 16, kernel_size=(3,), stride=(1,), padding=(3,), dilation=(3,))
          (2): Conv1d(16, 16, kernel_size=(3,), stride=(1,), padding=(5,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(16, 16, kernel_size=(3,), stride=(1,), padding=(1,))
        )
      )
      (13): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(16, 16, kernel_size=(7,), stride=(1,), padding=(3,))
          (1): Conv1d(16, 16, kernel_size=(7,), stride=(1,), padding=(9,), dilation=(3,))
          (2): Conv1d(16, 16, kernel_size=(7,), stride=(1,), padding=(15,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(16, 16, kernel_size=(7,), stride=(1,), padding=(3,))
        )
      )
      (14): ResBlock1(
        (convs1): ModuleList(
          (0): Conv1d(16, 16, kernel_size=(11,), stride=(1,), padding=(5,))
          (1): Conv1d(16, 16, kernel_size=(11,), stride=(1,), padding=(15,), dilation=(3,))
          (2): Conv1d(16, 16, kernel_size=(11,), stride=(1,), padding=(25,), dilation=(5,))
        )
        (convs2): ModuleList(
          (0-2): 3 x Conv1d(16, 16, kernel_size=(11,), stride=(1,), padding=(5,))
        )
      )
    )
    (conv_post): Conv1d(16, 1, kernel_size=(7,), stride=(1,), padding=(3,), bias=False)
    (cond): Conv1d(512, 512, kernel_size=(1,), stride=(1,))
  )
  (enc_q): PosteriorEncoder(
    (pre): Conv1d(1025, 192, kernel_size=(1,), stride=(1,))
    (enc): WN(
      (in_layers): ModuleList(
        (0-15): 16 x Conv1d(192, 384, kernel_size=(5,), stride=(1,), padding=(2,))
      )
      (res_skip_layers): ModuleList(
        (0-14): 15 x Conv1d(192, 384, kernel_size=(1,), stride=(1,))
        (15): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
      )
      (drop): Dropout(p=0, inplace=False)
      (cond_layer): Conv1d(512, 6144, kernel_size=(1,), stride=(1,))
    )
    (proj): Conv1d(192, 384, kernel_size=(1,), stride=(1,))
  )
  (flow): ResidualCouplingBlock(
    (flows): ModuleList(
      (0): ResidualCouplingLayer(
        (pre): Conv1d(96, 192, kernel_size=(1,), stride=(1,))
        (enc): WN(
          (in_layers): ModuleList(
            (0-3): 4 x Conv1d(192, 384, kernel_size=(5,), stride=(1,), padding=(2,))
          )
          (res_skip_layers): ModuleList(
            (0-2): 3 x Conv1d(192, 384, kernel_size=(1,), stride=(1,))
            (3): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          )
          (drop): Dropout(p=0, inplace=False)
          (cond_layer): Conv1d(512, 1536, kernel_size=(1,), stride=(1,))
        )
        (post): Conv1d(192, 96, kernel_size=(1,), stride=(1,))
      )
      (1): Flip()
      (2): ResidualCouplingLayer(
        (pre): Conv1d(96, 192, kernel_size=(1,), stride=(1,))
        (enc): WN(
          (in_layers): ModuleList(
            (0-3): 4 x Conv1d(192, 384, kernel_size=(5,), stride=(1,), padding=(2,))
          )
          (res_skip_layers): ModuleList(
            (0-2): 3 x Conv1d(192, 384, kernel_size=(1,), stride=(1,))
            (3): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          )
          (drop): Dropout(p=0, inplace=False)
          (cond_layer): Conv1d(512, 1536, kernel_size=(1,), stride=(1,))
        )
        (post): Conv1d(192, 96, kernel_size=(1,), stride=(1,))
      )
      (3): Flip()
      (4): ResidualCouplingLayer(
        (pre): Conv1d(96, 192, kernel_size=(1,), stride=(1,))
        (enc): WN(
          (in_layers): ModuleList(
            (0-3): 4 x Conv1d(192, 384, kernel_size=(5,), stride=(1,), padding=(2,))
          )
          (res_skip_layers): ModuleList(
            (0-2): 3 x Conv1d(192, 384, kernel_size=(1,), stride=(1,))
            (3): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          )
          (drop): Dropout(p=0, inplace=False)
          (cond_layer): Conv1d(512, 1536, kernel_size=(1,), stride=(1,))
        )
        (post): Conv1d(192, 96, kernel_size=(1,), stride=(1,))
      )
      (5): Flip()
      (6): ResidualCouplingLayer(
        (pre): Conv1d(96, 192, kernel_size=(1,), stride=(1,))
        (enc): WN(
          (in_layers): ModuleList(
            (0-3): 4 x Conv1d(192, 384, kernel_size=(5,), stride=(1,), padding=(2,))
          )
          (res_skip_layers): ModuleList(
            (0-2): 3 x Conv1d(192, 384, kernel_size=(1,), stride=(1,))
            (3): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          )
          (drop): Dropout(p=0, inplace=False)
          (cond_layer): Conv1d(512, 1536, kernel_size=(1,), stride=(1,))
        )
        (post): Conv1d(192, 96, kernel_size=(1,), stride=(1,))
      )
      (7): Flip()
    )
  )
  (ref_enc): MelStyleEncoder(
    (spectral): Sequential(
      (0): LinearNorm(
        (fc): Linear(in_features=1025, out_features=128, bias=True)
      )
      (1): Mish()
      (2): Dropout(p=0.1, inplace=False)
      (3): LinearNorm(
        (fc): Linear(in_features=128, out_features=128, bias=True)
      )
      (4): Mish()
      (5): Dropout(p=0.1, inplace=False)
    )
    (temporal): Sequential(
      (0): Conv1dGLU(
        (conv1): ConvNorm(
          (conv): Conv1d(128, 256, kernel_size=(5,), stride=(1,), padding=(2,))
        )
        (dropout): Dropout(p=0.1, inplace=False)
      )
      (1): Conv1dGLU(
        (conv1): ConvNorm(
          (conv): Conv1d(128, 256, kernel_size=(5,), stride=(1,), padding=(2,))
        )
        (dropout): Dropout(p=0.1, inplace=False)
      )
    )
    (slf_attn): MultiHeadAttention(
      (w_qs): Linear(in_features=128, out_features=128, bias=True)
      (w_ks): Linear(in_features=128, out_features=128, bias=True)
      (w_vs): Linear(in_features=128, out_features=128, bias=True)
      (attention): ScaledDotProductAttention(
        (softmax): Softmax(dim=2)
        (dropout): Dropout(p=0.1, inplace=False)
      )
      (fc): Linear(in_features=128, out_features=128, bias=True)
      (dropout): Dropout(p=0.1, inplace=False)
    )
    (fc): LinearNorm(
      (fc): Linear(in_features=128, out_features=512, bias=True)
    )
  )
  (ssl_proj): Conv1d(768, 768, kernel_size=(2,), stride=(2,))
  (quantizer): ResidualVectorQuantizer(
    (vq): ResidualVectorQuantization(
      (layers): ModuleList(
        (0): VectorQuantization(
          (project_in): Identity()
          (project_out): Identity()
          (_codebook): EuclideanCodebook()
        )
      )
    )
  )
)
```









