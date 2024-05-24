# NAI-SVC Experiment Log

## Reference

{% embed url="https://github.com/SUC-DriverOld/so-vits-svc-Chinese-Detaild-Documents" %}

[https://www.bilibili.com/video/BV1Hr4y197Cy](https://www.bilibili.com/video/BV1Hr4y197Cy/?p=22\&spm\_id\_from=333.880.my\_history.page.click\&vd\_source=1077d0b350f0a7b3167e4ce477f95e9f)





## Environment

Linux (millennium-A24)

Python 3.10.13



## Initial Setup

```bash
git clone git@github.com:T-K-233/NAI-SVC-WS.git NAI-SVC-Workspace
git clone git@github.com:T-K-233/NAI-SVC-WS.git Inference
```



```bash
conda create --prefix ./.conda-env python=3.10
conda activate ./.conda-env/

cd ./Inference
pip install -r ./requirements.txt
```



```bash
> python3 env_checker.py
Python 3.10.13 | packaged by conda-forge | (main, Oct 26 2023, 18:07:37) [GCC 12.3.0] ✓
Numpy 1.23.5 ✓
Torch 2.1.0+cu121 ✓
TorchVision 0.16.1+cu121 ✓
TorchAudio 2.1.0+cu121 ✓
gradio 3.50.2 ✓
numba 0.58.1 ✓
pyworld 0.3.4 ✓
scipy 1.10.0 ✓
tqdm 4.66.1 ✓
parselmouth 0.4.3 ✓
fairseq 0.12.2 ✓
librosa 0.9.1 ✓
```



```bash
> nvidia-smi
NVIDIA-SMI 535.86.10              Driver Version: 535.86.10    CUDA Version: 12.2
```



## Usage

```bash
cd /scratch/NAI-SVC-Workspace/
conda activate ./.conda-env/
```



### File Structure

**./dataset\_raw/**

store wave files, something like

```bash
dataset_raw
 |- singer0
   |- singer_0.wav
   |- singer_1.wav
   
```



**./filelists/**

store wav file names, referenced from root path



**./raw/**

inference source



**./results/**

generated results



### Pretrain Model Downloads

```bash
./scripts/get_base_model.sh
./scripts/get_contentvec_model.sh
./scripts/get_nsf_gan.sh
```



## Dataset Preparation

### Requirements for Dataset

Minimum: 100 entries of 5\~15s audio clips

Normal: 1.5 hours of audio



Sampling rate: 48000

### BGM Removal

<figure><img src="../.gitbook/assets/image (185).png" alt=""><figcaption></figcaption></figure>







### Noise Removal









### Slice

{% embed url="https://github.com/flutydeer/audio-slicer/releases" %}

<figure><img src="../.gitbook/assets/image (176).png" alt=""><figcaption></figcaption></figure>



### Filter

Remove audio clips that are shorter than 4 seconds, and cut the clips that are longer than 10 seconds.



### Normalize

In Adobe Audition, select "Window" -> "Match Loudness" to open the loudness-matching panel.

<figure><img src="../.gitbook/assets/image (178).png" alt=""><figcaption></figcaption></figure>

Set the settings to use

Match To: **ITU-R BS.1770-3 Loudness**

Target Loudness: **-11** LUFS

Tolerance: **0.5** LU

Max True Peak Level: **-1** dBTP

Finally, click "Run" to run loudness matching.





Put wav files in dataset\_raw/\<speaker>/\*.wav



### Preprocess

```bash
python resample.py --skip_loudnorm
```



```bash
python preprocess_flist_config.py --speech_encoder vec768l12 --vol_aug
```



```bash
python preprocess_hubert_f0.py --f0_predictor dio --use_diff
```



## Training



#### Shallow Diffusion Model

```bash
CUDA_VISIBLE_DEVICES=0 python train_diff.py -c configs/diffusion.yaml
```



#### Main Model

```bash
CUDA_VISIBLE_DEVICES=0 python train.py -c configs/config.json -m 44k
```

{% embed url="https://github.com/Anjok07/ultimatevocalremovergui" %}



### Model Fusion

```json
{
  "/tmp/gradio/bfb6032133f82121787fe12bda7a72772ea1ded5/G_24000.pth": 20,
  "/tmp/gradio/0b96f6ff59483d973613dbc911688d02d68d959a/G_104000.pth": 20,
  "/tmp/gradio/c44409950386074c71bcfedbc2a155e64413cdc8/G_84000.pth": 30,
  "/tmp/gradio/b6807f32bc6c649e73f5b52eda19b6e7388f0cd3/G_152000.pth": 10,
  "/tmp/gradio/9ef9a8f73dd2ff8de416f708a0174d1d72d2e6e5/G_108000.pth": 20
}
```









### Loss

loss/g/f0、loss/g/mel 和 loss/g/total 应当是震荡下降的，并最终收敛在某个值

loss/g/kl 应当是低位震荡的

loss/g/fm 应当在训练的中期持续上升，并在后期放缓上升趋势甚至开始下降







## Model Architecture



```bash
> checkpoint_dict = torch.load(model.net_g_path, map_location='cpu')
> checkpoint_dict.keys()
dict_keys(['model', 'iteration', 'optimizer', 'learning_rate'])
# model: collections.OrderedDict
# optimizer: dict

> checkpoint_dict["iteration"]
5999
> checkpoint_dict["learning_rate"]
0.0001

> checkpoint_dict["model"].keys()
odict_keys(['emb_g.weight', 'emb_vol.weight', 'emb_vol.bias', 'pre.weight', 'pre.bias', 'enc_p.proj.weight', 'enc_p.proj.bias', 'enc_p.f0_emb.weight', 'enc_p.enc_.attn_layers.0.emb_rel_k', 'enc_p.enc_.attn_layers.0.emb_rel_v', 'enc_p.enc_.attn_layers.0.conv_q.weight', 'enc_p.enc_.attn_layers.0.conv_q.bias', 'enc_p.enc_.attn_layers.0.conv_k.weight', 'enc_p.enc_.attn_layers.0.conv_k.bias', 'enc_p.enc_.attn_layers.0.conv_v.weight', 'enc_p.enc_.attn_layers.0.conv_v.bias', 'enc_p.enc_.attn_layers.0.conv_o.weight', 'enc_p.enc_.attn_layers.0.conv_o.bias', 'enc_p.enc_.attn_layers.1.emb_rel_k', 'enc_p.enc_.attn_layers.1.emb_rel_v', 'enc_p.enc_.attn_layers.1.conv_q.weight', 'enc_p.enc_.attn_layers.1.conv_q.bias', 'enc_p.enc_.attn_layers.1.conv_k.weight', 'enc_p.enc_.attn_layers.1.conv_k.bias', 'enc_p.enc_.attn_layers.1.conv_v.weight', 'enc_p.enc_.attn_layers.1.conv_v.bias', 'enc_p.enc_.attn_layers.1.conv_o.weight', 'enc_p.enc_.attn_layers.1.conv_o.bias', 'enc_p.enc_.attn_layers.2.emb_rel_k', 'enc_p.enc_.attn_layers.2.emb_rel_v', 'enc_p.enc_.attn_layers.2.conv_q.weight', 'enc_p.enc_.attn_layers.2.conv_q.bias', 'enc_p.enc_.attn_layers.2.conv_k.weight', 'enc_p.enc_.attn_layers.2.conv_k.bias', 'enc_p.enc_.attn_layers.2.conv_v.weight', 'enc_p.enc_.attn_layers.2.conv_v.bias', 'enc_p.enc_.attn_layers.2.conv_o.weight', 'enc_p.enc_.attn_layers.2.conv_o.bias', 'enc_p.enc_.attn_layers.3.emb_rel_k', 'enc_p.enc_.attn_layers.3.emb_rel_v', 'enc_p.enc_.attn_layers.3.conv_q.weight', 'enc_p.enc_.attn_layers.3.conv_q.bias', 'enc_p.enc_.attn_layers.3.conv_k.weight', 'enc_p.enc_.attn_layers.3.conv_k.bias', 'enc_p.enc_.attn_layers.3.conv_v.weight', 'enc_p.enc_.attn_layers.3.conv_v.bias', 'enc_p.enc_.attn_layers.3.conv_o.weight', 'enc_p.enc_.attn_layers.3.conv_o.bias', 'enc_p.enc_.attn_layers.4.emb_rel_k', 'enc_p.enc_.attn_layers.4.emb_rel_v', 'enc_p.enc_.attn_layers.4.conv_q.weight', 'enc_p.enc_.attn_layers.4.conv_q.bias', 'enc_p.enc_.attn_layers.4.conv_k.weight', 'enc_p.enc_.attn_layers.4.conv_k.bias', 'enc_p.enc_.attn_layers.4.conv_v.weight', 'enc_p.enc_.attn_layers.4.conv_v.bias', 'enc_p.enc_.attn_layers.4.conv_o.weight', 'enc_p.enc_.attn_layers.4.conv_o.bias', 'enc_p.enc_.attn_layers.5.emb_rel_k', 'enc_p.enc_.attn_layers.5.emb_rel_v', 'enc_p.enc_.attn_layers.5.conv_q.weight', 'enc_p.enc_.attn_layers.5.conv_q.bias', 'enc_p.enc_.attn_layers.5.conv_k.weight', 'enc_p.enc_.attn_layers.5.conv_k.bias', 'enc_p.enc_.attn_layers.5.conv_v.weight', 'enc_p.enc_.attn_layers.5.conv_v.bias', 'enc_p.enc_.attn_layers.5.conv_o.weight', 'enc_p.enc_.attn_layers.5.conv_o.bias', 'enc_p.enc_.norm_layers_1.0.gamma', 'enc_p.enc_.norm_layers_1.0.beta', 'enc_p.enc_.norm_layers_1.1.gamma', 'enc_p.enc_.norm_layers_1.1.beta', 'enc_p.enc_.norm_layers_1.2.gamma', 'enc_p.enc_.norm_layers_1.2.beta', 'enc_p.enc_.norm_layers_1.3.gamma', 'enc_p.enc_.norm_layers_1.3.beta', 'enc_p.enc_.norm_layers_1.4.gamma', 'enc_p.enc_.norm_layers_1.4.beta', 'enc_p.enc_.norm_layers_1.5.gamma', 'enc_p.enc_.norm_layers_1.5.beta', 'enc_p.enc_.ffn_layers.0.conv_1.weight', 'enc_p.enc_.ffn_layers.0.conv_1.bias', 'enc_p.enc_.ffn_layers.0.conv_2.weight', 'enc_p.enc_.ffn_layers.0.conv_2.bias', 'enc_p.enc_.ffn_layers.1.conv_1.weight', 'enc_p.enc_.ffn_layers.1.conv_1.bias', 'enc_p.enc_.ffn_layers.1.conv_2.weight', 'enc_p.enc_.ffn_layers.1.conv_2.bias', 'enc_p.enc_.ffn_layers.2.conv_1.weight', 'enc_p.enc_.ffn_layers.2.conv_1.bias', 'enc_p.enc_.ffn_layers.2.conv_2.weight', 'enc_p.enc_.ffn_layers.2.conv_2.bias', 'enc_p.enc_.ffn_layers.3.conv_1.weight', 'enc_p.enc_.ffn_layers.3.conv_1.bias', 'enc_p.enc_.ffn_layers.3.conv_2.weight', 'enc_p.enc_.ffn_layers.3.conv_2.bias', 'enc_p.enc_.ffn_layers.4.conv_1.weight', 'enc_p.enc_.ffn_layers.4.conv_1.bias', 'enc_p.enc_.ffn_layers.4.conv_2.weight', 'enc_p.enc_.ffn_layers.4.conv_2.bias', 'enc_p.enc_.ffn_layers.5.conv_1.weight', 'enc_p.enc_.ffn_layers.5.conv_1.bias', 'enc_p.enc_.ffn_layers.5.conv_2.weight', 'enc_p.enc_.ffn_layers.5.conv_2.bias', 'enc_p.enc_.norm_layers_2.0.gamma', 'enc_p.enc_.norm_layers_2.0.beta', 'enc_p.enc_.norm_layers_2.1.gamma', 'enc_p.enc_.norm_layers_2.1.beta', 'enc_p.enc_.norm_layers_2.2.gamma', 'enc_p.enc_.norm_layers_2.2.beta', 'enc_p.enc_.norm_layers_2.3.gamma', 'enc_p.enc_.norm_layers_2.3.beta', 'enc_p.enc_.norm_layers_2.4.gamma', 'enc_p.enc_.norm_layers_2.4.beta', 'enc_p.enc_.norm_layers_2.5.gamma', 'enc_p.enc_.norm_layers_2.5.beta', 'dec.m_source.l_linear.weight', 'dec.m_source.l_linear.bias', 'dec.noise_convs.0.weight', 'dec.noise_convs.0.bias', 'dec.noise_convs.1.weight', 'dec.noise_convs.1.bias', 'dec.noise_convs.2.weight', 'dec.noise_convs.2.bias', 'dec.noise_convs.3.weight', 'dec.noise_convs.3.bias', 'dec.noise_convs.4.weight', 'dec.noise_convs.4.bias', 'dec.conv_pre.bias', 'dec.conv_pre.weight_g', 'dec.conv_pre.weight_v', 'dec.ups.0.bias', 'dec.ups.0.weight_g', 'dec.ups.0.weight_v', 'dec.ups.1.bias', 'dec.ups.1.weight_g', 'dec.ups.1.weight_v', 'dec.ups.2.bias', 'dec.ups.2.weight_g', 'dec.ups.2.weight_v', 'dec.ups.3.bias', 'dec.ups.3.weight_g', 'dec.ups.3.weight_v', 'dec.ups.4.bias', 'dec.ups.4.weight_g', 'dec.ups.4.weight_v', 'dec.resblocks.0.convs1.0.bias', 'dec.resblocks.0.convs1.0.weight_g', 'dec.resblocks.0.convs1.0.weight_v', 'dec.resblocks.0.convs1.1.bias', 'dec.resblocks.0.convs1.1.weight_g', 'dec.resblocks.0.convs1.1.weight_v', 'dec.resblocks.0.convs1.2.bias', 'dec.resblocks.0.convs1.2.weight_g', 'dec.resblocks.0.convs1.2.weight_v', 'dec.resblocks.0.convs2.0.bias', 'dec.resblocks.0.convs2.0.weight_g', 'dec.resblocks.0.convs2.0.weight_v', 'dec.resblocks.0.convs2.1.bias', 'dec.resblocks.0.convs2.1.weight_g', 'dec.resblocks.0.convs2.1.weight_v', 'dec.resblocks.0.convs2.2.bias', 'dec.resblocks.0.convs2.2.weight_g', 'dec.resblocks.0.convs2.2.weight_v', 'dec.resblocks.1.convs1.0.bias', 'dec.resblocks.1.convs1.0.weight_g', 'dec.resblocks.1.convs1.0.weight_v', 'dec.resblocks.1.convs1.1.bias', 'dec.resblocks.1.convs1.1.weight_g', 'dec.resblocks.1.convs1.1.weight_v', 'dec.resblocks.1.convs1.2.bias', 'dec.resblocks.1.convs1.2.weight_g', 'dec.resblocks.1.convs1.2.weight_v', 'dec.resblocks.1.convs2.0.bias', 'dec.resblocks.1.convs2.0.weight_g', 'dec.resblocks.1.convs2.0.weight_v', 'dec.resblocks.1.convs2.1.bias', 'dec.resblocks.1.convs2.1.weight_g', 'dec.resblocks.1.convs2.1.weight_v', 'dec.resblocks.1.convs2.2.bias', 'dec.resblocks.1.convs2.2.weight_g', 'dec.resblocks.1.convs2.2.weight_v', 'dec.resblocks.2.convs1.0.bias', 'dec.resblocks.2.convs1.0.weight_g', 'dec.resblocks.2.convs1.0.weight_v', 'dec.resblocks.2.convs1.1.bias', 'dec.resblocks.2.convs1.1.weight_g', 'dec.resblocks.2.convs1.1.weight_v', 'dec.resblocks.2.convs1.2.bias', 'dec.resblocks.2.convs1.2.weight_g', 'dec.resblocks.2.convs1.2.weight_v', 'dec.resblocks.2.convs2.0.bias', 'dec.resblocks.2.convs2.0.weight_g', 'dec.resblocks.2.convs2.0.weight_v', 'dec.resblocks.2.convs2.1.bias', 'dec.resblocks.2.convs2.1.weight_g', 'dec.resblocks.2.convs2.1.weight_v', 'dec.resblocks.2.convs2.2.bias', 'dec.resblocks.2.convs2.2.weight_g', 'dec.resblocks.2.convs2.2.weight_v', 'dec.resblocks.3.convs1.0.bias', 'dec.resblocks.3.convs1.0.weight_g', 'dec.resblocks.3.convs1.0.weight_v', 'dec.resblocks.3.convs1.1.bias', 'dec.resblocks.3.convs1.1.weight_g', 'dec.resblocks.3.convs1.1.weight_v', 'dec.resblocks.3.convs1.2.bias', 'dec.resblocks.3.convs1.2.weight_g', 'dec.resblocks.3.convs1.2.weight_v', 'dec.resblocks.3.convs2.0.bias', 'dec.resblocks.3.convs2.0.weight_g', 'dec.resblocks.3.convs2.0.weight_v', 'dec.resblocks.3.convs2.1.bias', 'dec.resblocks.3.convs2.1.weight_g', 'dec.resblocks.3.convs2.1.weight_v', 'dec.resblocks.3.convs2.2.bias', 'dec.resblocks.3.convs2.2.weight_g', 'dec.resblocks.3.convs2.2.weight_v', 'dec.resblocks.4.convs1.0.bias', 'dec.resblocks.4.convs1.0.weight_g', 'dec.resblocks.4.convs1.0.weight_v', 'dec.resblocks.4.convs1.1.bias', 'dec.resblocks.4.convs1.1.weight_g', 'dec.resblocks.4.convs1.1.weight_v', 'dec.resblocks.4.convs1.2.bias', 'dec.resblocks.4.convs1.2.weight_g', 'dec.resblocks.4.convs1.2.weight_v', 'dec.resblocks.4.convs2.0.bias', 'dec.resblocks.4.convs2.0.weight_g', 'dec.resblocks.4.convs2.0.weight_v', 'dec.resblocks.4.convs2.1.bias', 'dec.resblocks.4.convs2.1.weight_g', 'dec.resblocks.4.convs2.1.weight_v', 'dec.resblocks.4.convs2.2.bias', 'dec.resblocks.4.convs2.2.weight_g', 'dec.resblocks.4.convs2.2.weight_v', 'dec.resblocks.5.convs1.0.bias', 'dec.resblocks.5.convs1.0.weight_g', 'dec.resblocks.5.convs1.0.weight_v', 'dec.resblocks.5.convs1.1.bias', 'dec.resblocks.5.convs1.1.weight_g', 'dec.resblocks.5.convs1.1.weight_v', 'dec.resblocks.5.convs1.2.bias', 'dec.resblocks.5.convs1.2.weight_g', 'dec.resblocks.5.convs1.2.weight_v', 'dec.resblocks.5.convs2.0.bias', 'dec.resblocks.5.convs2.0.weight_g', 'dec.resblocks.5.convs2.0.weight_v', 'dec.resblocks.5.convs2.1.bias', 'dec.resblocks.5.convs2.1.weight_g', 'dec.resblocks.5.convs2.1.weight_v', 'dec.resblocks.5.convs2.2.bias', 'dec.resblocks.5.convs2.2.weight_g', 'dec.resblocks.5.convs2.2.weight_v', 'dec.resblocks.6.convs1.0.bias', 'dec.resblocks.6.convs1.0.weight_g', 'dec.resblocks.6.convs1.0.weight_v', 'dec.resblocks.6.convs1.1.bias', 'dec.resblocks.6.convs1.1.weight_g', 'dec.resblocks.6.convs1.1.weight_v', 'dec.resblocks.6.convs1.2.bias', 'dec.resblocks.6.convs1.2.weight_g', 'dec.resblocks.6.convs1.2.weight_v', 'dec.resblocks.6.convs2.0.bias', 'dec.resblocks.6.convs2.0.weight_g', 'dec.resblocks.6.convs2.0.weight_v', 'dec.resblocks.6.convs2.1.bias', 'dec.resblocks.6.convs2.1.weight_g', 'dec.resblocks.6.convs2.1.weight_v', 'dec.resblocks.6.convs2.2.bias', 'dec.resblocks.6.convs2.2.weight_g', 'dec.resblocks.6.convs2.2.weight_v', 'dec.resblocks.7.convs1.0.bias', 'dec.resblocks.7.convs1.0.weight_g', 'dec.resblocks.7.convs1.0.weight_v', 'dec.resblocks.7.convs1.1.bias', 'dec.resblocks.7.convs1.1.weight_g', 'dec.resblocks.7.convs1.1.weight_v', 'dec.resblocks.7.convs1.2.bias', 'dec.resblocks.7.convs1.2.weight_g', 'dec.resblocks.7.convs1.2.weight_v', 'dec.resblocks.7.convs2.0.bias', 'dec.resblocks.7.convs2.0.weight_g', 'dec.resblocks.7.convs2.0.weight_v', 'dec.resblocks.7.convs2.1.bias', 'dec.resblocks.7.convs2.1.weight_g', 'dec.resblocks.7.convs2.1.weight_v', 'dec.resblocks.7.convs2.2.bias', 'dec.resblocks.7.convs2.2.weight_g', 'dec.resblocks.7.convs2.2.weight_v', 'dec.resblocks.8.convs1.0.bias', 'dec.resblocks.8.convs1.0.weight_g', 'dec.resblocks.8.convs1.0.weight_v', 'dec.resblocks.8.convs1.1.bias', 'dec.resblocks.8.convs1.1.weight_g', 'dec.resblocks.8.convs1.1.weight_v', 'dec.resblocks.8.convs1.2.bias', 'dec.resblocks.8.convs1.2.weight_g', 'dec.resblocks.8.convs1.2.weight_v', 'dec.resblocks.8.convs2.0.bias', 'dec.resblocks.8.convs2.0.weight_g', 'dec.resblocks.8.convs2.0.weight_v', 'dec.resblocks.8.convs2.1.bias', 'dec.resblocks.8.convs2.1.weight_g', 'dec.resblocks.8.convs2.1.weight_v', 'dec.resblocks.8.convs2.2.bias', 'dec.resblocks.8.convs2.2.weight_g', 'dec.resblocks.8.convs2.2.weight_v', 'dec.resblocks.9.convs1.0.bias', 'dec.resblocks.9.convs1.0.weight_g', 'dec.resblocks.9.convs1.0.weight_v', 'dec.resblocks.9.convs1.1.bias', 'dec.resblocks.9.convs1.1.weight_g', 'dec.resblocks.9.convs1.1.weight_v', 'dec.resblocks.9.convs1.2.bias', 'dec.resblocks.9.convs1.2.weight_g', 'dec.resblocks.9.convs1.2.weight_v', 'dec.resblocks.9.convs2.0.bias', 'dec.resblocks.9.convs2.0.weight_g', 'dec.resblocks.9.convs2.0.weight_v', 'dec.resblocks.9.convs2.1.bias', 'dec.resblocks.9.convs2.1.weight_g', 'dec.resblocks.9.convs2.1.weight_v', 'dec.resblocks.9.convs2.2.bias', 'dec.resblocks.9.convs2.2.weight_g', 'dec.resblocks.9.convs2.2.weight_v', 'dec.resblocks.10.convs1.0.bias', 'dec.resblocks.10.convs1.0.weight_g', 'dec.resblocks.10.convs1.0.weight_v', 'dec.resblocks.10.convs1.1.bias', 'dec.resblocks.10.convs1.1.weight_g', 'dec.resblocks.10.convs1.1.weight_v', 'dec.resblocks.10.convs1.2.bias', 'dec.resblocks.10.convs1.2.weight_g', 'dec.resblocks.10.convs1.2.weight_v', 'dec.resblocks.10.convs2.0.bias', 'dec.resblocks.10.convs2.0.weight_g', 'dec.resblocks.10.convs2.0.weight_v', 'dec.resblocks.10.convs2.1.bias', 'dec.resblocks.10.convs2.1.weight_g', 'dec.resblocks.10.convs2.1.weight_v', 'dec.resblocks.10.convs2.2.bias', 'dec.resblocks.10.convs2.2.weight_g', 'dec.resblocks.10.convs2.2.weight_v', 'dec.resblocks.11.convs1.0.bias', 'dec.resblocks.11.convs1.0.weight_g', 'dec.resblocks.11.convs1.0.weight_v', 'dec.resblocks.11.convs1.1.bias', 'dec.resblocks.11.convs1.1.weight_g', 'dec.resblocks.11.convs1.1.weight_v', 'dec.resblocks.11.convs1.2.bias', 'dec.resblocks.11.convs1.2.weight_g', 'dec.resblocks.11.convs1.2.weight_v', 'dec.resblocks.11.convs2.0.bias', 'dec.resblocks.11.convs2.0.weight_g', 'dec.resblocks.11.convs2.0.weight_v', 'dec.resblocks.11.convs2.1.bias', 'dec.resblocks.11.convs2.1.weight_g', 'dec.resblocks.11.convs2.1.weight_v', 'dec.resblocks.11.convs2.2.bias', 'dec.resblocks.11.convs2.2.weight_g', 'dec.resblocks.11.convs2.2.weight_v', 'dec.resblocks.12.convs1.0.bias', 'dec.resblocks.12.convs1.0.weight_g', 'dec.resblocks.12.convs1.0.weight_v', 'dec.resblocks.12.convs1.1.bias', 'dec.resblocks.12.convs1.1.weight_g', 'dec.resblocks.12.convs1.1.weight_v', 'dec.resblocks.12.convs1.2.bias', 'dec.resblocks.12.convs1.2.weight_g', 'dec.resblocks.12.convs1.2.weight_v', 'dec.resblocks.12.convs2.0.bias', 'dec.resblocks.12.convs2.0.weight_g', 'dec.resblocks.12.convs2.0.weight_v', 'dec.resblocks.12.convs2.1.bias', 'dec.resblocks.12.convs2.1.weight_g', 'dec.resblocks.12.convs2.1.weight_v', 'dec.resblocks.12.convs2.2.bias', 'dec.resblocks.12.convs2.2.weight_g', 'dec.resblocks.12.convs2.2.weight_v', 'dec.resblocks.13.convs1.0.bias', 'dec.resblocks.13.convs1.0.weight_g', 'dec.resblocks.13.convs1.0.weight_v', 'dec.resblocks.13.convs1.1.bias', 'dec.resblocks.13.convs1.1.weight_g', 'dec.resblocks.13.convs1.1.weight_v', 'dec.resblocks.13.convs1.2.bias', 'dec.resblocks.13.convs1.2.weight_g', 'dec.resblocks.13.convs1.2.weight_v', 'dec.resblocks.13.convs2.0.bias', 'dec.resblocks.13.convs2.0.weight_g', 'dec.resblocks.13.convs2.0.weight_v', 'dec.resblocks.13.convs2.1.bias', 'dec.resblocks.13.convs2.1.weight_g', 'dec.resblocks.13.convs2.1.weight_v', 'dec.resblocks.13.convs2.2.bias', 'dec.resblocks.13.convs2.2.weight_g', 'dec.resblocks.13.convs2.2.weight_v', 'dec.resblocks.14.convs1.0.bias', 'dec.resblocks.14.convs1.0.weight_g', 'dec.resblocks.14.convs1.0.weight_v', 'dec.resblocks.14.convs1.1.bias', 'dec.resblocks.14.convs1.1.weight_g', 'dec.resblocks.14.convs1.1.weight_v', 'dec.resblocks.14.convs1.2.bias', 'dec.resblocks.14.convs1.2.weight_g', 'dec.resblocks.14.convs1.2.weight_v', 'dec.resblocks.14.convs2.0.bias', 'dec.resblocks.14.convs2.0.weight_g', 'dec.resblocks.14.convs2.0.weight_v', 'dec.resblocks.14.convs2.1.bias', 'dec.resblocks.14.convs2.1.weight_g', 'dec.resblocks.14.convs2.1.weight_v', 'dec.resblocks.14.convs2.2.bias', 'dec.resblocks.14.convs2.2.weight_g', 'dec.resblocks.14.convs2.2.weight_v', 'dec.conv_post.bias', 'dec.conv_post.weight_g', 'dec.conv_post.weight_v', 'dec.cond.weight', 'dec.cond.bias', 'enc_q.pre.weight', 'enc_q.pre.bias', 'enc_q.enc.in_layers.0.bias', 'enc_q.enc.in_layers.0.weight_g', 'enc_q.enc.in_layers.0.weight_v', 'enc_q.enc.in_layers.1.bias', 'enc_q.enc.in_layers.1.weight_g', 'enc_q.enc.in_layers.1.weight_v', 'enc_q.enc.in_layers.2.bias', 'enc_q.enc.in_layers.2.weight_g', 'enc_q.enc.in_layers.2.weight_v', 'enc_q.enc.in_layers.3.bias', 'enc_q.enc.in_layers.3.weight_g', 'enc_q.enc.in_layers.3.weight_v', 'enc_q.enc.in_layers.4.bias', 'enc_q.enc.in_layers.4.weight_g', 'enc_q.enc.in_layers.4.weight_v', 'enc_q.enc.in_layers.5.bias', 'enc_q.enc.in_layers.5.weight_g', 'enc_q.enc.in_layers.5.weight_v', 'enc_q.enc.in_layers.6.bias', 'enc_q.enc.in_layers.6.weight_g', 'enc_q.enc.in_layers.6.weight_v', 'enc_q.enc.in_layers.7.bias', 'enc_q.enc.in_layers.7.weight_g', 'enc_q.enc.in_layers.7.weight_v', 'enc_q.enc.in_layers.8.bias', 'enc_q.enc.in_layers.8.weight_g', 'enc_q.enc.in_layers.8.weight_v', 'enc_q.enc.in_layers.9.bias', 'enc_q.enc.in_layers.9.weight_g', 'enc_q.enc.in_layers.9.weight_v', 'enc_q.enc.in_layers.10.bias', 'enc_q.enc.in_layers.10.weight_g', 'enc_q.enc.in_layers.10.weight_v', 'enc_q.enc.in_layers.11.bias', 'enc_q.enc.in_layers.11.weight_g', 'enc_q.enc.in_layers.11.weight_v', 'enc_q.enc.in_layers.12.bias', 'enc_q.enc.in_layers.12.weight_g', 'enc_q.enc.in_layers.12.weight_v', 'enc_q.enc.in_layers.13.bias', 'enc_q.enc.in_layers.13.weight_g', 'enc_q.enc.in_layers.13.weight_v', 'enc_q.enc.in_layers.14.bias', 'enc_q.enc.in_layers.14.weight_g', 'enc_q.enc.in_layers.14.weight_v', 'enc_q.enc.in_layers.15.bias', 'enc_q.enc.in_layers.15.weight_g', 'enc_q.enc.in_layers.15.weight_v', 'enc_q.enc.res_skip_layers.0.bias', 'enc_q.enc.res_skip_layers.0.weight_g', 'enc_q.enc.res_skip_layers.0.weight_v', 'enc_q.enc.res_skip_layers.1.bias', 'enc_q.enc.res_skip_layers.1.weight_g', 'enc_q.enc.res_skip_layers.1.weight_v', 'enc_q.enc.res_skip_layers.2.bias', 'enc_q.enc.res_skip_layers.2.weight_g', 'enc_q.enc.res_skip_layers.2.weight_v', 'enc_q.enc.res_skip_layers.3.bias', 'enc_q.enc.res_skip_layers.3.weight_g', 'enc_q.enc.res_skip_layers.3.weight_v', 'enc_q.enc.res_skip_layers.4.bias', 'enc_q.enc.res_skip_layers.4.weight_g', 'enc_q.enc.res_skip_layers.4.weight_v', 'enc_q.enc.res_skip_layers.5.bias', 'enc_q.enc.res_skip_layers.5.weight_g', 'enc_q.enc.res_skip_layers.5.weight_v', 'enc_q.enc.res_skip_layers.6.bias', 'enc_q.enc.res_skip_layers.6.weight_g', 'enc_q.enc.res_skip_layers.6.weight_v', 'enc_q.enc.res_skip_layers.7.bias', 'enc_q.enc.res_skip_layers.7.weight_g', 'enc_q.enc.res_skip_layers.7.weight_v', 'enc_q.enc.res_skip_layers.8.bias', 'enc_q.enc.res_skip_layers.8.weight_g', 'enc_q.enc.res_skip_layers.8.weight_v', 'enc_q.enc.res_skip_layers.9.bias', 'enc_q.enc.res_skip_layers.9.weight_g', 'enc_q.enc.res_skip_layers.9.weight_v', 'enc_q.enc.res_skip_layers.10.bias', 'enc_q.enc.res_skip_layers.10.weight_g', 'enc_q.enc.res_skip_layers.10.weight_v', 'enc_q.enc.res_skip_layers.11.bias', 'enc_q.enc.res_skip_layers.11.weight_g', 'enc_q.enc.res_skip_layers.11.weight_v', 'enc_q.enc.res_skip_layers.12.bias', 'enc_q.enc.res_skip_layers.12.weight_g', 'enc_q.enc.res_skip_layers.12.weight_v', 'enc_q.enc.res_skip_layers.13.bias', 'enc_q.enc.res_skip_layers.13.weight_g', 'enc_q.enc.res_skip_layers.13.weight_v', 'enc_q.enc.res_skip_layers.14.bias', 'enc_q.enc.res_skip_layers.14.weight_g', 'enc_q.enc.res_skip_layers.14.weight_v', 'enc_q.enc.res_skip_layers.15.bias', 'enc_q.enc.res_skip_layers.15.weight_g', 'enc_q.enc.res_skip_layers.15.weight_v', 'enc_q.enc.cond_layer.bias', 'enc_q.enc.cond_layer.weight_g', 'enc_q.enc.cond_layer.weight_v', 'enc_q.proj.weight', 'enc_q.proj.bias', 'flow.flows.0.pre.weight', 'flow.flows.0.pre.bias', 'flow.flows.0.enc.in_layers.0.bias', 'flow.flows.0.enc.in_layers.0.weight_g', 'flow.flows.0.enc.in_layers.0.weight_v', 'flow.flows.0.enc.in_layers.1.bias', 'flow.flows.0.enc.in_layers.1.weight_g', 'flow.flows.0.enc.in_layers.1.weight_v', 'flow.flows.0.enc.in_layers.2.bias', 'flow.flows.0.enc.in_layers.2.weight_g', 'flow.flows.0.enc.in_layers.2.weight_v', 'flow.flows.0.enc.in_layers.3.bias', 'flow.flows.0.enc.in_layers.3.weight_g', 'flow.flows.0.enc.in_layers.3.weight_v', 'flow.flows.0.enc.res_skip_layers.0.bias', 'flow.flows.0.enc.res_skip_layers.0.weight_g', 'flow.flows.0.enc.res_skip_layers.0.weight_v', 'flow.flows.0.enc.res_skip_layers.1.bias', 'flow.flows.0.enc.res_skip_layers.1.weight_g', 'flow.flows.0.enc.res_skip_layers.1.weight_v', 'flow.flows.0.enc.res_skip_layers.2.bias', 'flow.flows.0.enc.res_skip_layers.2.weight_g', 'flow.flows.0.enc.res_skip_layers.2.weight_v', 'flow.flows.0.enc.res_skip_layers.3.bias', 'flow.flows.0.enc.res_skip_layers.3.weight_g', 'flow.flows.0.enc.res_skip_layers.3.weight_v', 'flow.flows.0.enc.cond_layer.bias', 'flow.flows.0.enc.cond_layer.weight_g', 'flow.flows.0.enc.cond_layer.weight_v', 'flow.flows.0.post.weight', 'flow.flows.0.post.bias', 'flow.flows.2.pre.weight', 'flow.flows.2.pre.bias', 'flow.flows.2.enc.in_layers.0.bias', 'flow.flows.2.enc.in_layers.0.weight_g', 'flow.flows.2.enc.in_layers.0.weight_v', 'flow.flows.2.enc.in_layers.1.bias', 'flow.flows.2.enc.in_layers.1.weight_g', 'flow.flows.2.enc.in_layers.1.weight_v', 'flow.flows.2.enc.in_layers.2.bias', 'flow.flows.2.enc.in_layers.2.weight_g', 'flow.flows.2.enc.in_layers.2.weight_v', 'flow.flows.2.enc.in_layers.3.bias', 'flow.flows.2.enc.in_layers.3.weight_g', 'flow.flows.2.enc.in_layers.3.weight_v', 'flow.flows.2.enc.res_skip_layers.0.bias', 'flow.flows.2.enc.res_skip_layers.0.weight_g', 'flow.flows.2.enc.res_skip_layers.0.weight_v', 'flow.flows.2.enc.res_skip_layers.1.bias', 'flow.flows.2.enc.res_skip_layers.1.weight_g', 'flow.flows.2.enc.res_skip_layers.1.weight_v', 'flow.flows.2.enc.res_skip_layers.2.bias', 'flow.flows.2.enc.res_skip_layers.2.weight_g', 'flow.flows.2.enc.res_skip_layers.2.weight_v', 'flow.flows.2.enc.res_skip_layers.3.bias', 'flow.flows.2.enc.res_skip_layers.3.weight_g', 'flow.flows.2.enc.res_skip_layers.3.weight_v', 'flow.flows.2.enc.cond_layer.bias', 'flow.flows.2.enc.cond_layer.weight_g', 'flow.flows.2.enc.cond_layer.weight_v', 'flow.flows.2.post.weight', 'flow.flows.2.post.bias', 'flow.flows.4.pre.weight', 'flow.flows.4.pre.bias', 'flow.flows.4.enc.in_layers.0.bias', 'flow.flows.4.enc.in_layers.0.weight_g', 'flow.flows.4.enc.in_layers.0.weight_v', 'flow.flows.4.enc.in_layers.1.bias', 'flow.flows.4.enc.in_layers.1.weight_g', 'flow.flows.4.enc.in_layers.1.weight_v', 'flow.flows.4.enc.in_layers.2.bias', 'flow.flows.4.enc.in_layers.2.weight_g', 'flow.flows.4.enc.in_layers.2.weight_v', 'flow.flows.4.enc.in_layers.3.bias', 'flow.flows.4.enc.in_layers.3.weight_g', 'flow.flows.4.enc.in_layers.3.weight_v', 'flow.flows.4.enc.res_skip_layers.0.bias', 'flow.flows.4.enc.res_skip_layers.0.weight_g', 'flow.flows.4.enc.res_skip_layers.0.weight_v', 'flow.flows.4.enc.res_skip_layers.1.bias', 'flow.flows.4.enc.res_skip_layers.1.weight_g', 'flow.flows.4.enc.res_skip_layers.1.weight_v', 'flow.flows.4.enc.res_skip_layers.2.bias', 'flow.flows.4.enc.res_skip_layers.2.weight_g', 'flow.flows.4.enc.res_skip_layers.2.weight_v', 'flow.flows.4.enc.res_skip_layers.3.bias', 'flow.flows.4.enc.res_skip_layers.3.weight_g', 'flow.flows.4.enc.res_skip_layers.3.weight_v', 'flow.flows.4.enc.cond_layer.bias', 'flow.flows.4.enc.cond_layer.weight_g', 'flow.flows.4.enc.cond_layer.weight_v', 'flow.flows.4.post.weight', 'flow.flows.4.post.bias', 'flow.flows.6.pre.weight', 'flow.flows.6.pre.bias', 'flow.flows.6.enc.in_layers.0.bias', 'flow.flows.6.enc.in_layers.0.weight_g', 'flow.flows.6.enc.in_layers.0.weight_v', 'flow.flows.6.enc.in_layers.1.bias', 'flow.flows.6.enc.in_layers.1.weight_g', 'flow.flows.6.enc.in_layers.1.weight_v', 'flow.flows.6.enc.in_layers.2.bias', 'flow.flows.6.enc.in_layers.2.weight_g', 'flow.flows.6.enc.in_layers.2.weight_v', 'flow.flows.6.enc.in_layers.3.bias', 'flow.flows.6.enc.in_layers.3.weight_g', 'flow.flows.6.enc.in_layers.3.weight_v', 'flow.flows.6.enc.res_skip_layers.0.bias', 'flow.flows.6.enc.res_skip_layers.0.weight_g', 'flow.flows.6.enc.res_skip_layers.0.weight_v', 'flow.flows.6.enc.res_skip_layers.1.bias', 'flow.flows.6.enc.res_skip_layers.1.weight_g', 'flow.flows.6.enc.res_skip_layers.1.weight_v', 'flow.flows.6.enc.res_skip_layers.2.bias', 'flow.flows.6.enc.res_skip_layers.2.weight_g', 'flow.flows.6.enc.res_skip_layers.2.weight_v', 'flow.flows.6.enc.res_skip_layers.3.bias', 'flow.flows.6.enc.res_skip_layers.3.weight_g', 'flow.flows.6.enc.res_skip_layers.3.weight_v', 'flow.flows.6.enc.cond_layer.bias', 'flow.flows.6.enc.cond_layer.weight_g', 'flow.flows.6.enc.cond_layer.weight_v', 'flow.flows.6.post.weight', 'flow.flows.6.post.bias', 'f0_decoder.prenet.weight', 'f0_decoder.prenet.bias', 'f0_decoder.decoder.self_attn_layers.0.conv_q.weight', 'f0_decoder.decoder.self_attn_layers.0.conv_q.bias', 'f0_decoder.decoder.self_attn_layers.0.conv_k.weight', 'f0_decoder.decoder.self_attn_layers.0.conv_k.bias', 'f0_decoder.decoder.self_attn_layers.0.conv_v.weight', 'f0_decoder.decoder.self_attn_layers.0.conv_v.bias', 'f0_decoder.decoder.self_attn_layers.0.conv_o.weight', 'f0_decoder.decoder.self_attn_layers.0.conv_o.bias', 'f0_decoder.decoder.self_attn_layers.1.conv_q.weight', 'f0_decoder.decoder.self_attn_layers.1.conv_q.bias', 'f0_decoder.decoder.self_attn_layers.1.conv_k.weight', 'f0_decoder.decoder.self_attn_layers.1.conv_k.bias', 'f0_decoder.decoder.self_attn_layers.1.conv_v.weight', 'f0_decoder.decoder.self_attn_layers.1.conv_v.bias', 'f0_decoder.decoder.self_attn_layers.1.conv_o.weight', 'f0_decoder.decoder.self_attn_layers.1.conv_o.bias', 'f0_decoder.decoder.self_attn_layers.2.conv_q.weight', 'f0_decoder.decoder.self_attn_layers.2.conv_q.bias', 'f0_decoder.decoder.self_attn_layers.2.conv_k.weight', 'f0_decoder.decoder.self_attn_layers.2.conv_k.bias', 'f0_decoder.decoder.self_attn_layers.2.conv_v.weight', 'f0_decoder.decoder.self_attn_layers.2.conv_v.bias', 'f0_decoder.decoder.self_attn_layers.2.conv_o.weight', 'f0_decoder.decoder.self_attn_layers.2.conv_o.bias', 'f0_decoder.decoder.self_attn_layers.3.conv_q.weight', 'f0_decoder.decoder.self_attn_layers.3.conv_q.bias', 'f0_decoder.decoder.self_attn_layers.3.conv_k.weight', 'f0_decoder.decoder.self_attn_layers.3.conv_k.bias', 'f0_decoder.decoder.self_attn_layers.3.conv_v.weight', 'f0_decoder.decoder.self_attn_layers.3.conv_v.bias', 'f0_decoder.decoder.self_attn_layers.3.conv_o.weight', 'f0_decoder.decoder.self_attn_layers.3.conv_o.bias', 'f0_decoder.decoder.self_attn_layers.4.conv_q.weight', 'f0_decoder.decoder.self_attn_layers.4.conv_q.bias', 'f0_decoder.decoder.self_attn_layers.4.conv_k.weight', 'f0_decoder.decoder.self_attn_layers.4.conv_k.bias', 'f0_decoder.decoder.self_attn_layers.4.conv_v.weight', 'f0_decoder.decoder.self_attn_layers.4.conv_v.bias', 'f0_decoder.decoder.self_attn_layers.4.conv_o.weight', 'f0_decoder.decoder.self_attn_layers.4.conv_o.bias', 'f0_decoder.decoder.self_attn_layers.5.conv_q.weight', 'f0_decoder.decoder.self_attn_layers.5.conv_q.bias', 'f0_decoder.decoder.self_attn_layers.5.conv_k.weight', 'f0_decoder.decoder.self_attn_layers.5.conv_k.bias', 'f0_decoder.decoder.self_attn_layers.5.conv_v.weight', 'f0_decoder.decoder.self_attn_layers.5.conv_v.bias', 'f0_decoder.decoder.self_attn_layers.5.conv_o.weight', 'f0_decoder.decoder.self_attn_layers.5.conv_o.bias', 'f0_decoder.decoder.norm_layers_0.0.gamma', 'f0_decoder.decoder.norm_layers_0.0.beta', 'f0_decoder.decoder.norm_layers_0.1.gamma', 'f0_decoder.decoder.norm_layers_0.1.beta', 'f0_decoder.decoder.norm_layers_0.2.gamma', 'f0_decoder.decoder.norm_layers_0.2.beta', 'f0_decoder.decoder.norm_layers_0.3.gamma', 'f0_decoder.decoder.norm_layers_0.3.beta', 'f0_decoder.decoder.norm_layers_0.4.gamma', 'f0_decoder.decoder.norm_layers_0.4.beta', 'f0_decoder.decoder.norm_layers_0.5.gamma', 'f0_decoder.decoder.norm_layers_0.5.beta', 'f0_decoder.decoder.ffn_layers.0.conv_1.weight', 'f0_decoder.decoder.ffn_layers.0.conv_1.bias', 'f0_decoder.decoder.ffn_layers.0.conv_2.weight', 'f0_decoder.decoder.ffn_layers.0.conv_2.bias', 'f0_decoder.decoder.ffn_layers.1.conv_1.weight', 'f0_decoder.decoder.ffn_layers.1.conv_1.bias', 'f0_decoder.decoder.ffn_layers.1.conv_2.weight', 'f0_decoder.decoder.ffn_layers.1.conv_2.bias', 'f0_decoder.decoder.ffn_layers.2.conv_1.weight', 'f0_decoder.decoder.ffn_layers.2.conv_1.bias', 'f0_decoder.decoder.ffn_layers.2.conv_2.weight', 'f0_decoder.decoder.ffn_layers.2.conv_2.bias', 'f0_decoder.decoder.ffn_layers.3.conv_1.weight', 'f0_decoder.decoder.ffn_layers.3.conv_1.bias', 'f0_decoder.decoder.ffn_layers.3.conv_2.weight', 'f0_decoder.decoder.ffn_layers.3.conv_2.bias', 'f0_decoder.decoder.ffn_layers.4.conv_1.weight', 'f0_decoder.decoder.ffn_layers.4.conv_1.bias', 'f0_decoder.decoder.ffn_layers.4.conv_2.weight', 'f0_decoder.decoder.ffn_layers.4.conv_2.bias', 'f0_decoder.decoder.ffn_layers.5.conv_1.weight', 'f0_decoder.decoder.ffn_layers.5.conv_1.bias', 'f0_decoder.decoder.ffn_layers.5.conv_2.weight', 'f0_decoder.decoder.ffn_layers.5.conv_2.bias', 'f0_decoder.decoder.norm_layers_1.0.gamma', 'f0_decoder.decoder.norm_layers_1.0.beta', 'f0_decoder.decoder.norm_layers_1.1.gamma', 'f0_decoder.decoder.norm_layers_1.1.beta', 'f0_decoder.decoder.norm_layers_1.2.gamma', 'f0_decoder.decoder.norm_layers_1.2.beta', 'f0_decoder.decoder.norm_layers_1.3.gamma', 'f0_decoder.decoder.norm_layers_1.3.beta', 'f0_decoder.decoder.norm_layers_1.4.gamma', 'f0_decoder.decoder.norm_layers_1.4.beta', 'f0_decoder.decoder.norm_layers_1.5.gamma', 'f0_decoder.decoder.norm_layers_1.5.beta', 'f0_decoder.proj.weight', 'f0_decoder.proj.bias', 'f0_decoder.f0_prenet.weight', 'f0_decoder.f0_prenet.bias', 'f0_decoder.cond.weight', 'f0_decoder.cond.bias', 'emb_uv.weight'])
> checkpoint_dict["optimizer"].keys()
dict_keys(['state', 'param_groups'])


```









```python
> model.net_g_ms
SynthesizerTrn(
  (emb_g): Embedding(1, 768)
  (emb_vol): Linear(in_features=1, out_features=192, bias=True)
  (pre): Conv1d(768, 192, kernel_size=(5,), stride=(1,), padding=(2,))
  (enc_p): TextEncoder(
    (proj): Conv1d(192, 384, kernel_size=(1,), stride=(1,))
    (f0_emb): Embedding(256, 192)
    (enc_): Encoder(
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
  )
  (dec): Generator(
    (f0_upsamp): Upsample(scale_factor=512.0, mode='nearest')
    (m_source): SourceModuleHnNSF(
      (l_sin_gen): SineGen()
      (l_linear): Linear(in_features=9, out_features=1, bias=True)
      (l_tanh): Tanh()
    )
    (noise_convs): ModuleList(
      (0): Conv1d(1, 256, kernel_size=(128,), stride=(64,), padding=(32,))
      (1): Conv1d(1, 128, kernel_size=(16,), stride=(8,), padding=(4,))
      (2): Conv1d(1, 64, kernel_size=(8,), stride=(4,), padding=(2,))
      (3): Conv1d(1, 32, kernel_size=(4,), stride=(2,), padding=(1,))
      (4): Conv1d(1, 16, kernel_size=(1,), stride=(1,))
    )
    (conv_pre): Conv1d(192, 512, kernel_size=(7,), stride=(1,), padding=(3,))
    (ups): ModuleList(
      (0): ConvTranspose1d(512, 256, kernel_size=(16,), stride=(8,), padding=(4,))
      (1): ConvTranspose1d(256, 128, kernel_size=(16,), stride=(8,), padding=(4,))
      (2): ConvTranspose1d(128, 64, kernel_size=(4,), stride=(2,), padding=(1,))
      (3): ConvTranspose1d(64, 32, kernel_size=(4,), stride=(2,), padding=(1,))
      (4): ConvTranspose1d(32, 16, kernel_size=(4,), stride=(2,), padding=(1,))
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
    (conv_post): Conv1d(16, 1, kernel_size=(7,), stride=(1,), padding=(3,))
    (cond): Conv1d(768, 512, kernel_size=(1,), stride=(1,))
  )
  (enc_q): Encoder(
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
      (cond_layer): Conv1d(768, 6144, kernel_size=(1,), stride=(1,))
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
          (cond_layer): Conv1d(768, 1536, kernel_size=(1,), stride=(1,))
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
          (cond_layer): Conv1d(768, 1536, kernel_size=(1,), stride=(1,))
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
          (cond_layer): Conv1d(768, 1536, kernel_size=(1,), stride=(1,))
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
          (cond_layer): Conv1d(768, 1536, kernel_size=(1,), stride=(1,))
        )
        (post): Conv1d(192, 96, kernel_size=(1,), stride=(1,))
      )
      (7): Flip()
    )
  )
  (f0_decoder): F0Decoder(
    (prenet): Conv1d(192, 192, kernel_size=(3,), stride=(1,), padding=(1,))
    (decoder): FFT(
      (drop): Dropout(p=0.1, inplace=False)
      (self_attn_layers): ModuleList(
        (0-5): 6 x MultiHeadAttention(
          (conv_q): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (conv_k): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (conv_v): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (conv_o): Conv1d(192, 192, kernel_size=(1,), stride=(1,))
          (drop): Dropout(p=0.1, inplace=False)
        )
      )
      (norm_layers_0): ModuleList(
        (0-5): 6 x LayerNorm()
      )
      (ffn_layers): ModuleList(
        (0-5): 6 x FFN(
          (conv_1): Conv1d(192, 768, kernel_size=(3,), stride=(1,))
          (conv_2): Conv1d(768, 192, kernel_size=(3,), stride=(1,))
          (drop): Dropout(p=0.1, inplace=False)
        )
      )
      (norm_layers_1): ModuleList(
        (0-5): 6 x LayerNorm()
      )
    )
    (proj): Conv1d(192, 1, kernel_size=(1,), stride=(1,))
    (f0_prenet): Conv1d(1, 192, kernel_size=(3,), stride=(1,), padding=(1,))
    (cond): Conv1d(768, 192, kernel_size=(1,), stride=(1,))
  )
  (emb_uv): Embedding(2, 192)
)
```











