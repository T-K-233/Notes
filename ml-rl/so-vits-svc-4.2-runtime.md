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







