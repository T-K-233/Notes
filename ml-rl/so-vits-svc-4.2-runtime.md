# so-vits-svc 4.2

## Reference

[https://github.com/SUC-DriverOld/so-vits-svc-Chinese-Detaild-Documents](https://github.com/SUC-DriverOld/so-vits-svc-Chinese-Detaild-Documents)



## Environment

Linux (millennium-A24)

Python 3.10.13



```
Python 3.10.11 (main, Apr  5 2023, 14:15:10) [GCC 9.4.0]
Numpy 1.22.4
numba 0.57.0
PyTorch 1.13.1+cu117
fairseq 0.12.2
librosa 0.10.0.post2
```





## Dataset Preparation

### Requirements for Dataset

Minimum: 100 entries of 5\~15s audio clips

Normal: 1.5 hours of audio



Sampling rate: 48000

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



### Preprocess

Put wav files in dataset\_raw/\<speaker>/\*.wav



```bash
python resample.py --skip_loudnorm
```



```bash
python preprocess_flist_config.py --speech_encoder vec768l12 --vol_aug
```



```bash
python preprocess_hubert_f0.py --f0_predictor dio --use_diff
```



{% embed url="https://github.com/Anjok07/ultimatevocalremovergui" %}

