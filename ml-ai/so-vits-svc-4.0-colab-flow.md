# so-vits-svc 4.0: CoLab Flow

## Reference

【so-vits-svc】手把手教你老婆唱歌 [https://www.bilibili.com/video/BV1vM4y1S7zB/](https://www.bilibili.com/video/BV1vM4y1S7zB/)

soVITS3.0数据集准备 [https://www.bilibili.com/read/cv20514221](https://www.bilibili.com/read/cv20514221)

[https://github.com/innnky/so-vits-svc/tree/4.0](https://github.com/innnky/so-vits-svc/tree/4.0)



## Colab Notebook

{% embed url="https://colab.research.google.com/drive/1k_ba8SBvj-jVnwE5Fxr-ucxoGiGt40TE?usp=sharing" %}



## 1. Preparing Training Data

### Dataset Requirements

* 60-100 slices of audio, in .wav format
* each slice should be around 4-8 seconds
* Sample rate should be 44100 Hz



### Training Data

This radio story series of Majo no Tabitabi is a decent training dataset:

[https://www.bilibili.com/video/BV1d54y1m7cK/](https://www.bilibili.com/video/BV1d54y1m7cK/)



To download the audio, use [JiJiDown](https://www.jijidown.com/).

<figure><img src="../.gitbook/assets/image (2).png" alt=""><figcaption></figcaption></figure>

### Data Preparation

We need to assemble the audio pieces together with Adobe Pr Pro

At the beginning and end of each episode, the volume of the background music is raised. For the best quality, we will trim those sections out.



Also, we can observe that the BGM volume never raise above **-30 dB**. We will since use this value as the noise floor later on.

<figure><img src="../.gitbook/assets/image (1).png" alt=""><figcaption></figcaption></figure>

Audio export settings:

<figure><img src="../.gitbook/assets/image (4).png" alt=""><figcaption></figcaption></figure>

Then, we need to slice the data into pieces. To do that, we can use [openvpi/audio-slicer](https://github.com/openvpi/audio-slicer).

```python
import numpy as np


# This function is obtained from librosa.
def get_rms(
    y,
    *,
    frame_length=2048,
    hop_length=512,
    pad_mode="constant",
):
    padding = (int(frame_length // 2), int(frame_length // 2))
    y = np.pad(y, padding, mode=pad_mode)

    axis = -1
    # put our new within-frame axis at the end for now
    out_strides = y.strides + tuple([y.strides[axis]])
    # Reduce the shape on the framing axis
    x_shape_trimmed = list(y.shape)
    x_shape_trimmed[axis] -= frame_length - 1
    out_shape = tuple(x_shape_trimmed) + tuple([frame_length])
    xw = np.lib.stride_tricks.as_strided(
        y, shape=out_shape, strides=out_strides
    )
    if axis < 0:
        target_axis = axis - 1
    else:
        target_axis = axis + 1
    xw = np.moveaxis(xw, -1, target_axis)
    # Downsample along the target axis
    slices = [slice(None)] * xw.ndim
    slices[axis] = slice(0, None, hop_length)
    x = xw[tuple(slices)]

    # Calculate power
    power = np.mean(np.abs(x) ** 2, axis=-2, keepdims=True)

    return np.sqrt(power)


class Slicer:
    def __init__(self,
                 sr: int,
                 threshold: float = -40.,
                 min_length: int = 5000,
                 min_interval: int = 300,
                 hop_size: int = 20,
                 max_sil_kept: int = 5000):
        if not min_length >= min_interval >= hop_size:
            raise ValueError('The following condition must be satisfied: min_length >= min_interval >= hop_size')
        if not max_sil_kept >= hop_size:
            raise ValueError('The following condition must be satisfied: max_sil_kept >= hop_size')
        min_interval = sr * min_interval / 1000
        self.threshold = 10 ** (threshold / 20.)
        self.hop_size = round(sr * hop_size / 1000)
        self.win_size = min(round(min_interval), 4 * self.hop_size)
        self.min_length = round(sr * min_length / 1000 / self.hop_size)
        self.min_interval = round(min_interval / self.hop_size)
        self.max_sil_kept = round(sr * max_sil_kept / 1000 / self.hop_size)

    def _apply_slice(self, waveform, begin, end):
        if len(waveform.shape) > 1:
            return waveform[:, begin * self.hop_size: min(waveform.shape[1], end * self.hop_size)]
        else:
            return waveform[begin * self.hop_size: min(waveform.shape[0], end * self.hop_size)]

    # @timeit
    def slice(self, waveform):
        if len(waveform.shape) > 1:
            samples = waveform.mean(axis=0)
        else:
            samples = waveform
        if samples.shape[0] <= self.min_length:
            return [waveform]
        rms_list = get_rms(y=samples, frame_length=self.win_size, hop_length=self.hop_size).squeeze(0)
        sil_tags = []
        silence_start = None
        clip_start = 0
        for i, rms in enumerate(rms_list):
            # Keep looping while frame is silent.
            if rms < self.threshold:
                # Record start of silent frames.
                if silence_start is None:
                    silence_start = i
                continue
            # Keep looping while frame is not silent and silence start has not been recorded.
            if silence_start is None:
                continue
            # Clear recorded silence start if interval is not enough or clip is too short
            is_leading_silence = silence_start == 0 and i > self.max_sil_kept
            need_slice_middle = i - silence_start >= self.min_interval and i - clip_start >= self.min_length
            if not is_leading_silence and not need_slice_middle:
                silence_start = None
                continue
            # Need slicing. Record the range of silent frames to be removed.
            if i - silence_start <= self.max_sil_kept:
                pos = rms_list[silence_start: i + 1].argmin() + silence_start
                if silence_start == 0:
                    sil_tags.append((0, pos))
                else:
                    sil_tags.append((pos, pos))
                clip_start = pos
            elif i - silence_start <= self.max_sil_kept * 2:
                pos = rms_list[i - self.max_sil_kept: silence_start + self.max_sil_kept + 1].argmin()
                pos += i - self.max_sil_kept
                pos_l = rms_list[silence_start: silence_start + self.max_sil_kept + 1].argmin() + silence_start
                pos_r = rms_list[i - self.max_sil_kept: i + 1].argmin() + i - self.max_sil_kept
                if silence_start == 0:
                    sil_tags.append((0, pos_r))
                    clip_start = pos_r
                else:
                    sil_tags.append((min(pos_l, pos), max(pos_r, pos)))
                    clip_start = max(pos_r, pos)
            else:
                pos_l = rms_list[silence_start: silence_start + self.max_sil_kept + 1].argmin() + silence_start
                pos_r = rms_list[i - self.max_sil_kept: i + 1].argmin() + i - self.max_sil_kept
                if silence_start == 0:
                    sil_tags.append((0, pos_r))
                else:
                    sil_tags.append((pos_l, pos_r))
                clip_start = pos_r
            silence_start = None
        # Deal with trailing silence.
        total_frames = rms_list.shape[0]
        if silence_start is not None and total_frames - silence_start >= self.min_interval:
            silence_end = min(total_frames, silence_start + self.max_sil_kept)
            pos = rms_list[silence_start: silence_end + 1].argmin() + silence_start
            sil_tags.append((pos, total_frames + 1))
        # Apply and return slices.
        if len(sil_tags) == 0:
            return [waveform]
        else:
            chunks = []
            if sil_tags[0][0] > 0:
                chunks.append(self._apply_slice(waveform, 0, sil_tags[0][0]))
            for i in range(len(sil_tags) - 1):
                chunks.append(self._apply_slice(waveform, sil_tags[i][1], sil_tags[i + 1][0]))
            if sil_tags[-1][1] < total_frames:
                chunks.append(self._apply_slice(waveform, sil_tags[-1][1], total_frames))
            return chunks


def main():
    import os
    import shutil
    import librosa
    import soundfile
    
    IN_FILENAME = "Elaina_TrainingSequence_1hr.wav"
    OUT_FILE_DIR = "Elaina"

    print("creating directory...")
    shutil.rmtree(OUT_FILE_DIR, ignore_errors=True)
    os.makedirs(OUT_FILE_DIR)

    print("loading audio...")
    audio, sr = librosa.load(IN_FILENAME, sr=None, mono=False)
    slicer = Slicer(
        sr=sr,
        
        # The RMS threshold presented in dB. Areas where all RMS values are below this threshold
        # will be regarded as silence. Increase this value if your audio is noisy. 
        threshold=-30,

        # The minimum length required for each sliced audio clip, presented in milliseconds.
        min_length=4000,

        # The minimum length for a silence part to be sliced, presented in milliseconds. Set
        # this value smaller if your audio contains only short breaks. The smaller this value is,
        # the more sliced audio clips this script is likely to generate. Note that this value must
        # be smaller than min_length and larger than hop_size. 
        min_interval=500,

        # Length of each RMS frame, presented in milliseconds. Increasing this value will increase
        # the precision of slicing, but will slow down the process.
        hop_size=5,

        # The maximum silence length kept around the sliced audio, presented in milliseconds. Adjust
        # this value according to your needs. Note that setting this value does not mean that
        # silence parts in the sliced audio have exactly the given length. The algorithm will search
        # for the best position to slice, as described above. 
        max_sil_kept=500
    )
    print("slicing...")
    chunks = slicer.slice(audio)

    print("writing results...")
    for i, chunk in enumerate(chunks):
        if len(chunk.shape) > 1:
            # Swap axes if the audio is stereo.
            chunk = chunk.T
        # Save sliced audio files with soundfile.
        soundfile.write(os.path.join(OUT_FILE_DIR, f"seq_{i}.wav"), chunk, sr)

if __name__ == "__main__":
    main()

```



Open a few audio files to verify that the audio has been sliced correctly.

We can also enable the "Length" property in the file browser to get a sense of the duration of each slice. To do so, select Sort by -> More... and click Length in the right-click menu.

<figure><img src="../.gitbook/assets/image (3).png" alt=""><figcaption></figcaption></figure>

Finally, zip the dataset.

<figure><img src="../.gitbook/assets/image (2) (3).png" alt=""><figcaption></figcaption></figure>

## 2. Setting up the training environment







