# 深入浅出理解 So-VITS-SVC 原理





## Feature extraction stage



### f0 extraction

**DIO (Distributed Inline Filtering with Overlap)**&#x20;

An algorithm for fundamental frequency (F0) estimation in speech signals. It uses a two-step process: first, it applies a low-pass filter to the signal to extract the harmonic structure, and then it uses a peak-picking algorithm to estimate the F0.



**CREPE (Convolutional REctified Phase Expressions)**

A deep learning-based pitch detection algorithm that uses a convolutional neural network (CNN) to extract pitch features from the audio signal.



**Harvest (Harmonic Product Spectrum)**

An algorithm for pitch detection that works by computing the harmonic product spectrum of the audio signal, which is a spectral representation that emphasizes harmonic frequencies.



**Parselmouth**

A Python library for Praat, which is a software tool commonly used in phonetics research. Parselmouth provides an interface for accessing Praat's functionality in Python code, including functions for analyzing and synthesizing speech signals, as well as extracting features like pitch, formants, and spectrograms.





## Reference

{% embed url="https://blog.csdn.net/u014281392/article/details/131422183" %}

{% embed url="https://github.com/voicepaw/so-vits-svc-fork/discussions/318#discussioncomment-5777556" %}

