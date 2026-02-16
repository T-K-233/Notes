# Performance Testing of Jetson Devices



Install real-time kernel

{% embed url="https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/SD/Kernel/RealTimeKernel.html" %}









```
sudo cyclictest -t 8 -D 2h --policy=fifo
```

```
sudo jetson_clocks
stress --cpu 8 --io 8 --vm 8
```



Compare MAX latency between RT and non-RT kernel.

{% embed url="https://forums.developer.nvidia.com/t/jetson-orin-agx-preempt-rt-rt-test/330785" %}







