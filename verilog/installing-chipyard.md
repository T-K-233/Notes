# Installing Chipyard

Ubuntu 22.04.1 LTS





1. Install conda

{% embed url="https://github.com/conda-forge/miniforge/releases" %}

<figure><img src="../.gitbook/assets/image (96) (1).png" alt=""><figcaption></figcaption></figure>

```bash
tk@TK-Framework-Laptop:~/Downloads$ ./Mambaforge-4.14.0-0-Linux-x86_64.sh 
```





<figure><img src="../.gitbook/assets/image (4) (3).png" alt=""><figcaption></figcaption></figure>



```
conda install -n base conda-lock
conda activate base
```







```
git clone https://github.com/ucb-bar/chipyard.git

cd chipyard

git checkout stable

```



```
./build-setup.sh riscv-tools
```

<figure><img src="../.gitbook/assets/image (3) (2).png" alt=""><figcaption></figcaption></figure>







