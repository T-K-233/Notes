# Executorch on ARM

{% embed url="https://pytorch.org/executorch/stable/executorch-arm-delegate-tutorial.html" %}



#### Download and Set Up the Corstone-300 FVP

```bash
mkdir ./arm-example/
cd ./arm-example/
```



```bash
mkdir ./FVP/
cd ./FVP/

curl \
    --output FVP_cs300.tgz \
    'https://developer.arm.com/-/media/Arm%20Developer%20Community/Downloads/OSS/FVP/Corstone-300/FVP_Corstone_SSE-300_11.22_20_Linux64.tgz?rev=018659bd574f4e7b95fa647e7836ccf4&hash=22A79103C6FA5FFA7AFF3BE0447F3FF9'

```



```bash
tar -xvzf ./FVP_cs300.tgz
```



```bash
./FVP_Corstone_SSE-300.sh          \
   --i-agree-to-the-contained-eula \
   --force                         \
   --destination ./                \
   --quiet                         \
   --no-interactive
```



{% code overflow="wrap" %}
```bash
export PATH=${PATH}:/scratch/tk/Desktop/riscv-vector/executorch/arm-example/FVP/models/Linux64_GCC-9.3
```
{% endcode %}



If setup is successful, running the following command should return nothing

```bash
FVP_Corstone_SSE-300_Ethos-U55 --help
```



#### Download and Install the Arm GNU AArch32 Bare-Metal Toolchain

```bash
cd ./arm-example/
mkdir ./arm-gnu-toolchain/

curl \
    --output gcc.tar.xz \
    'https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu/12.3.rel1/binrel/arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi.tar.xz'
```



```bash
tar -xf gcc.tar.xz
```



```bash
export PATH=${PATH}:/scratch/tk/Desktop/riscv-vector/executorch/arm-example/arm-gnu-toolchain/arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi/bin
```



```bash
arm-none-eabi-gcc --help
```



#### Setup the Arm Ethos-U Software Development



```bash
git clone https://review.mlplatform.org/ml/ethos-u/ethos-u
cd ethos-u
```



```bash
./fetch_externals.py fetch
```



```bash
git clone https://review.mlplatform.org/ml/ethos-u/ethos-u-vela
```







```bash
cd ethos-u # this is the top level Ethos-U software directory

# Let's patch core_platform repo
cd core_platform
git reset --hard 204210b1074071532627da9dc69950d058a809f4
git am -3 ../../../executorch/examples/arm/ethos-u-setup/core_platform/patches/*.patch
cd ../.. # To the top-level development dir
```





```bash
cd ethos-u-vela
pip install .
```





```bash
git clone https://review.mlplatform.org/tosa/reference_model -b v0.80
cd reference_model
git submodule update --init --recursive
mkdir -p build
cd build
cmake ..
n=$(nproc)
make -j"$((n - 5))"
cd reference_model # Within the build directory
# Add tosa_reference_model to the path
export PATH=${PATH}:`pwd`
```











