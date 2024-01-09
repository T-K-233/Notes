# Protobuf

## Install Protobuf

this does not work:

```bash
sudo apt install libprotobuf-dev protobuf-compiler
```



also do not work:



follow this to install protobuf C++ compiler

{% embed url="https://github.com/protocolbuffers/protobuf/blob/v25.1/src/README.md" %}



If Bazel cannot be located, do following, reference [this thread](https://stackoverflow.com/questions/61982500/cannot-install-bazel-on-ubuntu-20-04-invalid-expkeysig), we are using david's method

```bash
sudo apt install apt-transport-https curl gnupg
curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor >bazel-archive-keyring.gpg
sudo mv bazel-archive-keyring.gpg /usr/share/keyrings
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list
```

```bash
bazel build :protoc :protobuf
```





```
sudo cp bazel-bin/protoc /usr/local/bin
```



Verify install correctly:

```bash
$ protoc --version
libprotoc 26.0

$ whereis protoc
protoc: /usr/local/bin/protoc
```



Also download the release here and move the `include` folder to `/usr/local/include/google`

{% embed url="https://github.com/protocolbuffers/protobuf/releases" %}

DOES NOT WORK:

```bash
sudo apt install protobuf-compiler
```



Download the prebuilt file, copy the include/google folder to /usr/include

```
sudo cp -r ./include/google/ /usr/include/
```

Also need to download source, and copy the `protobuf-25.1/src/google/protobuf/stubs` folder to there

```bash
cd ~/Downloads/protobuf-25.1/src/google/protobuf/
sudo cp -r ./stubs/ /usr/include/google/protobuf/
```





FINALLY WORKS

```
sudo apt install autogen autoconf libtool
```





```
git clone https://github.com/protocolbuffers/protobuf.git
cd protobuf
git checkout v3.6.1
```

i think this works up to 3.12.3, havent tried



instruction is [here](https://github.com/protocolbuffers/protobuf/blob/v3.6.1/src/README.md)

```
git submodule update --init --recursive
./autogen.sh

./configure --prefix=/usr
make -j8
make check
sudo make install
sudo ldconfig # refresh shared library cache.
```













