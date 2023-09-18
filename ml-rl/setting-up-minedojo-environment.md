# Setting up MineDojo Environment

Following tutorial [here](https://docs.minedojo.org/sections/getting\_started/install.html).

##

## Installation

Install JDK 8

```bash
sudo apt update -y
sudo apt install -y software-properties-common
sudo add-apt-repository ppa:openjdk-r/ppa
sudo apt update -y
sudo apt install -y openjdk-8-jdk
```



Switch Java version

```bash
sudo update-alternatives --config java
```



Install MineDojo

```bash
pip install minedojo
```
