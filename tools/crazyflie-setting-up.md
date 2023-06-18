# CrazyFlie Setting Up

## AI Deck

### Environment setup

Ubuntu 22.04

#### Set up KVM

Follow the instructions here to set up KVM

{% embed url="https://docs.docker.com/desktop/install/linux-install/#kvm-virtualization-support" %}

<figure><img src="../.gitbook/assets/image (18) (1).png" alt=""><figcaption></figcaption></figure>

#### Install Docker

Follow the instructions here

{% embed url="https://docs.docker.com/desktop/install/ubuntu/#install-docker-desktop" %}

If there is error about "docker-desktop : Depends: docker-ce-cli but it is not installable", follow solutions here

{% embed url="https://stackoverflow.com/questions/72299444/docker-desktop-doesnt-install-saying-docker-ce-cli-not-installable" %}



#### Launch Docker

```bash
systemctl --user start docker-desktop
```

<figure><img src="../.gitbook/assets/image (14) (3).png" alt=""><figcaption></figcaption></figure>



hello world should be able to run

```bash
docker run hello-world
```

<figure><img src="../.gitbook/assets/image (13) (1).png" alt=""><figcaption></figcaption></figure>





sudo apt install libtool pkg-config autoconf automake texinfo
