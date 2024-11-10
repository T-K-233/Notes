---
cover: ../../.gitbook/assets/image (227).png
coverY: 143.150498381877
layout:
  cover:
    visible: true
    size: full
  title:
    visible: true
  description:
    visible: true
  tableOfContents:
    visible: true
  outline:
    visible: true
  pagination:
    visible: true
---

# Install Cursor the AI Editor on Ubuntu 22.04/24.04

<figure><img src="../../.gitbook/assets/image (227).png" alt=""><figcaption></figcaption></figure>

## Download

Download cursor from [here](https://www.cursor.com/).



## Grant executable permission

```bash
chmod +x ~/Downloads/cursor-0.41.3x86_64.AppImage
```



## Install FUSE Dependency

```bash
sudo apt install libfuse2
```



## Adding cursor to the system

Move cursor to system directory

```bash
sudo mkdir /opt/cursor/
sudo mv ~/Downloads/cursor-0.41.3x86_64.AppImage /opt/cursor/cursor.appimage
sudo mv ~/Downloads/cursor.png /opt/cursor/cursor.png
```



Create a new cursor.desktop application description file and input the following content

```bash
sudo nano /usr/share/applications/cursor.desktop
```

{% code title="cursor.desktop" %}
```bash
[Desktop Entry]
Name=Cursor
Exec=/opt/cursor/cursor.appimage
Icon=/opt/cursor/cursor.png
Type=Application
Categories=Development;

```
{% endcode %}



For icon, this image can be used, taken from [cursor GitHub repo](https://github.com/getcursor/cursor).

<figure><img src="../../.gitbook/assets/cursor.png" alt=""><figcaption></figcaption></figure>



## Set up apparmor to allow executing cursor command at command line

[https://github.com/getcursor/cursor/issues/844#issuecomment-2080288303](https://github.com/getcursor/cursor/issues/844#issuecomment-2080288303)



```bash
sudo nano /etc/apparmor.d/cursor-appimage
```

{% code title="cursor-appimage" %}
```bash
# This profile allows everything and only exists to give the
# application a name instead of having the label "unconfined"

abi <abi/4.0>,
include <tunables/global>

profile cursor /opt/cursor/cursor*.appimage flags=(unconfined) {
  userns,

  # Site-specific additions and overrides.  See local/README for details.
  include if exists <local/cursor>
}
```
{% endcode %}



And then run

```bash
sudo apparmor_parser -r /etc/apparmor.d/cursor-appimage
```





