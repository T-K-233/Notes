# Install Blender on Ubuntu 22.04



Blender installed from snap software store creates a read-only directory for the Python dependency, which renders it hard to integrate with custom scripting tools.&#x20;

To enable Blender scripting capabilities, we need to install Blender from the official tar.xz package.



## Download Blender from Official Site

{% embed url="https://www.blender.org/download/" %}

Then, extract the .tar.xz file to `~/Documents/Blender`



## Register Blender as Application



Edit the following entries of blender.desktop

{% code title="blender.desktop" %}
```bash
Exec=/home/tk/Documents/Blender/blender %f
Icon=/home/tk/Documents/Blender/blender.svg
```
{% endcode %}



And then

```bash
cp ~/Documents/Blender/blender.desktop ~/.local/share/applications
# alternatively, cp ~/Documents/Blender/blender.desktop /usr/share/applications
```



To create a synlink, do

```bash
sudo ln -s ~/Documents/Blender/blender /usr/local/bin/blender
```



## Create File Association

Edit `/usr/share/applications/defaults.list`, add the following line

{% code title="defaults.list" %}
```
application/x-blender=blender.desktop
```
{% endcode %}





## Reference

{% embed url="https://askubuntu.com/questions/1331428/how-to-install-the-latest-version-of-blender-in-ubuntu-instead-of-the-old-versi" %}





