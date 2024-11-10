# Disabling the "\<Application> is not responding." System Message on Ubuntu

When running computationally heavy applications like Isaac Gym or Blender, it's common that the computation loop takes longer such that the application cannot meet Ubuntu's refresh time limit.&#x20;

If this response deadline is missed, Ubuntu will show a pop-up window to notify the user, which is annoying.

<figure><img src="../../.gitbook/assets/image (3) (1) (1) (1) (1) (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>

## Solution

To address this issue, use the following command to either set the timeout to be longer, or disable the aliveness detection by setting the value to 0. The timeout value is in milliseconds.

```bash
gsettings set org.gnome.mutter check-alive-timeout 0
```



## Errata

In VSCode, it's possible to encounter the following error:

```bash
/snap/core20/current/lib/x86_64-linux-gnu/libstdc++.so.6: version `GLIBCXX_3.4.29' not found (required by /lib64/libproxy.so.1)
Failed to load module: /home/untitled1/snap/code/common/.cache/gio-modules/libgiolibproxy.so
```



This is due to an artifact of VSCode from snap install, metioned in [this thread](https://github.com/microsoft/vscode/issues/180629). The solution is to uninstall vscode from snap store and install it from the Microsoft website .deb release package.



```
1. Removed the snap: `sudo snap remove code`

2. Downloaded the .deb package from the VSCode website.

3. Installed the .deb package: `sudo apt install ./<downloaded-file.deb>`
```

