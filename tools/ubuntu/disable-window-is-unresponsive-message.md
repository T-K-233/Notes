# Disable "window is unresponsive" Message







```bash
gsettings set org.gnome.mutter check-alive-timeout 0
```

timeout is in milliseconds



if encountered error below, this is due to error metioned in [this thread](https://github.com/microsoft/vscode/issues/180629). The solution is to uninstall vscode from snap store, and install it from Microsoft website .deb release package.

```bash
/snap/core20/current/lib/x86_64-linux-gnu/libstdc++.so.6: version `GLIBCXX_3.4.29' not found (required by /lib64/libproxy.so.1)
Failed to load module: /home/untitled1/snap/code/common/.cache/gio-modules/libgiolibproxy.so
```

