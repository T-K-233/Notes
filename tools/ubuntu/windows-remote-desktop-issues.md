# Windows Remote Desktop Issues



### Windows Remote Desktop cannot connect to XFCE session



solution: ssh into the remote machine and run

```bash
xfce4-session-logout --logout
```

might need to run multiple times until it shows error that cannot logout anymore. then re-login.



### Screen not properly updated

<figure><img src="../../.gitbook/assets/image (251).png" alt=""><figcaption></figcaption></figure>

[https://askubuntu.com/questions/1523503/x2go-remote-desktop-with-xfce-doesnt-refresh-the-screen](https://askubuntu.com/questions/1523503/x2go-remote-desktop-with-xfce-doesnt-refresh-the-screen)



<figure><img src="../../.gitbook/assets/image (252).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (253).png" alt=""><figcaption></figcaption></figure>







