# Windows Ubuntu Dual Boot Issues

{% embed url="https://askubuntu.com/questions/52963/how-do-i-set-windows-to-boot-as-the-default-in-the-boot-loader" %}

### Windows Linux timestamp difference issue

### Let Linux to use local time

```
timedatectl set-local-rtc 1 --adjust-system-clock
```



now, to check if the change is effective, run

```
timedatectl
```

<figure><img src="../../.gitbook/assets/image (1) (2) (2) (1).png" alt=""><figcaption></figcaption></figure>



To undo this change:

```
timedatectl set-local-rtc 0 --adjust-system-clock
```
