# Cannot Start Chrome in Ubuntu 22.04 After Changing Network Settings

When double click the Chrome icon, it shows the chrome is loading on the top task bar, but then nothing happens. No window is poped.



When running from terminal, it shows the following error

```bash
$ google-chrome
[6302:6302:1204/161040.571963:ERROR:process_singleton_posix.cc(353)] The profile appears to be in use by another Google Chrome process (4557) on another computer (tk-MS-7924).  Chrome has locked the profile so that it doesn't get corrupted.  If you are sure no other processes are using this profile, you can unlock the profile and relaunch Chrome.
[6302:6302:1204/161040.572079:ERROR:message_box_dialog.cc(146)] Unable to show a dialog outside the UI thread message loop: Google Chrome - The profile appears to be in use by another Google Chrome process (4557) on another computer (tk-MS-7924).  Chrome has locked the profile so that it doesn't get corrupted.  If you are sure no other processes are using this profile, you can unlock the profile and relaunch Chrome.
```



Solution:

```bash
rm -rf ~/.config/google-chrome/Singleton*
```



## Reference

{% embed url="https://askubuntu.com/questions/476918/google-chrome-wont-start-after-changing-hostname" %}
