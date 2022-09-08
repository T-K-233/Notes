# Github-Related Info

## SSH Keys

```
ssh-keygen -t ed25519 -C "tk??@outlook.com"
```

```
git config --global user.name "-T.K.-"
git config --global user.email "tk??@outlook.com"
```



When adding to GitHub / other platforms, use the form "TK Device Name" for personal devices, and use the form "OrgName Device Name" for organizational devices.

#### Current entries are:

TK Shizun Desktop

TK Hillside Desktop

TK HXG DELL Workstation

TK SurfaceBook

TK Framework Laptop

TK Hillside Ubuntu

TK Framework Ubuntu

Ma Lab NUC

EECS 151 Inst Machine



## File Structure

`scripts` folder

{% embed url="https://github.blog/2015-06-30-scripts-to-rule-them-all" %}



## Git Commit Message Format

![](<../.gitbook/assets/image (105).png>)



## Filenames

Use this format when naming files:

```
└─ year-month-day_filename_version_other_info.extension. 
```

e.g. `vision_processing_v1.0.2.py`, `comm_test_2019-12-04.cpp`

Note: on systems where dot and dash are not permitted, use underline instead.

e.g. `vision_processing_v1_0_2.py`, `comm_test_2019_12_04.cpp`

``

The version number in the filename should be three Arabic numbers with dots in between.

```
    ├─ vision_processing_2019-12-04_v1.0.0.py
    ├─ vision_processing_2019-12-05_v1.0.5.py
    └─ vision_processing_2019-12-05_v1.0.12.py
```

Reference [semver.org](https://semver.org) for more information on versioning.

