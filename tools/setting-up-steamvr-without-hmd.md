# Setting up SteamVR

## Install on Ubuntu 24.04

First, download and install Steam

```bash
sudo apt install ./steam_latest.deb
```

When running for the first time, Steam will detect the dependencies and automatically install the missing packages.

<figure><img src="../.gitbook/assets/image (276).png" alt=""><figcaption></figcaption></figure>

Then, download and install SteamVR

<figure><img src="../.gitbook/assets/image (277).png" alt=""><figcaption></figcaption></figure>

## Running without HMD

reference [https://vvvv.org/blog/using-htc-vive-trackers-without-headset](https://vvvv.org/blog/using-htc-vive-trackers-without-headset)

Edit `D:/Documents/Steam/steamapps/common/SteamVR/resources/settings/default.vrsettings`

Edit the following fields

```json
    ...
    "steamvr": {
        "requireHmd": false,
        ...
    },
    ...  
    "driver_imu": {
        "enable":  true
    },
    ...

```

Next, edit file `D:/Documents/Steam/config/steamvr.vrsettings`

Add the following lines in `steamvr` field

```json
      "activateMultipleDrivers" : false,
      "forcedDriver" : "null",
```

After change:

```json
   "steamvr" : {
      "activateMultipleDrivers" : false,
      "forcedDriver" : "null",
      "installID" : "18301835954502125344",
      "lastVersionNotice" : "2.2.3",
      "lastVersionNoticeDate" : "1703041472",
      "showAdvancedSettings" : true
   },
```

Restart SteamVR. Might need to add controller again

Then it will look like this

<figure><img src="../.gitbook/assets/image (200).png" alt=""><figcaption></figcaption></figure>
