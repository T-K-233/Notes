# Setting up SteamVR without HMD



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







