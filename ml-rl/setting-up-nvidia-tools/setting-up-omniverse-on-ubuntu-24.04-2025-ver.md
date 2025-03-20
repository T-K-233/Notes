# Setting up Omniverse on Ubuntu 24.04 (2025 Ver)



NVIDIA is [deprecating support for Omniverse Launcher](https://developer.nvidia.com/omniverse/legacy-tools).

Instead, each application and extension should be installed separately.



Omniverse now comes as [Omniverse Kit SDK](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/omniverse/collections/kit).



## Download Kit

Go to the [Omniverse website](https://developer.nvidia.com/omniverse/) and scroll to the Get Started section.

<figure><img src="../../.gitbook/assets/image (254).png" alt=""><figcaption></figcaption></figure>

Click "Linux" to download a zip folder.

<figure><img src="../../.gitbook/assets/image (255).png" alt=""><figcaption></figcaption></figure>

Extract the zip folder at a known location. Here, we put it under `/home/tk/Documents/Omniverse/kit-sdk-public@106.5.0+1102.eecf68f2/`.



## Running Omniverse

To launch Omniverse, go to the kit install directory and run

```bash
./omni.app.full.sh
```



The Omniverse should now be running.





## Setting up Extensions

By default, this version of Omniverse do not have most of the Extensions enabled. We need to manually configure them in the UI interface.



### Setting up Onshape exporter

In the Omniverse application window, go to Window -> Extensions.

<figure><img src="../../.gitbook/assets/image (256).png" alt=""><figcaption></figcaption></figure>



Search for Physx, install, enable, and set to be autoload

<figure><img src="../../.gitbook/assets/image (257).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (258).png" alt=""><figcaption></figcaption></figure>



Similarly, search for Physx bundle and do the same

<figure><img src="../../.gitbook/assets/image (259).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (260).png" alt=""><figcaption></figcaption></figure>



Then, search for onshape

<figure><img src="../../.gitbook/assets/image (261).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (262).png" alt=""><figcaption></figcaption></figure>



Now the Import from Onshape should be available under File

<figure><img src="../../.gitbook/assets/image (263).png" alt=""><figcaption></figcaption></figure>





#### Configuring default scene frame

Go to Edit -> Preferences

<figure><img src="../../.gitbook/assets/image (264).png" alt=""><figcaption></figcaption></figure>



Under the Stage tab, set Default Up Axis to be Z.





<figure><img src="../../.gitbook/assets/image (265).png" alt=""><figcaption></figcaption></figure>



Then, create a new stage with File -> New, or Ctrl + N

The frame should be updated.











