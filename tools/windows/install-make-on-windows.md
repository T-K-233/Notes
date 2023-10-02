# Install Make on Windows

## Method 1: Install with MSYS2

Download the installer from [https://www.msys2.org/](https://www.msys2.org/).

<figure><img src="../../.gitbook/assets/image (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>

Install make with the command:

```bash
pacman -S make
```

![](<../../.gitbook/assets/image (1) (1) (1) (1) (1).png>)

Add the following path to PATH:

```bash
D:\Documents\msys64\usr\bin
```

## Method 2: Install with GNUWin (Not Recommended)

Download GunWin make from [here](https://gnuwin32.sourceforge.net/packages/make.htm).

Download both the "Binaries" and "Dependencies" Zip files.

<figure><img src="../../.gitbook/assets/image (2) (1).png" alt=""><figcaption></figcaption></figure>

Create a folder for Make in a known location, for example, `D:\Documents\Make`.

Unzip the "make-3.81-bin.zip" first to the newly created folder, then merge the content in "make-3.81-dep.zip" to the newly created folder.

<figure><img src="../../.gitbook/assets/image (1) (1) (1) (1) (1) (1).png" alt=""><figcaption><p>Content inside "bin" after the merge</p></figcaption></figure>

Add `D:\Documents\Make\bin` to PATH.

Now make is installed on the system. Run `make` to invoke.

<figure><img src="../../.gitbook/assets/image (2) (1) (1).png" alt=""><figcaption></figcaption></figure>
