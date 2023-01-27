# Build EagleX Linux Image

Environment:

Windows 10 WSL Ubuntu 22.02

OR

bwrcrdsl-1.eecs.berkeley.edu

OR

Ubuntu 22.02



## Clone Repo

```bash
git clone git@bwrcrepo.eecs.berkeley.edu:chiyufeng/eagle-sdk.git
cd eagle-sdk
git submodule init
git submodule update
```



## Set up the cross-compiler toolchain

```bash
make toolchain
```

if encounter this error:

<figure><img src="https://lh5.googleusercontent.com/EWFO2yK2MAFsLrGdbA_3Yv47RYMHUSRFAZHf6057rwyaQPcGpkW3QM5yy7cOjxyNRXjicqYknjLbvtq05zllLIbjZkXDPKi6bWfvc7qMzqQhBYzKM56-ICA42OpVZlhOLtztrOHSMHciFjbSbZ0YGOO8HKwXMyVgYT0rPJcOY6ng6qipvfo9CFtUo26Ee-qq=s2048" alt=""><figcaption></figcaption></figure>

```bash
cd buildroot
git fetch
git cherry-pick 9e2128bf5072e5f2fd69e2fc0239558782dfc677

```

We need to&#x20;

updating the m4 package to version 1.4.19 by updating to latest version: [https://git.buildroot.net/buildroot/tree/package/m4](https://git.buildroot.net/buildroot/tree/package/m4)



Navigate to `buildroot/package/m4` and make the following change:

* m4.mk:

```makefile
################################################################################
#
# m4
#
################################################################################

M4_VERSION = 1.4.19
M4_SOURCE = m4-$(M4_VERSION).tar.xz
M4_SITE = $(BR2_GNU_MIRROR)/m4
M4_LICENSE = GPL-3.0+
M4_LICENSE_FILES = COPYING

$(eval $(host-autotools-package))
```

* m4.hash

```
# Locally calculated after checking pgp signature
sha256  63aede5c6d33b6d9b13511cd0be2cac046f2e70fd0a07aa9573a04a82783af96  m4-1.4.19.tar.xz
# License files, locally calculated
sha256  3972dc9744f6499f0f9b2dbf76696f2ae7ad8af9b23dde66d6af86c9dfb36986  COPYING
```

* delete all .patch files, if any





\
















