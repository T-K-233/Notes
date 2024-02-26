# nsswitch.conf



So when I tried to install some packages on the gym server, I got this error:

```bash
(base) tk@gym:/scratch/Downloads$ sudo apt-get -y install cuda
Reading package lists... Done
Building dependency tree       
Reading state information... Done
cuda is already the newest version (12.2.2-1).
0 upgraded, 0 newly installed, 0 to remove and 0 not upgraded.
8 not fully installed or removed.
After this operation, 0 B of additional disk space will be used.
Setting up nvidia-compute-utils-535 (535.104.05-0ubuntu1) ...
Warning: The home dir /nonexistent you specified can't be accessed: No such file or directory
Adding system user `nvidia-persistenced' (UID 136) ...
Adding new group `nvidia-persistenced' (GID -1) ...
groupadd: invalid group ID '-1'
adduser: `/sbin/groupadd -g -1 nvidia-persistenced' returned error code 3. Exiting.
dpkg: error processing package nvidia-compute-utils-535 (--configure):
 installed nvidia-compute-utils-535 package post-installation script subprocess returned error exit status 1
dpkg: dependency problems prevent configuration of cuda-drivers-535:
 cuda-drivers-535 depends on nvidia-compute-utils-535 (>= 535.104.05); however:
  Package nvidia-compute-utils-535 is not configured yet.
```



Thanks to Kosta for explaning, it seems that the main issue is that linux cannot get a free user group ID for the daemon process that is performing the installation. This only happens on EECS servers since the computers here connects to the LDAP server to manage network account stuff. Their server could not respond a new available group id, since there are a ton of group entires within EECS.



To solve this, we need to disconnect the network group service temporarily.

This is configured in the `/etc/nsswitch.conf` file:

```bash
GNU nano 4.8                                         /etc/nsswitch.conf
# /etc/nsswitch.conf
#
# Example configuration of GNU Name Service Switch functionality.
# If you have the `glibc-doc-reference' and `info' packages installed, try:
# `info libc "Name Service Switch"' for information about this file.
passwd:         files sss systemd
group:          files  systemd
shadow:         files sss
gshadow:        files
hosts:          files mdns4_minimal [NOTFOUND=return] dns
networks:       files
protocols:      db files
services:       db files sss
ethers:         db files
rpc:            db files
netgroup:       files sss
sudoers:        files
automount:      files sss
```



`sss` is the EECS network account service. We need to remove it from the `group` entry.



Now it fixes the issue, and apt can install packages correctly.

