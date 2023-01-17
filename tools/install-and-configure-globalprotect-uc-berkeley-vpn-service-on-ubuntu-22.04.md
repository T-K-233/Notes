# Install and Configure GlobalProtect UC Berkeley VPN Service on Ubuntu 22.04

{% embed url="https://bugs.launchpad.net/ubuntu/+source/openssl/+bug/1960268/comments/26" %}

## Dependencies

```bash
sudo apt install gp-saml-gui
```





Create `~/ssl.conf` with following content

```
openssl_conf = openssl_init
[openssl_init]
ssl_conf = ssl_sect
[ssl_sect]
system_default = system_default_sect
[system_default_sect]
Options = UnsafeLegacyRenegotiation

```



Run the following command

```bash
eval $(OPENSSL_CONF=~/ssl.conf gp-saml-gui --portal --clientos=Windows vpn.berkeley.edu)
```

```
Looking for SAML auth tags in response to https://vpn.berkeley.edu/global-protect/prelogin.esp...
Got SAML REDIRECT, opening browser...
[PAGE   ] Finished loading page https://auth.berkeley.edu/cas/login?service=https%3A%2F%2Fshib.berkeley.edu%2Fidp%2FAuthn%2FExternal%3Fconversation%3De1s1%26entityId%3Dhttps%3A%2F%2Fvpn.berkeley.edu%3A443%2FSAML20%2FSP
[PAGE   ] Finished loading page https://api-6b447a0c.duosecurity.com/frame/frameless/v4/auth?sid=frameless-fc980f16-dda9-47a2-8808-cd6a6dde011a&tx=eyJhbGciOiJIUzUxMiIsInR5cCI6IkpXVCJ9.eyJkdW9fdW5hbWUiOiJjaGl5dWZlbmciLCJzY29wZSI6Im9wZW5pZCIsInJlc3BvbnNlX3R5cGUiOiJjb2RlIiwicmVkaXJlY3RfdXJpIjoiaHR0cHM6Ly9hdXRoLmJlcmtlbGV5LmVkdS9jYXMvbG9naW4iLCJzdGF0ZSI6IlRTVC04MTNiZGM2MGYwZDVhYTliNWZjNDlhMTViMzM4ZDFmMDgyMzYiLCJleHAiOjE2NzM5ODEzODgsInVzZV9kdW9fY29kZV9hdHRyaWJ1dGUiOnRydWUsImNsaWVudF9pZCI6IkRJU0oxVkJITVdNVU5RRFNSN0dLIn0.zySWWkJgZrIJ85urcbh9hDRmkJu8gRN_ZWiMwmfxGoHHXeUfke71bc2_doGnGWqNHyMGcWpNK0rlB3lvXdawZg
[PAGE   ] Finished loading page https://shib.berkeley.edu/idp/profile/SAML2/Redirect/SSO?execution=e1s1&_eventId_proceed=1
[PAGE   ] Finished loading page https://vpn.berkeley.edu/SAML20/SP/ACS
[SAML   ] Got SAML result headers: {'saml-username': 'chiyufeng', 'prelogin-cookie': 'QU345hrIt8olhp9k72f1oP592bOILG1JocWSUQexAN7/MPPxF9Fp8938i5pL85Ig', 'saml-slo': 'no', 'saml-auth-status': '1'}
[SAML   ] Got all required SAML headers, done.
IMPORTANT: We started with SAML auth to the portal interface, but received a cookie that's often associated with the gateway interface. You should probably try both.


SAML response converted to OpenConnect command line invocation:

    echo QU345hrIt8olhp9k72f1oP592bOILG1JocWSUQexAN7/MPPxF9Fp8938i5pL85Ig |
        sudo openconnect --protocol=gp --user=chiyufeng --os=win --usergroup=portal:prelogin-cookie --passwd-on-stdin vpn.berkeley.edu

SAML response converted to test-globalprotect-login.py invocation:

    test-globalprotect-login.py --user=chiyufeng --clientos=Windows -p '' \
         https://vpn.berkeley.edu/global-protect/getconfig.esp prelogin-cookie=QU345hrIt8olhp9k72f1oP592bOILG1JocWSUQexAN7/MPPxF9Fp8938i5pL85Ig


```

Search the results of above command for "SAML response converted to OpenConnect command line invocation:", the echo content will be the key. For UC Berkeley vpn, we need to run the two commands separately.

```bash
QU345hrIt8olhp9k72f1oP592bOILG1JocWSUQexAN7/MPPxF9Fp8938i5pL85Ig
```



```bash
sudo openconnect --protocol=gp --user=chiyufeng --os=win --usergroup=portal:prelogin-cookie vpn.berkeley.edu
```

```
POST https://vpn.berkeley.edu/global-protect/prelogin.esp?tmp=tmp&clientVer=4100&clientos=Windows
Attempting to connect to server 136.152.12.0:443
Connected to 136.152.12.0:443
SSL negotiation with vpn.berkeley.edu
Connected to HTTPS on vpn.berkeley.edu with ciphersuite (TLS1.2)-(RSA)-(AES-256-GCM)
Got HTTP response: HTTP/1.1 200 OK
Date: Tue, 17 Jan 2023 17:37:50 GMT
Content-Type: application/xml; charset=UTF-8
Content-Length: 1389
Connection: keep-alive
ETag: "167861bd5382"
Pragma: no-cache
Cache-Control: no-store, no-cache, must-revalidate, post-check=0, pre-check=0
Expires: Thu, 19 Nov 1981 08:52:00 GMT
X-FRAME-OPTIONS: DENY
Set-Cookie: PHPSESSID=e8eae0613e06fb9bb6d97eb669672323; secure; HttpOnly
Set-Cookie: PHPSESSID=e8eae0613e06fb9bb6d97eb669672323; secure; HttpOnly
Set-Cookie: PHPSESSID=e8eae0613e06fb9bb6d97eb669672323; secure; HttpOnly
Set-Cookie: PHPSESSID=e8eae0613e06fb9bb6d97eb669672323; secure; HttpOnly
Set-Cookie: PHPSESSID=e8eae0613e06fb9bb6d97eb669672323; secure; HttpOnly
Set-Cookie: PHPSESSID=e8eae0613e06fb9bb6d97eb669672323; secure; HttpOnly
Set-Cookie: PHPSESSID=e8eae0613e06fb9bb6d97eb669672323; secure; HttpOnly
Set-Cookie: PHPSESSID=e8eae0613e06fb9bb6d97eb669672323; secure; HttpOnly
Set-Cookie: PHPSESSID=e8eae0613e06fb9bb6d97eb669672323; secure; HttpOnly
Set-Cookie: PHPSESSID=e8eae0613e06fb9bb6d97eb669672323; secure; HttpOnly
Set-Cookie: PHPSESSID=e8eae0613e06fb9bb6d97eb669672323; secure; HttpOnly
Set-Cookie: PHPSESSID=e8eae0613e06fb9bb6d97eb669672323; path=/; secure; httponly
Set-Cookie: PHPSESSID=e8eae0613e06fb9bb6d97eb669672323; secure; HttpOnly
Strict-Transport-Security: max-age=31536000;
X-XSS-Protection: 1; mode=block;
X-Content-Type-Options: nosniff
Content-Security-Policy: default-src 'self'; script-src 'self' 'unsafe-inline'; img-src * data:; style-src 'self' 'unsafe-inline';
HTTP body length:  (1389)
Destination form field prelogin-cookie was specified; assuming SAML REDIRECT authentication is complete.
Prelogin form _login: "Username: " user(TEXT)=(null), "prelogin-cookie: " prelogin-cookie(PASSWORD)
Enter login credentials
POST https://vpn.berkeley.edu/global-protect/getconfig.esp
Got HTTP response: HTTP/1.1 200 OK
Date: Tue, 17 Jan 2023 17:37:50 GMT
Content-Type: application/xml; charset=UTF-8
Content-Length: 9221
Connection: keep-alive
ETag: "ba361bd5382"
Pragma: no-cache
Cache-Control: no-store, no-cache, must-revalidate, post-check=0, pre-check=0
Expires: Thu, 19 Nov 1981 08:52:00 GMT
X-FRAME-OPTIONS: DENY
Set-Cookie: PHPSESSID=b9b6239d8e2118a80efe5be3bee5cfd0; path=/; secure; HttpOnly
Set-Cookie: PHPSESSID=b9b6239d8e2118a80efe5be3bee5cfd0; path=/; secure; HttpOnly
Set-Cookie: PHPSESSID=b9b6239d8e2118a80efe5be3bee5cfd0; path=/; secure; HttpOnly
Set-Cookie: PHPSESSID=b9b6239d8e2118a80efe5be3bee5cfd0; path=/; secure; HttpOnly
Set-Cookie: PHPSESSID=b9b6239d8e2118a80efe5be3bee5cfd0; path=/; secure; HttpOnly
Set-Cookie: PHPSESSID=b9b6239d8e2118a80efe5be3bee5cfd0; path=/; secure; HttpOnly
Set-Cookie: PHPSESSID=b9b6239d8e2118a80efe5be3bee5cfd0; path=/; secure; HttpOnly
Set-Cookie: PHPSESSID=b9b6239d8e2118a80efe5be3bee5cfd0; path=/; secure; HttpOnly
Set-Cookie: PHPSESSID=b9b6239d8e2118a80efe5be3bee5cfd0; path=/; secure; HttpOnly
Set-Cookie: PHPSESSID=b9b6239d8e2118a80efe5be3bee5cfd0; path=/; secure; HttpOnly
Strict-Transport-Security: max-age=31536000;
X-XSS-Protection: 1; mode=block;
X-Content-Type-Options: nosniff
Content-Security-Policy: default-src 'self'; script-src 'self' 'unsafe-inline'; img-src * data:; style-src 'self' 'unsafe-inline';
HTTP body length:  (9221)
Portal set HIP report interval to 60 minutes).
3 gateway servers available:
  Library Access and Full Tunnel (campus.vpn.berkeley.edu)
  Restricted Tunnel (restricted.vpn.berkeley.edu)
  Split Tunnel (Default) (campus-split.vpn.berkeley.edu)
Please select GlobalProtect gateway.
GATEWAY: [Library Access and Full Tunnel|Restricted Tunnel|Split Tunnel (Default)]: rrLibrary Access and Full Tunnel

```

Search the results of the command for one gateway



```
eval $(OPENSSL_CONF=~/ssl.conf gp-saml-gui --gateway --clientos=Windows campus.vpn.berkeley.edu)
```



```bash
sudo openconnect --protocol=gp --user=chiyufeng --os=win --usergroup=gateway:prelogin-cookie campus.vpn.berkeley.edu
```

