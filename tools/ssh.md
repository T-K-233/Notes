# SSH

```bash
ssh chiyufeng@ssh.ocf.berkeley.edu "mkdir -p ~/.ssh && chmod 700 ~/.ssh && echo '$(cat ~/.ssh/id_ed25519.pub)' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
```
