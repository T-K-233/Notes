# Retrieve Github user avatar with Github API

```python
import time

import requests

headers = {
    "Authorization": "Bearer GITHUB_OAUTH_KEY"
    }


fetched_names = []

for i in range(100):
    print("="*16)
    print(i)
    print("="*16)
    resp = requests.get("https://api.github.com/repos/ucb-bar/chipyard/commits?per_page=100&page={page}".format(page=i), headers=headers)
    commits = resp.json()

    time.sleep(0.1)
    if not commits:
        break

    for commit in commits:
        author = commit.get("author")
        if not author:
            continue
        
        username = author.get("login")
        name = commit.get("commit").get("author").get("name")

        print(username, "-", name)
        if name in fetched_names:
            continue
        print("  downloading...")

        r = requests.get("https://github.com/{username}.png?size=90".format(username=username), headers=headers, allow_redirects=True)
        open(".git\\avatar\\{name}.png".format(name=name), "wb").write(r.content)
        fetched_names.append(name)

        time.sleep(0.1)

print("finished")

```





```python
#!/usr/bin/env python3
# fetch Gravatars

import hashlib
import os
import requests
from subprocess import Popen, PIPE
import time

size = 90
output_dir = ".git/avatar"

if not os.path.isdir(".git"):
    raise FileNotFoundError("no .git/ directory found in current path")

if not os.path.isdir(output_dir):
    os.makedirs(output_dir)

git_log_process = Popen(["git", "log", "--pretty=format:%ae|%an"], stdout=PIPE, text=True)
git_log_output = git_log_process.communicate()[0]

processed_authors = set()

for line in git_log_output.splitlines():
    email, author = line.strip().split("|")

    if author in processed_authors:
        continue

    processed_authors.add(author)

    author_image_file = os.path.join(output_dir, f"{author}.png")

    # skip images we have
    if os.path.exists(author_image_file):
        continue

    # try and fetch image
    md5_hash = hashlib.md5(email.lower().encode()).hexdigest()
    grav_url = f"http://www.gravatar.com/avatar/{md5_hash}?d=404&size={size}"

    print(f"fetching image for '{author}' {email} ({grav_url})...")

    response = requests.get(grav_url, stream=True)

    if response.status_code == 200:
        with open(author_image_file, "wb") as f:
            for chunk in response.iter_content(1024):
                f.write(chunk)
        response.close()
    else:
        print("error:", response.status_code)
        response.close()
        continue

    # sleep to avoid hitting Gravatar API too hard
    time.sleep(1)

```
