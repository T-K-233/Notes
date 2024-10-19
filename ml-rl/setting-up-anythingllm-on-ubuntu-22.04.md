# Setting up AnythingLLM on Ubuntu 22.04



Follow this document

{% embed url="https://docs.anythingllm.com/installation-docker/local-docker" %}

```bash
docker pull mintplexlabs/anythingllm
```



```bash
export STORAGE_LOCATION=$HOME/Documents/anythingllm
mkdir -p $STORAGE_LOCATION
touch "$STORAGE_LOCATION/.env"
docker run -d -p 3001:3001 \
  --cap-add SYS_ADMIN \
  -v ${STORAGE_LOCATION}:/app/server/storage \
  -v ${STORAGE_LOCATION}/.env:/app/server/.env \
  -e STORAGE_DIR="/app/server/storage" \
  mintplexlabs/anythingllm
```



The service should be available on localhost:3001



Run Ollama

```bash
ollama run llama3

ollama serve
```









