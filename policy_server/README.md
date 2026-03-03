# Policy Server

GPU-side inference server for SO-101. Loads [LeRobot](https://github.com/huggingface/lerobot) policies and serves action predictions over ZMQ or gRPC.

## Prerequisites

Install LeRobot **before** installing the policy server (it's not on PyPI):

```bash
pip install lerobot "lerobot[async]" "lerobot[smolvla]"
```

## Install

### Option 1 — Editable (for development)

Clone the repo and install in editable mode. Edit code directly, changes take effect immediately.

```bash
REPO_DIR="/workspace/repo"

# Clone or update
if [ -d "${REPO_DIR}/.git" ]; then
  git -C "${REPO_DIR}" pull
else
  git clone --depth 1 \
    https://github.com/legalaspro/so101-ros-physical-ai.git "${REPO_DIR}"
fi

uv pip install -e "${REPO_DIR}/policy_server"
```

### Option 2 — Direct from GitHub

One-liner, no local clone. Re-run to update.

```bash
uv pip install \
  "policy_server @ git+https://github.com/legalaspro/so101-ros-physical-ai.git#subdirectory=policy_server"
```

## Deploy on vast.ai

1. Create an instance using the **PyTorch** template
2. Set `PROVISIONING_SCRIPT` to the [provisioning gist](https://gist.github.com/legalaspro/d81fabb628f600cc27bc33ce5f5c130d)
3. Open TCP port **8090** in the instance config

The script installs LeRobot + policy_server automatically on first boot.

## Usage

```bash
# Console script
policy-server --transport=zmq --host=0.0.0.0 --port=8090 --fps=50

# Or as a module
python -m policy_server --transport=zmq --host=0.0.0.0 --port=8090 --fps=50
```

