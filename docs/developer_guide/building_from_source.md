# Eclipse Muto - Building from Source

Muto v1 is a Python-based device agent and does not require a ROS build toolchain.

## Prerequisites

- Python 3.10+
- Eclipse Symphony API + MQTT broker (for integration testing)

## Install (editable)

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .
```

## Run the Agent

```bash
export MUTO_CONFIG=$(pwd)/config/device_agent.json
muto_device_agent
```
