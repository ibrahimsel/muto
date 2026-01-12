# Eclipse Muto - Quick Start

This guide shows how to run the Muto device agent and deploy a versioned ROS2 workspace via Eclipse Symphony.

## Prerequisites

- Eclipse Symphony control plane (API + MQTT broker)
- HTTP(S) server for release artifacts
- Python 3.10+

## 1) Prepare a Release Artifact

Package your ROS2 workspace into an archive (example):

```bash
tar -czf towtruck-autonomy-1.0.1.tar.gz ./ros2_ws
sha256sum towtruck-autonomy-1.0.1.tar.gz
```

Upload the archive to your HTTP(S) server.

## 2) Configure the Device Agent

Use `config/device_agent.json` as a starting point:

```json
{
  "device_id": "muto-device-001",
  "storage": { "root_dir": "/var/lib/muto" },
  "symphony": {
    "enabled": true,
    "topic_prefix": "symphony",
    "request_topic": "coa-request",
    "response_topic": "coa-response",
    "target_name": "muto-device-001",
    "mqtt": { "host": "localhost", "port": 1883, "name": "muto-device-001" }
  }
}
```

## 3) Run the Device Agent

```bash
export MUTO_CONFIG=$(pwd)/config/device_agent.json
muto_device_agent
```

The agent registers with Symphony (if enabled) and begins reconciling desired state.

## 4) Define a Stack in Symphony

Use Symphony to define a stack model. The payload below is stored in the component `properties.data`.

```json
{
  "name": "towtruck-autonomy",
  "version": "1.0.1",
  "artifact_uri": "https://artifacts.example.com/towtruck-autonomy-1.0.1.tar.gz",
  "checksum": "sha256:8c4f0d3e1e9b1a8f2c2c9c7a3f4d5b6a9e8f7c6d5b4a3a2f1e0d9c8b7a6",
  "start_command": "./run.sh",
  "stop_command": "pkill -f run.sh",
  "working_directory": ".",
  "environment": {
    "ROS_DOMAIN_ID": "42",
    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"
  }
}
```

You can use the helper scripts in `docs/samples/symphony/` or call the Symphony API directly.

## 5) Deploy and Verify

When the desired solution targets the device, the agent will:

- download the artifact
- install atomically
- start the release
- rollback on failure

Check status via Symphony `get` calls or inspect the device log files in:

```
/var/lib/muto/stacks/<stack_name>/logs/
```
