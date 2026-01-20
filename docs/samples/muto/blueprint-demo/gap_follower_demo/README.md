# Eclipse Muto Gap Follower OTA Demo

This demo showcases Eclipse Muto's Over-The-Air (OTA) update capabilities with automatic rollback on failure. It deploys three variants of a gap follower ROS node to demonstrate version management and fault recovery.

## Overview

The demo includes three gap follower variants with different speed configurations:

| Variant | Version | MAX_SPEED | Behavior |
|---------|---------|-----------|----------|
| **slow** | 0.0.1 | 0.5 m/s | Works normally |
| **medium** | 0.0.2 | 1.0 m/s | Works normally |
| **fast** | 0.0.3 | 2.0 m/s | **Fails intentionally** to trigger rollback |

The fast variant is designed to fail after startup, demonstrating Muto's automatic rollback capability.

## Prerequisites

- Docker and Docker Compose
- ROS 2 Humble (for local testing)
- Python 3.10+
- `colcon` build tools

## Directory Structure

```
gap_follower_demo/
├── README.md                    # This file
├── create-packages.sh           # Creates artifact packages
├── check-status.sh              # Checks deployment status
├── slow/
│   └── run.sh                   # Slow variant startup script
├── medium/
│   └── run.sh                   # Medium variant startup script
├── fast/
│   └── run.sh                   # Fast variant (fails for demo)
├── stacks/
│   ├── gap_follower_slow.json   # Stack definition v0.0.1
│   ├── gap_follower_medium.json # Stack definition v0.0.2
│   └── gap_follower_fast.json   # Stack definition v0.0.3
└── artifacts/                   # Generated tar.gz packages
```

## Quick Start

### 1. Create Artifact Packages

First, generate the artifact packages for each variant:

```bash
cd docs/samples/gap_follower_demo
./create-packages.sh
```

This creates:
- `artifacts/gap_follower_slow.tar.gz`
- `artifacts/gap_follower_medium.tar.gz`
- `artifacts/gap_follower_fast.tar.gz`

And updates the checksums in the stack JSON files.

### 2. Start the Artifact Server

Serve the artifacts via HTTP:

```bash
cd artifacts
python3 -m http.server 8080
```

The artifacts will be available at `http://localhost:8080/`.

### 3. Start Muto

In a separate terminal, start the Muto stack:

```bash
cd /path/to/muto
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch launch/muto.launch.py
```

### 4. Deploy Stacks

Deploy the slow variant first:

```bash
# Using the Muto agent or Symphony API, send the stack definition
cat stacks/gap_follower_slow.json
```

## Demo Flow: Rollback on Failure

This demonstrates automatic rollback when a deployment fails.

### Step 1: Deploy Slow Variant (v0.0.1)

Deploy the initial working version:

```bash
# Send gap_follower_slow.json to Muto
# The stack will be downloaded, built, and launched
```

Check status:
```bash
./check-status.sh
```

Expected output:
```
Stack: gap_follower_slow
  Status:           running
  Current Version:  0.0.1
  Previous Version: N/A
```

### Step 2: Upgrade to Medium Variant (v0.0.2)

Deploy the medium speed version:

```bash
# Send gap_follower_medium.json to Muto
```

Check status:
```bash
./check-status.sh
```

Expected output:
```
Stack: gap_follower_medium
  Status:           running
  Current Version:  0.0.2
  Previous Version: 0.0.1
```

### Step 3: Upgrade to Fast Variant (v0.0.3) - TRIGGERS ROLLBACK

Deploy the fast variant, which is designed to fail:

```bash
# Send gap_follower_fast.json to Muto
```

**What happens:**
1. Muto downloads and extracts the fast variant
2. The `run.sh` script starts but exits with error after 3 seconds
3. Muto detects the failure
4. Automatic rollback to v0.0.2 (medium) is triggered
5. The medium variant is redeployed

Check status:
```bash
./check-status.sh
```

Expected output:
```
Stack: gap_follower_medium
  Status:           rolled_back
  Current Version:  0.0.2
  Previous Version: 0.0.3
  Rollback Count:   1
```

## State Persistence

Muto persists deployment state to `~/.muto/state/<stack_name>/state.json`.

### State File Structure

```json
{
  "stack_id": "gap_follower_medium",
  "stack_name": "gap_follower_medium",
  "current_version": "0.0.2",
  "previous_version": "0.0.3",
  "current_stack": { /* full stack definition */ },
  "previous_stack": { /* previous stack for rollback */ },
  "status": "rolled_back",
  "deployed_at": "2025-01-19T12:00:00Z",
  "last_updated": "2025-01-19T12:05:00Z",
  "error_message": "",
  "rollback_count": 1
}
```

### Status Values

| Status | Description |
|--------|-------------|
| `pending` | Deployment queued |
| `deploying` | Deployment in progress |
| `running` | Successfully deployed and running |
| `failed` | Deployment failed |
| `rolled_back` | Rolled back to previous version |
| `stopped` | Stack stopped |

## Configuration

### Workspace Path

By default, Muto stores workspaces in `~/.muto/workspaces`. Override with:

```bash
export MUTO_ROOT=/custom/path
```

This sets:
- Workspaces: `$MUTO_ROOT/workspaces`
- State: `$MUTO_ROOT/state`

### Stack JSON Format

Stack definitions use the `stack/archive` content type:

```json
{
  "metadata": {
    "name": "gap_follower_medium",
    "description": "Gap Follower - Medium variant",
    "content_type": "stack/archive",
    "version": "0.0.2"
  },
  "launch": {
    "url": "http://artifact-server:8080/gap_follower_medium.tar.gz",
    "properties": {
      "algorithm": "sha256",
      "checksum": "<sha256-hash>",
      "launch_file": "run.sh",
      "flatten": true
    }
  },
  "runtime": {
    "start_command": "./run.sh",
    "stop_command": "pkill -f gap_follower",
    "working_directory": "."
  }
}
```

## Troubleshooting

### Artifacts not downloading

1. Verify the artifact server is running:
   ```bash
   curl http://localhost:8080/gap_follower_slow.tar.gz -I
   ```

2. Check the URL in the stack JSON matches your server address

### Rollback not triggering

1. Ensure a previous version was deployed first
2. Check the state file exists:
   ```bash
   ls ~/.muto/state/
   ```

3. Verify `previous_stack` is populated in state.json

### Build failures

1. Ensure ROS 2 Humble is sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Check rosdep is initialized:
   ```bash
   rosdep update
   ```

### Checking logs

View Muto composer logs for detailed information:
```bash
ros2 topic echo /muto/composer/events
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Symphony/Cloud                          │
│                    (External - not in demo)                  │
└─────────────────────┬───────────────────────────────────────┘
                      │ Stack Definition
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                       Muto Agent                             │
│                   (MQTT/Ditto Bridge)                        │
└─────────────────────┬───────────────────────────────────────┘
                      │ StackRequestEvent
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                     Muto Composer                            │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────────┐   │
│  │   Stack     │  │ Orchestration│  │   Pipeline       │   │
│  │  Manager    │──│   Manager    │──│   Engine         │   │
│  └─────────────┘  └──────────────┘  └──────────────────┘   │
│         │                │                    │             │
│         │         ┌──────┴──────┐             │             │
│         │         │  Rollback   │             │             │
│         │         │   Logic     │             │             │
│         │         └─────────────┘             │             │
│         ▼                                     ▼             │
│  ┌─────────────┐                    ┌──────────────────┐   │
│  │   State     │                    │    Plugins       │   │
│  │ Persistence │                    │ (Provision/Launch│   │
│  └─────────────┘                    └──────────────────┘   │
│         │                                     │             │
└─────────┼─────────────────────────────────────┼─────────────┘
          │                                     │
          ▼                                     ▼
   ~/.muto/state/                      ~/.muto/workspaces/
   └── gap_follower/                   └── gap_follower/
       └── state.json                      ├── src/
                                           ├── build/
                                           └── install/
```

## Next Steps

1. **Integrate with Symphony**: Connect the demo to Eclipse Symphony for cloud-based orchestration
2. **Add more variants**: Create additional speed profiles or different algorithms
3. **Monitor with metrics**: Add Prometheus/Grafana for deployment metrics
4. **Multi-device demo**: Deploy to multiple edge devices simultaneously
