# Eclipse Muto - User Guide

This guide covers how to deploy and operate Eclipse Muto as a **ROS2 workspace deployment orchestrator**.

## Getting Started

- **[Project Overview](../project_overview.md)** - Architecture and core concepts
- **[Quick Start](./quick_start.md)** - Run the device agent and deploy your first release
- **[Running Examples](./running_examples.md)** - End-to-end example flows
- **[Troubleshooting](./troubleshooting.md)** - Common issues and fixes

## Key Concepts

### Release Artifacts
Muto treats your ROS2 workspace as an opaque archive. You provide:
- `name`, `version`
- `artifact_uri` (HTTP/S)
- `checksum` (sha256)
- `start_command` / `stop_command` (optional)

### Control Plane
Muto uses **Eclipse Symphony** for desired-state orchestration. You define a stack in Symphony, and the device agent reconciles it locally.

### Device Agent
The agent installs artifacts atomically, runs your `start_command`, and rolls back if startup fails. State is persisted locally and reported through Symphony.

## Who Should Use This Guide

- Robotics engineers deploying ROS2 workspaces
- DevOps teams managing edge fleets
- Systems engineers integrating Symphony-based orchestration
