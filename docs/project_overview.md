# Eclipse Muto - Project Overview

## What is Eclipse Muto?

**Eclipse Muto** is an open-source deployment orchestrator for **ROS2 workspace artifacts**. It enables bulk deployment, version switching, and rollback across edge devices **without SSH**. Muto treats user software as an **opaque archive**: you version it, compress it, and provide start/stop commands. Muto handles transport, atomic install, activation, and rollback.

## The Problem Muto Solves

Traditional robotics deployments rely on:

1. SSH access to each device
2. Manual artifact transfer
3. Manual restarts and rollbacks
4. Limited visibility into what version is running where

Muto replaces these with **versioned artifacts**, **bulk deployment**, **atomic activation**, and **automatic rollback**.

## Architecture Overview

Muto uses **Eclipse Symphony** as its control plane and a lightweight device agent on each target.

```
┌─────────────────────────────────────────────────────────────┐
│                   Eclipse Symphony                          │
│  - Stores desired state (Stack definitions)                  │
│  - Issues deployment commands via MQTT                       │
└─────────────────────────────────────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────────────────────┐
│                   Muto Device Agent                         │
│  - Downloads artifacts (HTTP/S)                              │
│  - Installs atomically (current/previous symlinks)           │
│  - Starts/stops processes                                    │
│  - Rolls back on failure                                     │
│  - Reports state                                             │
└─────────────────────────────────────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────────────────────┐
│                     Artifact Store                           │
│  - Any user-provided HTTP server                             │
└─────────────────────────────────────────────────────────────┘
```

### Components

1. **Symphony Control Plane** (external)
2. **Artifact Store** (user-provided HTTP/S)
3. **Muto Device Agent** (`src/agent`)
4. **Muto Composer** (`src/composer`) for JSON model validation/composition

## Release Model

Muto uses a minimal, ROS-agnostic release schema:

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

## Atomic Installation

The device agent uses a crash-safe install layout:

```
<ROOT>/
  stacks/
    <stack_name>/
      releases/
        <version>/
        <version>.tmp/
      current -> releases/<version>
      previous -> releases/<version>
  incoming/
```

Installation steps:
1. Download artifact to `<ROOT>/incoming/`
2. Extract to `<version>.tmp/`
3. Verify checksum and metadata
4. Atomically rename to `<version>`
5. Atomically switch `current` and `previous` symlinks

## Non-Goals

- No container runtime support
- No ROS graph awareness or enforcement
- No hardware compatibility or dependency management
- No custom UI in v1 (Symphony API/CLI only)
