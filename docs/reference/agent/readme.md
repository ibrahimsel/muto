# Eclipse Muto Agent - Reference

The **Muto Device Agent** runs on each edge device and reconciles desired stack versions from Eclipse Symphony. It downloads artifacts, installs them atomically, starts the runtime, and rolls back on failure.

**Source Code**: `src/agent/`

## Responsibilities

- Download artifacts over HTTP(S) and verify SHA256 checksums
- Perform atomic installs with `current` / `previous` symlinks
- Execute user-provided `start_command` / `stop_command`
- Detect start failures and auto-rollback
- Persist state (current, previous, timestamps, failure reason)
- Report status via Symphony provider `get` calls

## Core Modules

- `src/agent/agent/device_agent.py`  
  Entry point for the Symphony-driven agent.
- `src/agent/agent/deployment_manager.py`  
  Atomic install, activation, rollback, and state updates.
- `src/agent/agent/artifact.py`  
  HTTP(S) download and archive extraction helpers.
- `src/agent/agent/executor.py`  
  Start/stop process execution with PID tracking.
- `src/agent/agent/state_store.py`  
  Persistent state storage (JSON).

## Filesystem Layout

```
<ROOT>/
  stacks/
    <stack_name>/
      releases/
        <version>/
        <version>.tmp/
      current -> releases/<version>
      previous -> releases/<version>
      logs/
        <version>.log
      state.json
  incoming/
```

`<ROOT>` defaults to `/var/lib/muto` and is configurable.

## Configuration

The device agent reads JSON configuration from `MUTO_CONFIG` (or defaults).

Sample config: `config/device_agent.json`

Systemd unit template: `config/muto-device-agent.service`

Key fields:

- `storage.root_dir`
- `downloads.retries`, `downloads.timeout_seconds`
- `executor.start_grace_seconds`
- `symphony.topic_prefix`, `symphony.request_topic`, `symphony.response_topic`
- `symphony.mqtt.host`, `symphony.mqtt.port`, `symphony.mqtt.user`

## Observability

Each device reports:

- current version
- previous version
- deployment state
- last failure reason
- timestamps (install, activation, rollback)

Status is available through the Symphony provider `get` endpoint.
