# Eclipse Muto - Running Examples

This guide demonstrates a minimal deployment and rollback flow using release artifacts.

## Example: Deploy and Roll Back

### 1) Create Two Releases

```bash
tar -czf demo-1.0.0.tar.gz ./ros2_ws
tar -czf demo-1.0.1.tar.gz ./ros2_ws
sha256sum demo-1.0.0.tar.gz
sha256sum demo-1.0.1.tar.gz
```

Upload both archives to your HTTP(S) artifact server.

### 2) Define the Stack in Symphony

Create a solution component where `properties.data` contains:

```json
{
  "name": "demo-stack",
  "version": "1.0.0",
  "artifact_uri": "https://artifacts.example.com/demo-1.0.0.tar.gz",
  "checksum": "sha256:<sha256>",
  "start_command": "./run.sh",
  "working_directory": "."
}
```

Deploy the instance targeting your device.

### 3) Update to v1.0.1 and Force a Failure

Update the same component data to version `1.0.1`, but use a broken start command:

```json
{
  "name": "demo-stack",
  "version": "1.0.1",
  "artifact_uri": "https://artifacts.example.com/demo-1.0.1.tar.gz",
  "checksum": "sha256:<sha256>",
  "start_command": "./missing.sh"
}
```

The device agent will attempt to install and start `1.0.1`, detect failure, and automatically roll back to `1.0.0`.

### 4) Observe Status

Check device status through Symphony `get` or inspect:

```
/var/lib/muto/stacks/demo-stack/state.json
```

Expected fields:

- `current` is `1.0.0`
- `deployment.state` is `running`
- `deployment.last_failure` describes the startup failure
