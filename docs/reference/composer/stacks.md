# Stack Types Reference

A **stack** is Muto's unit of deployment — a declarative description of the ROS 2 nodes, parameters, and launch configuration that should run on a device. Stacks are registered as digital twin features in Eclipse Ditto and delivered to devices over MQTT.

This document covers every stack type, how the Composer resolves and launches them, and how the system learns or derives the desired graph state for drift detection.

## Stack Types at a Glance

| Content Type | Alias (deprecated) | Handler | Desired State Source | Build Required |
|---|---|---|---|---|
| `stack/declarative` | `stack/json` | `JsonStackHandler` | Extracted from manifest | No |
| `stack/workspace` | `stack/archive` | `ArchiveStackHandler` | Learned from deployment | Yes (colcon) |
| `stack/native` | — | `NativeStackHandler` | Learned from deployment | No |
| `stack/legacy` | `stack/ditto` | `DittoStackHandler` | Extracted from manifest | No |

## Handler Selection

When a stack payload arrives, the `StackTypeRegistry` iterates handlers in fixed priority order and returns the first one whose `can_handle()` returns `True`:

1. **JsonStackHandler** — matches `metadata.content_type` of `stack/json` or `stack/declarative`
2. **ArchiveStackHandler** — matches `stack/archive` or `stack/workspace`
3. **NativeStackHandler** — matches `stack/native`
4. **DittoStackHandler** — catch-all for legacy payloads with no `content_type` or `stack/ditto`/`stack/legacy`

## Stack Lifecycle

Every handler implements four operations:

| Operation | When | What happens |
|---|---|---|
| **PROVISION** | Before first launch | Download, extract, build, or validate the stack artifacts |
| **START** | Deploy the stack | Launch the ROS 2 nodes |
| **KILL** | Tear down the stack | Terminate all managed processes |
| **APPLY** | Update a running stack | Kill then start (or incremental diff for JSON/legacy) |

---

## `stack/declarative`

The simplest stack type. The entire launch graph is defined inline in the manifest JSON — no external files, no build step.

### When to use

- Nodes come from packages already installed on the device (e.g. `demo_nodes_cpp`, `turtlesim`)
- You want the desired state to be fully known from the manifest
- You need fast deployment with no build overhead

### Manifest structure

```json
{
  "metadata": {
    "name": "my-stack",
    "content_type": "stack/declarative",
    "description": "...",
    "version": "1.0.0"
  },
  "launch": {
    "node": [
      {
        "name": "talker",
        "pkg": "demo_nodes_cpp",
        "exec": "talker",
        "namespace": "/demo",
        "output": "screen",
        "param": [{"name": "frequency", "value": "2.0"}],
        "remap": [{"from": "/chatter", "to": "/my_topic"}]
      }
    ],
    "composable": []
  }
}
```

### Node fields

| Field | Required | Default | Description |
|---|---|---|---|
| `pkg` | Yes | — | ROS 2 package name |
| `exec` | Yes | — | Executable name |
| `name` | Yes | — | Node name |
| `namespace` | No | `$MUTONS` env var or `""` | Node namespace |
| `output` | No | `"both"` | `"screen"`, `"log"`, or `"both"` |
| `param` | No | `[]` | `[{"name": "key", "value": "val"}]` or `[{"from": "path/to/yaml"}]` |
| `remap` | No | `[]` | `[{"from": "/old", "to": "/new"}]` |
| `args` | No | `""` | Extra CLI arguments; supports `$(find pkg)`, `$(env VAR)`, `$(arg name)` |
| `ros_args` | No | `""` | ROS-specific arguments |
| `launch-prefix` | No | `null` | e.g. `"gdb --args"`, `"xterm -e"` |
| `env` | No | `[]` | Environment variable overrides |
| `lifecycle` | No | `""` | Dict of `{"start": [...], "kill": [...]}` for managed (lifecycle) nodes |
| `plugin` | No | `""` | Component plugin class (composable nodes only) |
| `if` / `unless` | No | `""` | Conditional launch conditions |

### Lifecycle details

- **PROVISION**: No-op. Returns immediately.
- **START**: Kills any previously running instance for the same stack hash. Creates a `Ros2LaunchParent`, builds a `LaunchDescription` from the `node` and `composable` arrays, and starts a child launch service process.
- **KILL**: Sends `SIGKILL` to all tracked child processes.
- **APPLY**: Performs a diff between the running and desired node sets. Nodes present in both are kept alive. Removed nodes are killed. Added nodes are launched. This enables incremental reconfiguration.

### Desired state resolution

Nodes are **extracted directly from the manifest**. Each node's `pkg`, `exec`, `name`, and `namespace` are known, which means the Daemon can attempt per-node restarts on drift without a full relaunch.

---

## `stack/workspace`

A self-contained ROS 2 colcon workspace delivered as a compressed archive. The Composer downloads it, installs dependencies, builds it, and launches a designated launch file.

### When to use

- Deploying custom application code that isn't pre-installed on the device
- The stack needs a `colcon build` step
- You want reproducible, versioned deployments of full workspaces

### Manifest structure

```json
{
  "metadata": {
    "name": "my-workspace-stack",
    "content_type": "stack/workspace",
    "description": "...",
    "version": "1.0.0"
  },
  "launch": {
    "data": "<base64-encoded-tarball>",
    "properties": {
      "algorithm": "sha256",
      "checksum": "<hex-digest-of-archive>",
      "launch_file": "launch/my_stack.launch.py",
      "command": "launch",
      "flatten": true
    }
  }
}
```

**URL-based variant** (download instead of inline):

```json
{
  "launch": {
    "url": "https://example.com/stack.tar.gz",
    "headers": {"Authorization": "Bearer TOKEN"},
    "timeout": 60,
    "checksum": "<hex>",
    "algorithm": "sha256",
    "properties": {
      "launch_file": "launch/my_stack.launch.py",
      "command": "launch",
      "subdir": "optional_subdir",
      "flatten": true
    }
  }
}
```

### Launch properties

| Field | Required | Default | Description |
|---|---|---|---|
| `launch_file` | Yes | — | Relative path to the launch file inside the workspace |
| `command` | No | `"launch"` | Launch command (`"launch"`) |
| `flatten` | No | `true` | Flatten single top-level directory after extraction |
| `subdir` | No | `null` | Use a subdirectory inside the archive as the workspace root |
| `algorithm` | No | `"sha256"` | Checksum hash algorithm |
| `checksum` | No | — | Expected checksum hex string for integrity verification |

### Lifecycle details

- **PROVISION**:
  1. Downloads (URL) or decodes (base64) the archive.
  2. Verifies checksum if provided.
  3. Extracts to `~/.muto/workspaces/<stack-name>/`.
  4. Applies flatten/subdir logic.
  5. Writes `.muto_artifact.json` state file. On subsequent provisions, if the artifact state matches, extraction is skipped entirely.
  6. Runs `rosdep install --from-path <workspace> --ignore-src -r -y`.
  7. Runs `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`.
  8. On build failure, cleans `build/` and `install/` directories and retries once.
- **START**: Sources the workspace's `install/setup.bash`, then calls `ros2 launch <launch_file>`.
- **KILL**: Terminates the launch process.
- **APPLY**: Kill then start.

### Desired state resolution

Since the launch file can spawn arbitrary nodes not enumerable from the manifest, the desired state is **learned from deployment** via multi-sample stabilization (see [Learn from Deployment](#learn-from-deployment) below). The `package_name` and `executable` fields in the learned baseline are empty — the system cannot infer them from graph introspection.

**Implication for drift recovery**: When a node from a workspace stack goes missing, the Daemon cannot restart it individually (no package/executable info). Recovery escalates to a full launch-file relaunch.

---

## `stack/native`

Launches a `.launch.py` file that already exists on the device filesystem. No archive download, no build. Use this for pre-installed ROS 2 packages or workspaces that were deployed outside of Muto.

### When to use

- The software is already installed via `apt`, a pre-built workspace, or a container bind-mount
- You want Muto to manage the lifecycle of locally-installed ROS 2 applications
- No build step is needed

### Manifest structure

```json
{
  "metadata": {
    "name": "my-native-stack",
    "content_type": "stack/native",
    "description": "...",
    "version": "1.0.0"
  },
  "launch": {
    "file": "/absolute/path/to/my_stack.launch.py",
    "args": {
      "arg_name": "arg_value"
    }
  },
  "source": {
    "setup_files": [
      "/opt/ros/humble/setup.bash",
      "/home/user/ws/install/setup.bash"
    ]
  },
  "criticality_map": {
    "/my_critical_node": "mission",
    "/my_optional_node": "standard"
  }
}
```

### Manifest fields

| Field | Required | Default | Description |
|---|---|---|---|
| `launch.file` | Yes | — | Path to the `.launch.py` file on the device |
| `launch.args` | No | `{}` | Launch arguments as `{key: value}`, passed as `key:=value` |
| `source.setup_files` | No | `[]` | Shell scripts to `source` before launching |
| `criticality_map` | No | `{}` | Maps node FQNs to criticality levels for reconciliation |

### Lifecycle details

- **PROVISION**: Validates that `launch.file` exists on disk. Sources each file in `source.setup_files` into the process environment (runs `bash -c 'source <file> && env'` and merges into `os.environ`).
- **START**: Kills any previous instance. Builds a shell command: `source setup1.bash && source setup2.bash && ros2 launch <file> key:=value ...` and spawns it via `subprocess.Popen` with `os.setsid()` (new process group for clean teardown).
- **KILL**: Sends `SIGTERM` to the entire process group. Waits 10 seconds, then escalates to `SIGKILL` if still alive.
- **APPLY**: Kill then start. No incremental diffing.

### Desired state resolution

Like workspace stacks, native stacks use **learn from deployment** via multi-sample stabilization. The launch file is opaque — the system cannot enumerate nodes from it without actually running it.

---

## `stack/legacy`

The original Ditto-style stack format from before MEP-0001. Supports two sub-formats: node/composable arrays and script-based stacks. This handler is the catch-all — it fires only if no other handler matched.

### When to use

- Backward compatibility with existing Ditto-managed stacks
- Simple stacks that don't need the `metadata` envelope

### Sub-format A: Node/Composable arrays

```json
{
  "name": "my-legacy-stack",
  "context": "my_deployment",
  "stackId": "org.eclipse.muto:my_stack",
  "node": [
    {"name": "talker", "pkg": "demo_nodes_cpp", "exec": "talker"},
    {"name": "listener", "pkg": "demo_nodes_cpp", "exec": "listener"}
  ],
  "composable": [
    {
      "name": "my_container",
      "namespace": "",
      "package": "rclcpp_components",
      "executable": "component_container",
      "node": [
        {"pkg": "composition", "plugin": "composition::Server", "name": "server"},
        {"pkg": "composition", "plugin": "composition::Client", "name": "client"}
      ]
    }
  ]
}
```

### Sub-format B: Script-based

```json
{
  "name": "my-script-stack",
  "context": "my_deployment",
  "stackId": "org.eclipse.muto:my_script_stack",
  "on_start": "ros2 run demo_nodes_cpp talker",
  "on_kill": "pkill -f 'demo_nodes_cpp.*talker'"
}
```

Script-based stacks only have process-level monitoring — no graph-level drift detection.

### Lifecycle details

- **PROVISION**: No-op.
- **START** (node/composable): Creates a `Ros2LaunchParent` and launches via the `Stack` model, same as declarative.
- **START** (script-based): Delegated to the plugin. The handler returns immediately.
- **KILL**: Same as declarative — looks up the `Ros2LaunchParent` by stack hash and kills it.
- **APPLY**: Kill then start. For node/composable stacks, uses `Stack.apply()` which diffs the current and desired node sets.

### Desired state resolution

For node/composable sub-format, nodes are **extracted directly from the manifest** (same as declarative). For script-based sub-format, the desired state is opaque — only process-level health is available.

---

## Learn from Deployment

For `stack/workspace` and `stack/native` stacks, the set of nodes that will run is not known ahead of time — it depends on the launch file, which is opaque to the manifest. The system learns the desired state by observing the live graph after deployment.

This is implemented in the `GraphReconciliationManager` ([graph_reconciliation.py](../../src/composer/muto_composer/subsystems/graph_reconciliation.py)).

### How it works

1. **Deployment completes**: The `ORCHESTRATION_COMPLETED` event fires. The Composer pauses drift detection.
2. **Multi-sample stabilization begins**: A background thread takes N snapshots (default: 3) of the live ROS 2 graph at fixed intervals (default: 15 seconds each) by calling the Daemon's `GetGraphState` service.
3. **Baseline = intersection**: The baseline is the set of node FQNs present in **all** N samples. Nodes that appear in some samples but not all are classified as transient and excluded.
4. **Ignored prefixes filtered**: System nodes and nodes matching `ignored_prefixes` (e.g. `/rviz`, `/rosout`) are removed from the baseline.
5. **Desired state published**: The learned baseline is published to `/muto/daemon/desired_state` as a `DesiredState` message. Drift detection resumes.

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `graph_observer_stabilization_samples` | 3 | Number of graph snapshots to collect |
| `graph_observer_stabilization_sec` | 15.0 | Seconds between snapshots |
| `ignored_prefixes` | `[]` | Node namespace prefixes to exclude from the baseline |

### Why intersection?

A single snapshot can capture transient startup nodes (parameter bridges, lifecycle managers that exit after setup, etc.). By requiring a node to be present in every sample, these transients are naturally excluded. The tradeoff is that stabilization takes `N * interval` seconds before drift detection begins.

### Limitations

- `package_name` and `executable` are **empty** in the learned baseline — ROS 2 graph introspection APIs do not expose this information.
- Because of this, drift recovery for workspace/native stacks escalates to a **full launch-file relaunch** rather than restarting individual nodes.
- If a node takes longer than `N * interval` seconds to start, it will be excluded from the baseline.

### Stack types and desired state strategy

```
stack/declarative ──┐
stack/json ─────────┤── Desired state EXTRACTED from manifest
stack/legacy ───────┤   (pkg, exec, name, namespace all known)
stack/ditto ────────┘

stack/workspace ────┐
stack/archive ──────┤── Desired state LEARNED from deployment
stack/native ───────┘   (pkg, exec unknown; full relaunch on drift)
```

---

## Ditto Thing Registration

Stacks are registered to the Eclipse Ditto digital twin server as "things". The stack manifest is nested inside `features.stack.properties`:

```json
{
  "thingId": "org.eclipse.muto.sandbox:my_stack",
  "policyId": "org.eclipse.muto.sandbox:my_stack",
  "definition": "org.eclipse.muto:Stack:0.0.1",
  "attributes": {
    "type": "simulator"
  },
  "features": {
    "stack": {
      "properties": {
        "name": "My Stack",
        "context": "my_deployment",
        "stackId": "org.eclipse.muto.sandbox:my_stack",
        "node": [...],
        "launch": {...},
        "metadata": {
          "name": "My Stack",
          "content_type": "stack/declarative",
          "description": "...",
          "version": "1.0.0"
        }
      }
    }
  }
}
```

- `thingId` and `policyId` must be unique per stack.
- `stackId` inside properties should match the `thingId`.
- See [docs/samples/talker-listener/](../../docs/samples/talker-listener/) for working examples of all four stack types in Ditto registration format.

---

## Expression Substitution

The `args`, `param`, and `remap` fields support expression substitution at resolution time:

| Expression | Description |
|---|---|
| `$(find <package>)` | Resolves to the package's share directory via `ament_index` |
| `$(env <VAR>)` | Resolves to `$VAR` environment variable; errors if unset |
| `$(optenv <VAR>)` | Resolves to `$VAR` or empty string if unset |
| `$(arg <name>)` | Resolves to a previously declared arg value from the manifest |
