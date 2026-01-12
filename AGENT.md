## Project
Eclipse Muto  
A domain-specific deployment orchestrator for ROS2 workspaces

## Product Definition
Eclipse Muto is an open-source deployment orchestrator that enables users to **bulk deploy, switch, and rollback versions of existing ROS2 workspaces across multiple edge devices without SSH**.

Muto treats user software as an **opaque artifact**. Users are not required to adapt, refactor, or restructure their ROS2 workspaces. The only user responsibilities are:
- versioning their software
- compressing it into an archive
- providing instructions for how to start (and optionally stop) the software

Muto does **not** manage hardware compatibility, system dependencies, or correctness of the software. Those are explicitly the user’s responsibility.

---

## Non-Goals
- No container runtime support
- No ROS graph awareness or enforcement
- No mission orchestration or runtime autonomy logic
- No hardware abstraction, detection, or validation
- No requirement for users to modify their ROS2 codebase
- No UI required for v1 (CLI is sufficient)

---

## Core Value Proposition
Muto replaces:
- manual SSH into each device
- manual copying of release artifacts
- manual restarts
- manual rollbacks

with:
- versioned artifacts
- bulk deployment
- atomic activation
- automatic rollback on failure
- observability of what version is running where

---

## System Overview

Muto's architecture is realized using [Eclipse Symphony](https://eclipse-symphony.github.io/symphony/) as its core control plane, combined with a custom device agent.

### High-Level Components
1. **Symphony Control Plane**
   - Muto leverages Symphony to manage the desired state of each device (target).
   - It stores release metadata (`Stack` definitions) and tracks the deployment state per device.
   - Deployment commands are issued by Symphony to the agents via MQTT.
   - This component is external to this repository and is consumed as a service.
2. **Artifact Store**
   - A user-provided storage location (e.g., any HTTP server) that hosts the compressed workspace archives. Muto does not provide this component directly.
3. **Muto Device Agent (`src/agent`)**
   - The agent runs on each edge device, registers itself as a Symphony "target", and subscribes to MQTT topics for commands.
   - It is responsible for downloading artifacts, executing the atomic installation (FR-4), and managing the software's lifecycle.
   - The core logic resides in `src/agent/agent/muto_agent.py`.
4. **Muto Composer (`src/composer`)**
   - A utility package responsible for validating and composing the Symphony-compatible JSON models that define a Muto `Stack`.
5. **Runtime Executor (`src/agent/agent/command_executor.py`)**
   - A component of the Device Agent that abstracts the starting and stopping of user software based on the `start_command` and `stop_command` from the release metadata.

---

## Core Concepts

### Workspace Artifact
A user-provided compressed archive (eg `.tar.gz`) containing an existing ROS2 workspace or runtime directory.

Muto does not inspect or modify its contents.

### Release
A versioned reference to a workspace artifact plus metadata describing how to run it.

### Desired State
The version of a release that a device *should* be running.

### Actual State
The version of a release that a device *is* running.

Muto continuously reconciles actual state toward desired state.

---

## Functional Requirements

### FR-1 Artifact Handling
- Muto shall accept user-provided compressed archives as opaque blobs.
- Muto shall never modify archive contents.
- Muto shall support versioned artifacts identified by:
  - name
  - version string
  - checksum (sha256)

### FR-2 Release Metadata
Each release shall include minimal metadata:
- `name`
- `version`
- `artifact_uri`
- `checksum`
- `start_command`
- `stop_command` (optional)
- `environment` (optional key-value map)
- `working_directory` (optional)

No ROS-specific schema shall be required.

---

### FR-3 Artifact Transport
- Devices shall download artifacts from a user-configured location.
- Supported transport for v1:
  - HTTP(S)
- Artifact download shall:
  - verify checksum
  - support retry on failure
  - fail without modifying the active version if verification fails

---

Absolutely. Here is a **clean, corrected, and implementation-ready rewrite of FR-4 Atomic Installation**, designed to drop directly into your `AGENT.md` without ambiguity or corruption.

This version is precise about filesystem semantics and failure guarantees, which is what you need when you start coding.

---

### FR-4 Atomic Installation

Muto shall install and activate stack versions in a manner that guarantees **atomicity, crash safety, and rollbackability**.

#### FR-4.1 Installation Layout

Each device shall maintain the following directory structure:

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

Where:

* `<ROOT>` is a configurable base directory (default: `/var/lib/muto`)
* `<stack_name>` corresponds to the stack `name`
* `<version>` corresponds to the stack `version`

---

#### FR-4.2 Download Phase

* The device agent shall download the stack artifact into `<ROOT>/incoming/`.
* The active stack (`current`) shall not be modified during this phase.
* Partial or failed downloads shall not affect existing releases.

---

#### FR-4.3 Extraction Phase

* The downloaded artifact shall be extracted into:

  ```
  <ROOT>/stacks/<stack_name>/releases/<version>.tmp/
  ```
* Extraction shall not overwrite any existing release directory.
* If extraction fails, the temporary directory shall be removed and the operation aborted.

---

#### FR-4.4 Verification Phase

Before activation, the agent shall:

* Verify the artifact checksum matches the stack definition.
* Ensure the extracted directory exists and is non-empty.
* Verify required runtime fields (`start_command`) are present.

If verification fails:

* The temporary directory shall be deleted.
* The active stack shall remain unchanged.

---

#### FR-4.5 Promotion Phase (Atomic)

* Upon successful verification, the agent shall atomically rename:

  ```
  <version>.tmp -> <version>
  ```
* This operation shall be atomic at the filesystem level.
* If a release directory with the same `<version>` already exists, the installation shall be treated as idempotent and skipped.

---

#### FR-4.6 Activation Pointer Switch

* The agent shall atomically update symlinks:

  * `previous` shall point to the current `current` target (if any).
  * `current` shall point to `releases/<version>`.
* Symlink updates shall be atomic operations.
* At no point shall `current` point to a partially installed release.

---

#### FR-4.7 Failure Guarantees

The following guarantees must hold:

* A device reboot or power loss at any stage shall not corrupt the previously active stack.
* `current` shall always point to a fully extracted, verified release.
* Temporary directories shall be safe to clean up on agent restart.

---

#### FR-4.8 Post-Install Cleanup

* The agent may remove unused temporary directories after successful activation.
* The agent shall retain at least:

  * the currently active release
  * the previously active release

Old releases may be garbage collected according to policy, but never during an active installation.

---

#### FR-4.9 Restart Safety

* On agent startup, the agent shall:

  * detect incomplete installations (`*.tmp`)
  * remove or quarantine them
  * restore a consistent state without manual intervention

---

This version is intentionally **filesystem-precise**, **failure-oriented**, and **free of product creep**.

You can now implement this directly with confidence that:

* every operation is reversible
* crashes are survivable
* rollback is guaranteed by construction

If you want, the next natural step is to define the **deployment state machine** that drives FR-4 deterministically.

---

### FR-5 Runtime Execution
- Muto shall execute user-provided `start_command` to activate a release.
- Execution shall occur in the context of the installed release directory.
- Environment variables provided in release metadata shall be applied.
- Muto shall track the started process (PID).

---

### FR-6 Failure Detection
A deployment shall be considered failed if:
- the start command exits non-zero
- the process exits unexpectedly within a configurable grace period
- the executor reports inability to start the process

No ROS-aware health checks are required for v1.

---

### FR-7 Automatic Rollback
- On deployment failure, Muto shall automatically revert to the previous version.
- Rollback shall:
  - stop the failed version
  - atomically switch `current` back
  - restart the previous version
- Rollback events shall be recorded and reported.

---

### FR-8 Bulk Deployment
- Muto shall support deploying a release to multiple devices with one command.
- Devices shall reconcile desired version independently.
- Failure on one device shall not block others.

---

### FR-9 Idempotency
- Deployment commands shall be idempotent.
- Reissuing a deployment command for an already-running version shall be a no-op.

---

### FR-10 Observability
Each device shall report:
- current version
- previous version
- deployment state
- last failure reason
- timestamps of install, activation, rollback

Logs shall be retrievable per device.

---

### FR-11 CLI Interface (v1)
Interaction with the Muto control plane is handled through the underlying Eclipse Symphony API and its associated tooling. For v1, there is no custom Muto-specific CLI. Instead, a user will:
- Upload their release artifact to an HTTP server of their choice.
- Use helper scripts (like those in `samples/symphony/`) or direct API calls to Symphony to define and update the `solution` and `instance` objects.
- Trigger deployments by updating the desired `solution` for a given Symphony `target` (device).
- Query device state using Symphony's API or CLI.

No GUI required.

---

## Device Agent Requirements

### DA-1 Execution Model
- Device agent runs as a system service.
- Agent must restart automatically on crash.
- Agent must resume reconciliation after reboot.

---

### DA-2 Executor Interface
The device agent shall use an executor abstraction with the following responsibilities:
- start(command, env, cwd)
- stop(command or pid)
- status()

The executor must not interpret ROS semantics.

---

### DA-3 Persistence
The agent shall persist:
- current version
- previous version
- in-progress deployment state

Persistence must survive reboot.

---

### DA-4 Offline Behavior
- If disconnected, the agent shall continue running the current version.
- Desired state reconciliation resumes when connectivity returns.

---

## Non-Functional Requirements

### NFR-1 Reliability
- No deployment operation may corrupt the active version.
- Rollback must succeed even if the agent crashes mid-deploy.

---

### NFR-2 Security
- Artifact integrity must be verified via checksum.
- Transport must be encrypted when using HTTP(S).
- Command execution must run with least privilege possible.

---

### NFR-3 Maintainability
- Clear separation between:
  - control plane logic
  - device agent logic
  - executor implementation
- Codebase must be modular and testable.

---

### NFR-4 Extensibility
- Future versions may add:
  - optional health probes
  - additional executors
  - additional transports
- v1 design must not preclude these extensions.

---

## Explicit Product Boundaries

Muto **guarantees**:
- correct version distribution
- atomic activation
- rollback on failure
- visibility into deployment state

Muto **does not guarantee**:
- software correctness
- hardware compatibility
- runtime safety
- ROS graph health

---

## Minimum Demo Acceptance Criteria
- Device runs version `v1.0.0`
- Deploy `v1.0.1` remotely
- `v1.0.1` fails to start
- Muto automatically rolls back to `v1.0.0`
- No SSH interaction required
- Logs clearly show failure and rollback

---

## Implementation Priority Order
1. Device agent core loop and persistence
2. Atomic install and version switching
3. Executor abstraction
4. Rollback logic
5. Control plane CLI
6. Bulk deployment support

---

## License
EPL v2

## Example Stack Model

{
  "thingId": "org.eclipse.muto.sandbox:archive",
  "policyId": "org.eclipse.muto.sandbox:archive",
  "definition": "org.eclipse.muto:Stack:0.0.1",
  "attributes": {
    "type": "archive"
  },
  "features": {
    "stack": {
      "properties": {
        "artifact": {
          "uri": "https://artifacts.example.com/towtruck-autonomy-1.0.1.tar.gz",
          "checksum": "sha256:8c4f0d3e1e9b1a8f2c2c9c7a3f4d5b6a9e8f7c6d5b4a3a2f1e0d9c8b7a6"
        },
        "runtime": {
          "working_directory": ".",
          "start_command": "./run.sh",
          "stop_command": "pkill -f run.sh",
          "environment": {
            "ROS_DOMAIN_ID": "42",
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "LOG_LEVEL": "info"
          }
        }
      }
    }
  }
}