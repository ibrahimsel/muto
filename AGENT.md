# Eclipse Muto MVP

## Mission

Build a **reliable, offline-tolerant fleet orchestrator** that provides **automated orchestration of ROS 2 stacks for vehicle fleets** using **declarative desired state + on-vehicle reconciliation**.
No remote command execution. No Kubernetes-on-robots. Cloud declares intent. Vehicles decide when and how to apply safely.

## Non-Negotiable Principles

1. **Desired State is the only control API.**
2. **Vehicle-side safety gates are authoritative.**
3. **Offline is normal.** Vehicles converge using cached state and artifacts.
4. **Evidence over vibes.** Every action produces structured reports with reason codes.
5. **Rollback is mandatory.** A/B slots or previous-known-good required.

---

## MVP Scope (Ship This, Nothing More)

### Included

* Stack release registry metadata
* Desired state creation and per-vehicle latest fetch
* Vehicle agent reconcile loop with safety gates
* Artifact cache with rollback
* MQTT notify + HTTP fetch
* Reconcile and health reporting
* Canary rollout plus one wave
* Pause and rollback

### Excluded

* Fancy UI
* Complex RBAC
* Multi-tenant billing
* Service mesh
* Full policy DSL

---

## Architecture

### Planes

* **Control Plane (Cloud):** intent, rollout, inventory, audit
* **Connectivity Plane:** MQTT broker with retained notify
* **Data Plane (Vehicle):** Muto Agent reconciles locally

### Cloud runs on Kubernetes. Vehicles do not.

---

## Core Data Models

### StackRelease (immutable)

```yaml
stackName: autonomy-core
version: 1.0.0
manifestDigest: sha256:...
artifacts:
  - type: oci_image
    ref: registry/autonomy-core:1.0.0
    digest: sha256:...
compatibility:
  vehicleModels: [vehicle-v3]
  os: [ubuntu-22.04]
  rosDistro: [humble]
resourceProfile:
  diskMbMin: 8000
healthModel:
  readinessTimeoutSec: 180
security:
  signatureRef: cosign://...
```

### DesiredState (revisioned)

```json
{
  "desiredStateRevision": "dsr_...",
  "target": { "type": "fleetSelector", "selector": "model=vehicle-v3 AND ring=1" },
  "stack": { "name": "autonomy-core", "version": "1.0.0", "manifestDigest": "sha256:..." },
  "policy": {
    "applyWindow": { "startLocal": "02:00", "endLocal": "05:00" },
    "requireVehicleStationary": true,
    "denyIfAutonomousMode": true,
    "requireBatteryPctMin": 35
  },
  "rollout": { "strategy": "canary", "waveId": "wave-1" },
  "correlationId": "uuid"
}
```

### ReconcileReport

```json
{
  "vehicleId": "vehicle-v3-0142",
  "desiredStateRevision": "dsr_...",
  "status": "converged",
  "reasonCodes": [],
  "timings": { "applySec": 48, "readinessSec": 61 }
}
```

---

## Standard Reason Codes (Enum)

```
VERIFY_SIGNATURE_FAILED
INCOMPATIBLE_OS
INCOMPATIBLE_ROS
INSUFFICIENT_DISK
DENY_MOVING
DENY_AUTONOMOUS_MODE
DENY_LOW_BATTERY
DENY_OUTSIDE_WINDOW
ARTIFACT_DOWNLOAD_FAILED
APPLY_FAILED
READINESS_TIMEOUT
CRASH_LOOP_DETECTED
ROLLBACK_APPLIED
```

No free text failures allowed.

---

## Vehicle Agent Behavior

### Reconcile Loop

Trigger: startup, periodic poll (30s jitter), MQTT notify, drift detected.

Steps:

1. Fetch latest DesiredState or use cached
2. Verify signature and compatibility
3. Evaluate safety gates
4. Ensure artifacts cached
5. Apply via runtime adapter
6. Evaluate readiness
7. Report result
8. Rollback on failure

### Safety Gates (Local Authority)

* Vehicle moving
* Autonomous mode enabled
* Battery below threshold
* Outside apply window
* Disk below minimum

If blocked, report `status=blocked` with reason codes.

### Rollback

* Keep previous-known-good
* On failure or crash loop, restore and report `ROLLBACK_APPLIED`

---

## Messaging Interfaces

### MQTT Topics

Base: `muto/<tenant>/vehicles/<vehicleId>/...`

* `presence` vehicle -> cloud, QoS 1, retained
* `desired/notify` cloud -> vehicle, QoS 1, retained
* `reports/reconcile` vehicle -> cloud, QoS 1
* `reports/health` vehicle -> cloud, QoS 0 or 1

Desired notify payload contains revision id and HTTP fetch URL.

---

## Cloud HTTP API (Minimal)

Endpoints:

* `POST /vehicles`
* `PATCH /vehicles/{id}`
* `GET /vehicles?selector=...`
* `POST /releases`
* `GET /releases/{stack}/{version}`
* `POST /desired-states`
* `GET /desired-states/latest?vehicleId=...`
* `POST /reports/reconcile`
* `POST /reports/health`
* `POST /rollouts`
* `POST /rollouts/{id}/pause`
* `POST /rollouts/{id}/rollback`

Selectors are simple `key=value AND key=value`.

---

## Rollout Algorithm (MVP)

1. Select fleet via selector
2. Pick 1 canary per region-model
3. Publish DesiredState to canaries
4. Wait for success gates
5. If fail, rollback and abort
6. Publish DesiredState to remaining vehicles
7. Monitor and rollback if thresholds exceeded

Gates:

* Converged >= 95 percent
* No crash loops
* No readiness timeouts

---

## Repo Layout

```
/agent
  /reconcile
  /safety
  /runtime
  /cache
/cloud
  /api
  /rollout
  /inventory
/spec
  reason-codes.yaml
  mqtt-topics.md
  api.yaml
```

---

## Definition of Done

* Vehicle converges offline
* Canary rollout works
* Rollback proven
* All failures emit reason codes
* No imperative remote commands exist

---

## Explicit Anti-Goals

* No kubernetes-on-vehicle
* No ad-hoc SSH
* No mutable stack definitions
* No silent failures

---

## If You Deviate

If any tool proposes:

* “Just exec a command”
* “Let cloud override safety”
* “We can skip rollback for now”

Reject it. That path does not scale and will brick hardware.

---

## Final Instruction

Build the spine first. Everything else is decoration.

---

## Implementation Plan (Zero to MVP)

1. Capture the required API surface from `docs/api/openapi.yaml` and the
   frontend docs in `docs/frontend`, then expand the contract only where the
   MVP needs write/ingest endpoints and agent polling.
2. Scaffold a FastAPI service under `src/composer/api` with routers, Pydantic
   models, and a consistent response envelope (`data`, `metadata`, `timestamp`)
   to support pagination, filtering, and search.
3. Implement an in-memory data store with deterministic seed data so the UI
   can run without a fleet, plus CRUD for vehicles, releases, desired states,
   rollouts, reports, and audit entries.
4. Add agent-facing ingest endpoints for vehicle status, timeline events, and
   reports so the frontend can poll a single API instance for updates.
5. Write unit tests for the API models, storage, and key endpoints using
   `unittest` and FastAPI's test client.
6. Document how to run the API locally and how to seed or reset data for
   demos and testing.
