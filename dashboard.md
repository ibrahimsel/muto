# Eclipse Muto Dashboard Requirements

## Mission

Build a dashboard that makes fleet orchestration **operationally usable**: show desired vs actual state, rollout progress, and why vehicles are not converging. The dashboard is not a toy UI. It is a control surface for **declaring intent** and **watching evidence**.

The dashboard must be able to answer, in under 10 seconds:

* What is running on each vehicle right now?
* What should be running?
* Who is stuck, why, and since when?
* Is the rollout safe to continue?
* What changed, by whom, and what happened after?

---

## MVP Scope

### Included

* Fleet overview and filtering
* Vehicle details page
* Releases view (what can be deployed)
* Desired state creation (target by selector or list)
* Rollout monitoring (canary + wave)
* Pause rollout and initiate rollback
* Reason-code centered troubleshooting
* Audit trail of desired state changes
* Basic auth (single-tenant) and role separation: viewer vs operator

### Excluded

* Multi-tenant org hierarchy
* Complex RBAC policy editor
* Deep analytics, long-term trend dashboards
* Advanced map UI, 3D twin visuals
* Custom dashboards per customer

---

## User Roles

### Viewer

* Read-only access to fleet state, vehicle status, reports, history

### Operator

* Create desired state revisions
* Start rollout
* Pause rollout
* Trigger rollback
* Add or edit vehicle tags (optional in MVP)

No other roles in MVP.

---

## Core User Stories

### Fleet Overview

1. As an operator, I can see the fleet status at a glance:

* total vehicles
* converged count
* blocked count
* failed count
* offline count
* in rollout count
* last update time distribution

2. As an operator, I can filter vehicles by tags:

* model, region, ring, hardware capability, ROS distro, runtime mode

3. As an operator, I can search by vehicleId.

4. As an operator, I can quickly identify the top reasons for non-convergence by reason code.

### Vehicle Details

1. As an operator, I can open a vehicle page and see:

* desired state revision and target stack
* actual running stack
* last reconcile report
* reason codes if blocked or failed
* safety signals (moving, autonomy mode, battery, window)
* artifact cache status (optional), last apply timestamps
* short timeline of events: desired notify, apply start, readiness, converge or rollback

2. As an operator, I can view the raw JSON of the latest desired state and reconcile report for debugging.

3. As an operator, I can see if a vehicle is offline and when it was last seen.

### Releases

1. As an operator, I can browse releases per stack:

* version list
* manifest digest
* compatibility summary
* signature presence
* readiness timeout and health model

2. As an operator, I can select a release and deploy it via desired state creation.

### Desired State Creation

1. As an operator, I can create a desired state revision by:

* selecting stack release
* choosing target: selector string or explicit list
* setting policy: apply window, stationary requirement, deny autonomous mode, min battery
* selecting rollout strategy: canary then one wave
* adding an operator comment

2. After creation, dashboard shows:

* the new desired state revision id
* targeted vehicle count
* link to rollout monitor

### Rollout Monitor

1. As an operator, I can see rollout progress:

* canary status per group (region-model)
* wave status
* converge percent
* failed and blocked counts
* gate evaluation state: pass/fail/pending
* auto rollback triggers if implemented

2. As an operator, I can pause the rollout.

3. As an operator, I can trigger rollback:

* rollback all vehicles updated in this rollout
* optionally rollback only a wave

### Audit Trail

1. As an operator, I can see:

* who created desired state revision
* when it was created
* what stack version and policy
* target selector or list
* correlation id
* outcome summary (converged percent, rollback occurred)

---

## Data Requirements (Inputs)

The dashboard consumes these sources:

* Vehicles inventory: tags, capabilities, lastSeenAt
* Desired state revisions: content, createdAt, createdBy, correlationId
* Releases: stackName, version, digests, compatibility, security refs
* Reconcile reports: status, reason codes, timings, attestation fields
* Health reports: heartbeat signals (moving, autonomy mode, battery, temps)

The UI must be reason-code driven, not log-text driven.

---

## Key Screens and Requirements

### 1) Fleet Overview Page

Components:

* Summary tiles: Total, Converged, Blocked, Failed, Offline
* Filter bar: tag filters, free text search, status filter
* Vehicles table:

  * vehicleId
  * tags: model, region, ring
  * desired stack version
  * actual stack version
  * status (converged, applying, blocked, failed, offline)
  * lastSeenAt
  * lastReconcileAt
  * top reason code
* Top reasons panel:

  * reasonCode -> count
* Rollouts widget:

  * active rollouts list with status

Performance:

* Table must support at least 5k vehicles with pagination.
* Filters should be server-side.

### 2) Vehicle Details Page

Sections:

* Header: vehicleId, model, region, ring, online/offline
* Desired vs Actual:

  * desired revision id
  * desired stack and policy
  * actual stack and runtime mode
* Status and reasons:

  * status pill
  * reason codes list
  * safety state snapshot
* Timeline:

  * desired notified
  * apply started
  * readiness achieved or failed
  * rollback if occurred
* Raw payload viewer:

  * latest desired state JSON
  * latest reconcile report JSON

### 3) Releases Page

* List stacks and versions
* Show compatibility and digests
* Show signature present/missing
* Action: Deploy this release

### 4) Create Deployment Page (Desired State Wizard)

Steps:

* Select stack release
* Choose target: selector or vehicle list
* Configure policy (defaults must be safe)
* Choose rollout strategy:

  * canary per group = 1 default
  * wave percent = 100 default for MVP
* Confirm and create

Validation:

* Must warn if apply window missing
* Must warn if stationary gate disabled
* Must warn if deny autonomous mode disabled
  Operators can override but dashboard must show red warnings.

### 5) Rollout Monitor Page

* Overview: rollout id, stack version, selector, createdBy, createdAt
* Gate panel: converge rate, failures, readiness timeouts, crash loops
* Canary section: per group status
* Wave section: progress and top failures
* Actions:

  * Pause
  * Rollback

---

## API Contracts (Minimal)

Dashboard requires these HTTP endpoints.

Read:

* `GET /vehicles?selector=...&status=...&page=...`
* `GET /vehicles/{vehicleId}`
* `GET /desired-states/{revision}`
* `GET /desired-states/latest?vehicleId=...`
* `GET /releases/{stackName}/{version}`
* `GET /releases?stackName=...`
* `GET /rollouts?status=running`
* `GET /rollouts/{rolloutId}`
* `GET /reports/reconcile?vehicleId=...&limit=...`
* `GET /reports/health?vehicleId=...&limit=...`

Write:

* `POST /desired-states`
* `POST /rollouts`
* `POST /rollouts/{rolloutId}/pause`
* `POST /rollouts/{rolloutId}/rollback`

If the backend does not have some read endpoints yet, add them. The dashboard should not scrape MQTT directly in MVP.

---

## State Model and Status Rules

Vehicle status computation:

* Offline: lastSeenAt older than threshold (default 5 minutes)
* Converged: last reconcile report status converged and actual matches desired revision
* Applying: last status applying
* Blocked: last status blocked
* Failed: last status failed
* Rolled back: last status rolled_back

Mismatch detection:

* If desired revision is newer than last reconcile report and vehicle is online, show “pending converge”.

---

## UX Requirements

* Every non-converged state must show a primary reason code immediately.
* Actions that can brick vehicles must require a confirmation step and show the policy gates enabled.
* Avoid clutter. Operators need speed.
* Raw JSON viewer must be copyable.

No charts required in MVP. Tables and timelines are enough.

---

## Security Requirements (MVP)

* Authentication required.
* Two roles: viewer, operator.
* Audit log for every write action:

  * actor, timestamp, action, payload hash, correlationId

---

## Observability Requirements

Dashboard must expose:

* last API fetch time
* backend error banners with request ids
* event correlation id displayed on rollout and vehicle pages

---

## Performance Requirements

* Initial fleet overview load under 3 seconds for 5k vehicles with pagination.
* Vehicle details load under 2 seconds.
* Rollout monitor auto-refresh every 10 seconds without freezing UI.

---

## Acceptance Criteria

* Operator can deploy a release to a selector with canary then wave.
* Operator can see which vehicles are stuck and why via reason codes.
* Operator can pause and rollback from UI.
* Operator can open any vehicle and see desired vs actual, plus last reconcile details.
* Audit trail exists for all deployments.

---

## Implementation Notes

* Frontend stack is free choice, but keep it boring: React or similar.
* Backend should expose read endpoints needed by UI, even if internal.
* Do not require MQTT access in browser for MVP.
* Reason codes must be treated as an enum and rendered consistently.

---

## Anti-Goals

* No “terminal” style remote commands
* No live ROS graph visualization in MVP
* No map view required
* No complex policy editor

---

## Final Instruction

Build the dashboard to operate the fleet like a system, not like a science fair. Operators should feel confident pressing deploy because the UI makes safety gates and evidence unavoidable.
