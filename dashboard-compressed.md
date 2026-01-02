# Eclipse Muto Dashboard

## Mission

Build an **operational dashboard** for Eclipse Muto that lets operators **declare intent** and **observe evidence** for ROS 2 fleet orchestration.

The dashboard must answer fast:

* What should be running vs what is running?
* Who is stuck, why, and since when?
* Is the rollout safe to continue?
* What changed, by whom, and what happened after?

No command execution. No SSH metaphors. Desired state only.

---

## MVP Scope

### Included

* Fleet overview with filtering
* Vehicle details view
* Releases list
* Desired state creation
* Rollout monitoring (canary + one wave)
* Pause rollout and rollback
* Reason-code driven troubleshooting
* Audit trail of deployments
* Two roles: viewer, operator

### Excluded

* Multi-tenant orgs
* Complex RBAC
* Advanced analytics
* Maps, 3D twins
* Custom dashboards

---

## User Roles

* **Viewer:** read-only fleet visibility
* **Operator:** create desired state, start rollout, pause, rollback

---

## Core Screens

### Fleet Overview

Must show:

* Total vehicles
* Converged, applying, blocked, failed, offline counts
* Filter by tags (model, region, ring)
* Search by vehicleId
* Vehicles table:

  * vehicleId
  * tags
  * desired stack version
  * actual stack version
  * status
  * lastSeenAt
  * top reason code
* Top reason codes by count
* Active rollouts list

Performance: handle 5k vehicles with pagination.

---

### Vehicle Details

Must show:

* Desired vs actual stack and revision
* Current status (pill)
* Reason codes if blocked or failed
* Safety snapshot: moving, autonomy mode, battery, window
* Timeline:

  * desired notified
  * apply start
  * readiness or failure
  * rollback if occurred
* Raw JSON viewer:

  * desired state
  * latest reconcile report

---

### Releases

* List stacks and versions
* Show digest, compatibility summary
* Show signature present or missing
* Action: deploy release

---

### Create Deployment (Desired State)

Operator must:

* Select stack release
* Target via selector or vehicle list
* Set safety policy:

  * apply window
  * require stationary
  * deny autonomous mode
  * min battery
* Choose rollout:

  * canary per group = 1
  * wave = 100 percent (MVP)
* Add comment

UI must warn if safety gates are weakened.

---

### Rollout Monitor

Must show:

* Rollout id, stack version, selector
* Canary status per group
* Wave progress
* Converge percent
* Failed and blocked counts
* Gate status
  Actions:
* Pause rollout
* Rollback rollout or wave

---

## Data Requirements

Dashboard consumes:

* Vehicles inventory (tags, lastSeenAt)
* Desired state revisions
* Stack releases
* Reconcile reports (status, reason codes)
* Health reports (heartbeat)

UI is **reason-code first**, not log-text first.

---

## Status Rules

* Offline: lastSeenAt older than threshold
* Converged: actual matches desired revision
* Applying: last status applying
* Blocked: last status blocked
* Failed: last status failed
* Rolled back: last status rolled_back
* Pending: desired newer than last report

---

## Required APIs

Read:

* `GET /vehicles`
* `GET /vehicles/{id}`
* `GET /desired-states/{revision}`
* `GET /desired-states/latest?vehicleId=`
* `GET /releases`
* `GET /rollouts`
* `GET /reports/reconcile`
* `GET /reports/health`

Write:

* `POST /desired-states`
* `POST /rollouts`
* `POST /rollouts/{id}/pause`
* `POST /rollouts/{id}/rollback`

Dashboard does not talk to MQTT directly.

---

## UX Rules

* Every non-converged vehicle shows a reason code immediately
* Dangerous actions require confirmation
* Raw JSON must be copyable
* No charts required for MVP

---

## Security

* Authentication required
* Roles: viewer, operator
* Audit log for all write actions:

  * actor, timestamp, action, correlationId

---

## Acceptance Criteria

* Operator deploys a release via selector
* Canary then wave rollout visible
* Blocked vehicles clearly explain why
* Rollout can be paused and rolled back
* Audit trail exists for every deployment

---

## Anti-Goals

* No remote shell
* No live ROS graph UI
* No map view
* No policy DSL editor

---

## Final Instruction

Design the dashboard to make **unsafe actions uncomfortable** and **evidence unavoidable**. If an operator cannot explain why a vehicle is stuck by looking at the UI, the dashboard has failed.

---
