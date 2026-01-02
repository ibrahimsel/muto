# Vehicle Timeline Events

This document describes the events that can appear in a vehicle's timeline.

| Event                | Description                                                                                             |
|----------------------|---------------------------------------------------------------------------------------------------------|
| `desired_notified`   | The vehicle has been notified of a new desired software revision. The event details contain the revision. |
| `apply_start`        | The vehicle has started the process of applying the new software revision.                              |
| `ready`              | The vehicle has successfully applied the new revision and is considered `converged`.                      |
| `failed`             | The update process has failed. The event details may contain a reason for the failure.                  |
| `blocked`            | The update process is blocked. The event details may contain a reason for the blockage.                 |
