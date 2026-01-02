# Rollouts

This document explains the different statuses and components related to software rollouts.

## Rollout Status

| Status      | Description                                                              |
|-------------|--------------------------------------------------------------------------|
| `active`    | The rollout is currently in progress.                                    |
| `completed` | The rollout has finished for all targeted vehicles.                      |

## Canary Status

The canary phase is the first stage of a rollout, where the update is deployed to a small subset of devices to test for issues.

| Status      | Description                                                              |
|-------------|--------------------------------------------------------------------------|
| `converged` | The canary group has successfully applied the new revision.              |
| `failed`    | The canary group has encountered failures during the update.             |

## Wave Status

After a successful canary phase, the rollout proceeds in waves to the rest of the fleet.

| Status      | Description                                                              |
|-------------|--------------------------------------------------------------------------|
| `active`    | The wave is currently being rolled out.                                  |
| `completed` | The wave has been successfully rolled out to all targeted devices.       |

## Rollout Gates

Rollout gates are automated checks that must pass for a rollout to proceed from one stage to the next (e.g., from canary to a wider rollout).

| Gate                 | Description                                                                                       |
|----------------------|---------------------------------------------------------------------------------------------------|
| `canaryHealthy`      | `true` if the canary group has converged without significant errors.                               |
| `failureThreshold`   | `true` if the percentage of failed devices is below the configured threshold.                       |
| `safetyChecks`       | `true` if all automated safety checks (e.g., vehicle is stationary) are passing for the devices. |
