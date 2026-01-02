# Vehicle Status Codes

This document explains the different status codes and reason codes for vehicles in the Muto fleet.

## Vehicle Status

| Status      | Description                                                                 |
|-------------|-----------------------------------------------------------------------------|
| `converged` | The vehicle has successfully applied the desired software revision.         |
| `applying`  | The vehicle is currently in the process of applying a new software revision.|
| `pending`   | The vehicle has been notified of a new revision but has not started applying it yet. |
| `blocked`   | The vehicle is prevented from applying the desired revision due to a safety or health check failure. |
| `failed`    | The vehicle attempted to apply the desired revision but failed.             |
| `offline`   | The vehicle is not currently connected to the network.                      |

## Reason Codes

Reason codes provide more specific information about why a vehicle is in a particular state, especially for `blocked`, `failed`, and `offline` statuses.

| Reason Code                   | Associated Status | Description                                                              |
|-------------------------------|-------------------|--------------------------------------------------------------------------|
| `SAFETY_WINDOW_VIOLATION`     | `blocked`         | The vehicle is moving, but the update requires it to be stationary.      |
| `REQUIRE_STATIONARY`          | `blocked`         | The vehicle must be stationary to apply the update.                      |
| `BATTERY_LOW`                 | `blocked`         | The vehicle's battery is too low to safely perform the update.           |
| `CONTAINER_IMAGE_PULL_FAILED` | `failed`          | The vehicle was unable to download the required container image for the update. |
| `NETWORK_UNREACHABLE`         | `failed`          | The vehicle lost network connectivity during the update process.         |
| `HEARTBEAT_TIMEOUT`           | `offline`         | The vehicle has not sent a heartbeat signal within the expected time frame. |
