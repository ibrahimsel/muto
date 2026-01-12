# Eclipse Muto - Developer Guide

This guide focuses on the v1 deployment agent and Symphony integration.

## Development Setup

- Python 3.10+
- Eclipse Symphony API + MQTT broker
- Optional: local HTTP server for artifacts

## Key Packages

- `src/agent` - device agent runtime
- `src/composer` - release payload helpers

Legacy ROS modules (`src/core`, `src/messages`) remain in the repo but are not used by the v1 agent.
