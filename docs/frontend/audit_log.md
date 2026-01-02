# Audit Log Actions

This document describes the actions that can be recorded in the audit log.

| Action              | Description                                                                                               |
|---------------------|-----------------------------------------------------------------------------------------------------------|
| `create_deployment` | A user has initiated a new software deployment (rollout). The details include the rollout ID, stack version, and target selector. |
| `rollback`          | A user has initiated a rollback of a deployment. The details may include the reason for the rollback.    |
