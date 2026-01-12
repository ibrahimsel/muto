# Eclipse Muto

<img src="./docs/images/muto.png" alt="Eclipse Muto Modules" title="Eclipse Muto Modules" width="30%" style="display: block; margin: auto; width: 30%;">

## Overview

**Eclipse Muto** is an open-source deployment orchestrator that lets you bulk deploy, switch, and rollback **versioned ROS2 workspace artifacts** across edge devices **without SSH**. Muto treats user software as an opaque archive: you version it, compress it, and provide start/stop commands. Muto handles distribution, atomic activation, rollback, and observability.

## What Makes Muto Unique

Muto replaces manual SSH + file copying + restarts with **versioned artifacts**, **atomic activation**, and **automatic rollback**. It uses **Eclipse Symphony** as its control plane and a lightweight device agent for install/execute on each target.

## Key Features

- **Opaque artifacts**: Deploy `.tar.gz` (or similar) ROS2 workspaces without refactoring
- **Atomic installation**: Crash-safe installs with `current`/`previous` symlinks
- **Automatic rollback**: Revert on start failures or early exits
- **Bulk deployment**: One command updates fleets, devices reconcile independently
- **Observability**: Report running version, state, failure reason, and timestamps
- **No SSH required**: All orchestration flows through Symphony + MQTT

## Repository Structure

```
├── README.md                     # This file
├── docs/                      # Documentation
│   ├── project_overview.md      # Architecture and product definition
│   ├── user_guide/               # User documentation and guides
│   ├── developer_guide/          # Developer documentation
│   └── reference/                # Technical reference documentation
├── src/                          # Source code
│   ├── agent/                    # Device agent (install/activate/rollback)
│   ├── composer/                 # JSON model composition helpers
│   ├── core/                     # Legacy ROS services (not used in v1)
│   └── messages/                 # Legacy ROS message definitions
├── config/                       # Muto Configuration example files
└── launch/                       # Muto ROS launch  example files
```

All folder under the src/ folder are [GIT modules](https://git-scm.com/docs/gitmodules) that refer to other repositories.

## Quick Links


### 🚀 Getting Started
- **[🐳 Quick Start](./docs/user_guide/quick_start.md)** - Fast deployment with pre-built images (recommended)
- **[🔧 Source Build Quick Start](./docs/developer_guide/building_from_source.md)** - Build from source for maximum flexibility, including setup for  Containerized development environment

### 📚 Documentation
- **[📖 User Guide](./docs/user_guide/readme.md)** - User documentation on how to configure and run Muto
- **[🔧 Developer Guide](./docs/developer_guide/readme.md)** - Development and contribution guide
- **[📚 Reference Documentation](./docs/reference/readme.md)** - Technical reference for all components

### 📝 Examples
- **[🎯 Examples Overview](./docs/examples/readme.md)** - Muto examples and demonstration tutorials

## Target Use Cases

- **Autonomous Vehicle Fleets**: Bulk updates and rollback safety across fleets
- **Industrial Robotics**: Remote updates without SSH or manual intervention
- **Service Robotics**: Versioned release delivery with reliable rollback
- **R&D**: Fast iteration on ROS2 workspaces with consistent deployment semantics

## License

Eclipse Muto is licensed under the [Eclipse Public License 2.0](LICENSE).

## Contributing

We welcome contributions to Eclipse Muto! Please see our [Developer Guide](./docs/developer_guide/readme.md) for detailed information on how to contribute, including:

### Code Quality Workflow

- Install the tooling once per clone: `pip install pre-commit`.
- Enable the hooks so both `pre-commit` and `pre-push` run automatically: `pre-commit install --hook-type pre-commit --hook-type pre-push`.
- Formatting is enforced by `ruff format`, linting is handled by `ruff`, and strict type checks run via `mypy` using the rules in `pyproject.toml`. The hooks will auto-fix what they can on commit and block pushes if issues remain.
- Before sending larger changes, you can validate everything locally with `pre-commit run --all-files` to avoid CI noise.

## Community

- **GitHub Repository**: [https://github.com/eclipse-muto/muto](https://github.com/eclipse-muto/muto)
- **Eclipse Foundation Project Page**: [https://projects.eclipse.org/projects/automotive.muto](https://projects.eclipse.org/projects/automotive.muto)
- **Issue Tracker**: [GitHub Issues](https://github.com/eclipse-muto/muto/issues)
