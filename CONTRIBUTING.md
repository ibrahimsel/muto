# Contributing to Eclipse Muto

## Development Environment Setup

### Prerequisites

- Python 3.10+ (Humble) / 3.12+ (Jazzy)
- ROS 2 Humble or Jazzy
- Git

### Virtual Environment

Create a venv at the workspace root for development tools (linting, type checking). This venv is **not** for ROS runtime dependencies.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install ruff mypy pre-commit
```

> The `.venv` directory is gitignored.

## Code Quality Tools

### Ruff (Linter + Formatter)

Each Python submodule (`src/agent`, `src/composer`, `src/core`) has its own `ruff.toml` with shared settings:

```toml
line-length = 100
target-version = "py310"

[lint]
select = ["E", "F", "W", "I", "UP", "B", "SIM"]
```

**Check for lint errors:**

```bash
ruff check src/agent/
ruff check src/composer/
ruff check src/core/
```

**Auto-fix safe issues:**

```bash
ruff check --fix src/agent/
```

**Format code:**

```bash
ruff format src/agent/
```

**Check formatting without modifying:**

```bash
ruff format --check src/agent/
```

### Mypy (Type Checking)

Each Python submodule has a `mypy.ini`. Currently configured with `ignore_missing_imports = True` as a starting point.

```bash
mypy src/agent/muto_agent/
mypy src/composer/muto_composer/
mypy src/core/muto_core/
```

### Ament Lint (Buildfarm)

The following ament lint tools run automatically during `colcon test` on the ROS buildfarm:

- **ament_copyright** — checks for copyright headers (SPDX EPL-2.0 format)
- **ament_flake8** — PEP 8 style checks
- **ament_pep257** — docstring convention checks

These are declared as `<test_depend>` in each `package.xml`.

## Copyright Headers

All Composiv.ai source files must use the SPDX copyright header:

```python
#
# Copyright (c) <YEAR> Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
#
```

Third-party files (Apache 2.0, MIT) retain their original headers.

## Running Tests

```bash
# Build all Muto packages
colcon build --packages-select muto_agent muto_composer muto_core muto_msgs eclipse_muto

# Run tests
colcon test --packages-select muto_agent muto_composer muto_core muto_msgs

# View test results
colcon test-result --verbose
```

## Commit Conventions

We use [Conventional Commits](https://www.conventionalcommits.org/):

```
feat: add new feature
fix: correct a bug
chore: maintenance tasks (deps, config, CI)
docs: documentation changes
refactor: code restructuring without behavior change
test: add or update tests
```

## Repository Structure

This is a multi-repo workspace using Git submodules:

| Submodule | Package | Description |
|-----------|---------|-------------|
| `src/agent` | `muto_agent` | Communication gateway (MQTT/Ditto/Symphony) |
| `src/composer` | `muto_composer` | Stack deployment and orchestration engine |
| `src/core` | `muto_core` | Digital twin bridge |
| `src/messages` | `muto_msgs` | ROS 2 message/service definitions |
| `src/eclipse_muto` | `eclipse_muto` | Metapackage |

## License

All contributions are under the [Eclipse Public License 2.0](LICENSE).
