## Context

Eclipse Muto needs to be released to packages.ros.org via the bloom release process. Five packages in scope: **muto_agent**, **muto_composer**, **muto_core**, **muto_msgs**, and the **eclipse_muto** metapackage. Dashboard, blueprint, and docs repos are excluded.

The current state has significant gaps: version mismatches between setup.py and package.xml, no CHANGELOGs, no git tags, no pre-commit hooks, no unified linting, incorrect/missing dependencies, and generic package names that may collide on packages.ros.org.

**Target distros:** Humble (Ubuntu 22.04, Python 3.10) + Jazzy (Ubuntu 24.04, Python 3.12)

**Decision log:**
- Package names: Rename to `muto_agent`, `muto_composer`, `muto_core`, keep `muto_msgs` (generic names would be rejected on packages.ros.org)
- symphony-sdk-python: Kept separate as pip package, not bundled into agent
- Linting: ruff for dev/pre-commit + ament_flake8/pep257 for buildfarm compliance
- Both distros require code to work on Python 3.10 AND 3.12 (3.11 supported implicitly via `>=3.10`)
- Metapackage: `eclipse_muto` pulls in all sub-packages (`apt install ros-humble-eclipse-muto`)
- License: EPL-2.0 (mandated by Eclipse Foundation, accepted on packages.ros.org — same as Cyclone DDS)

---

## Overview
Since the intent is releasing Muto to `packages.ros.org`, some improvements to the codebase is needed

The current state has significant gaps: version mismatches between `setup.py` and `package.xml`, no `CHANGELOG`s, no git tags, no pre-commit hooks, no unified linting, and generic package names that may collide on packages.ros.org. More info on this below. Also currently a lot of unused code exists in the codebase. Perhaps we stash them for now.

## 1. Package Rename to `muto_` Prefix

The names `agent`, `composer`, `core` are too generic and would collide on packages.ros.org. Renaming proactively to `muto_agent`, `muto_composer`, `muto_core` (keeping `muto_msgs` as-is).

**Rename checklist per package:**
- [x] Rename Python module directory (e.g. `agent/` → `muto_agent/`)
- [x] Update all internal imports
- [x] Update all `entry_points` in `setup.py`
- [x] Update `setup.py` `name=`
- [x] Update `package.xml` `<name>`
- [x] Update all cross-package `<depend>` references in other packages' `package.xml`
- [x] Rename `resource/<pkg_name>` marker file
- [x] Update `data_files` references in `setup.py`
- [x] Update launch files and config references

## 2. Fix Version Alignment & Package Metadata
- [x] Unify version between `setup.py` and `package.xml` (currently mismatched in agent: 2.0.0 vs 1.0.0, composer: 0.0.0 vs 0.2.0, messages: N/A vs 0.0.0)
- [x] Decide on a starting version for the release (recommend `0.1.0` for all packages -- fresh start since nothing has been released) (Decided. 0.42.0 as the initial release)
- [ ] Upgrade `package.xml` to format 3 (`<package format="3">`)
- [ ] Add `<license file="LICENSE">EPL-2.0</license>` with `file` attribute
- [ ] Add `<url type="repository">`, `<url type="bugtracker">` tags
- [ ] Ensure `<buildtool_depend>ament_python</buildtool_depend>` is declared (agent, composer, core)
- [ ] Ensure `<buildtool_depend>ament_cmake</buildtool_depend>` for muto_msgs
- [ ] Make `setup.py` descriptions more verbose and accurate
- [ ] Fix `setup.cfg` to use underscores (`script_dir`, `install_scripts`) instead of dashes (suppresses setuptools deprecation warnings)
- [ ] Verify `resource/<package_name>` marker file exists (required for ament index)
- [ ] Verify `data_files` in setup.py includes both resource index entry and package.xml
- [ ] Set `python_requires='>=3.10'` (minimum for Humble, covers 3.10/3.11/3.12)
- [ ] Add PyPI classifiers for Python 3.10, 3.11, 3.12 in every `setup.py`
- [ ] Add missing `maintainer` / `maintainer_email` where inconsistent

**Files to modify per submodule:**
- `setup.py` -- version, metadata, description, python_requires, data_files, install_requires
- `setup.cfg` -- fix dash-separated keys to underscores
- `package.xml` -- version, format, license, urls, dependencies
- `resource/<pkg_name>` -- verify exists and the name is correct after we rename the submodules

## 3.  Python 3.10+ Compatibility

- [ ] Fix all `list[str]`, `dict[str, Any]`, `X | None` syntax to use `from __future__ import annotations` at the top of each file (works on 3.10+, zero runtime cost)
- [ ] Verify no use of `distutils` (removed in 3.12)
- [ ] Verify no use of `imp` module (removed in 3.12)
- [ ] Verify `typing.Self`, `type` statement, or other 3.11+/3.12+ features are not used
- [ ] Add CI matrix testing for Python 3.10 and 3.12

**Scope:** All `.py` files in `src/agent/agent/`, `src/composer/composer/`, `src/core/core/`

## 4. Linting & Formatting Setup

**Per submodule:**

- [ ] Add `ruff.toml` (or `[tool.ruff]` section) with shared config:
  ```toml
  line-length = 100
  target-version = "py310"
  [lint]
  select = ["E", "F", "W", "I", "UP", "B", "SIM"]
  ```
- [ ] Add `mypy.ini` or `[tool.mypy]` config (start with `--ignore-missing-imports`, gradually tighten)
- [ ] Add `ament_flake8`, `ament_pep257`, `ament_copyright` as `<test_depend>` in every package.xml
- [ ] Add copyright headers to all source files (ament_copyright will check this on buildfarm)
- [ ] Add ruff and mypy as dev dependencies (document install via `pip install ruff mypy` or uv)
- [ ] Run `ruff check --fix .` and `ruff format .` across all submodules to establish baseline

## 5. Pre-commit Hooks & Conventional Commits

**Per submodule (or shared config at top level):**

- [ ] Add `.pre-commit-config.yaml`:
  ```yaml
  repos:
    - repo: https://github.com/astral-sh/ruff-pre-commit
      rev: v0.9.x  # pin to latest
      hooks:
        - id: ruff
          args: [--fix]
        - id: ruff-format
    - repo: https://github.com/pre-commit/pre-commit-hooks
      rev: v5.0.0
      hooks:
        - id: trailing-whitespace
        - id: end-of-file-fixer
        - id: check-yaml
        - id: check-xml
    - repo: https://github.com/compilerla/conventional-pre-commit
      rev: v4.0.0
      hooks:
        - id: conventional-pre-commit
          stages: [commit-msg]
  ```
- [ ] Document pre-commit setup in each README: `pip install pre-commit && pre-commit install --hook-type commit-msg --hook-type pre-commit`

##  6. CHANGELOG Generation

- [ ] Install `catkin_pkg`: `pip install catkin-pkg`
- [ ] For each submodule, generate initial CHANGELOG.rst: `catkin_generate_changelog`
- [ ] Edit each CHANGELOG.rst to be human-readable (clean up git history noise)
- [ ] Commit CHANGELOGs

**Note:** CHANGELOG must be `CHANGELOG.rst` (reStructuredText), not Markdown. This is required by REP 132 and bloom.

## 7. CI Updates

**Per submodule `.github/workflows/`:**

- [ ] Add/update colcon build + test workflow:
  - Matrix: `ros_distro: [humble, jazzy]` with matching Ubuntu (`jammy` / `noble`)
  - Use `ros-tooling/setup-ros-action` and `ros-tooling/action-ros-ci`
  - Steps: checkout with submodules -> setup ROS -> colcon build -> colcon test
- [ ] Add linting job:
  - `ruff check .`
  - `ruff format --check .`
  - `mypy src/` (can be non-blocking initially with `continue-on-error: true`)
- [ ] Add ament lint tests (these run via `colcon test`):
  - ament_copyright, ament_flake8, ament_pep257 (declared as test_depend)
- [ ] Remove/update outdated `setuptools==58.2.0` pin in CI
- [ ] Add test coverage reporting (pytest-cov, minimum threshold)

**Top-level workflow (`/.github/workflows/`):**
- [ ] Update to match per-submodule CI structure
- [ ] Add matrix for ROS distros

## 8: Git & Repository Hygiene

- [ ] **Pin submodules to specific commits** (not tracking `branch = main`)
- [ ] **Tag initial release version** in each submodule after all prep work: `git tag 0.42.0`
- [ ] **Verify clean `git status`** in each submodule (catkin_prepare_release requires this)
- [ ] Add/update `.gitignore` in each submodule


## 9. Bump Versions & Release

- [ ] Run `catkin_generate_changelog` in each submodule
- [ ] Edit CHANGELOGs for clarity
- [ ] Run `catkin_prepare_release` (bumps version, commits, tags)
- [ ] Push commits and tags to official `eclipse-muto` repos
- [ ] Run `bloom-release --new-track --rosdistro humble my_package` for each package
- [ ] Run `bloom-release --new-track --rosdistro jazzy my_package` for each package
- [ ] Monitor rosdistro PR reviews and address feedback

**Release order matters** (due to inter-package dependencies):
1. `muto_msgs` (no Muto dependencies)
2. `muto_core` (depends on muto_msgs)
3. `muto_composer` (depends on muto_msgs)
4. `muto_agent` (depends on muto_msgs)
5. `eclipse_muto` (metapackage — depends on all of the above, released last)

### Core Specific
- [ ] Add README.md (currently missing)
### Messages Specific

- [ ] Add README.md (currently missing)
- [ ] Document each .msg and .srv file's purpose

---

### Agent & Composer Specific

- [ ] Remove `jsonschema` dependency -- replace with lightweight validation (manual dict checks or dataclass validation). Not worth the size inflation for schema validation alone
- [ ] Standardize stack models and document which stack format versions map to which Muto versions
- [ ] Lock stack model schema (document in code + README what fields are required/optional per content_type)

### Core Specific

- [ ] Support environment variables for sensitive configuration (twin URL, credentials)
- [ ] Add fallback chain: env var -> YAML config -> default
- [ ] Add README.md (currently missing)

### Messages Specific

- [ ] Add README.md (currently missing)
- [ ] Document each .msg and .srv file's purpose

---


## Verification

After all changes, verify:
1. `colcon build --packages-select eclipse_muto muto_agent muto_composer muto_core muto_msgs` succeeds
2. `colcon test --packages-select eclipse_muto muto_agent muto_composer muto_core muto_msgs` passes (including ament lint tests)
3. `bloom-release --dry-run` (if available) or manual validation of package.xml
4. `catkin_prepare_release --dry-run` succeeds
5. Build in Docker containers for both Humble and Jazzy base images
6. `pip check` clean in installed environments
7. `ros2 pkg list | grep muto_` finds all packages after install
