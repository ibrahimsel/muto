# Composer Technical Requirements

## Purpose

Provide automated orchestration of ROS 2 stack deployments for vehicle fleets.
Accept stack deployment requests, analyze stack definitions, and execute compose,
  provision, and launch pipelines via plugin services.

## Functional Requirements

FR-1. Subscribe to MutoAction messages on a configurable topic, parse the JSON
payload, and publish stack request events for downstream subsystems.
Sources: src/composer/composer/subsystems/message_handler.py,
src/composer/composer/events.py, src/messages/msg/MutoAction.msg.

FR-2. Analyze stack payloads to classify type and determine execution
requirements (provision, launch, merge).
Sources: src/composer/composer/subsystems/stack_manager.py,
src/composer/composer/subsystems/orchestration_manager.py.

FR-3. Execute pipelines defined in YAML, honoring step conditions and
compensation steps, and invoke ROS 2 service-based plugins.
Sources: src/composer/composer/subsystems/pipeline_engine.py,
src/composer/composer/workflow/pipeline.py,
src/composer/composer/workflow/schemas/pipeline_schema.py,
src/composer/config/pipeline.yaml.

FR-4. Provide compose, provision, and launch plugin services that parse stack
manifests, prepare workspaces, and start/kill/apply stacks.
Sources: src/composer/composer/plugins/compose_plugin.py,
src/composer/composer/plugins/provision_plugin.py,
src/composer/composer/plugins/launch_plugin.py.

FR-5. Support stack parsing for direct stack JSON, archive stacks, and legacy or
solution-manifest formats, including expression resolution and stack merging.
Sources: src/composer/composer/utils/stack_parser.py,
src/composer/composer/stack_handlers/json_handler.py,
src/composer/composer/stack_handlers/archive_handler.py,
src/composer/composer/stack_handlers/ditto_handler.py,
src/composer/composer/model/stack.py.

FR-6. Integrate with CoreTwin to fetch stack definitions for stackId-based
payloads and twin synchronization flows.
Sources: src/composer/composer/subsystems/digital_twin_integration.py,
src/composer/composer/plugins/base_plugin.py, src/messages/srv/CoreTwin.srv.

## Interfaces

Topics:
Subscribe: stack_topic for muto_msgs/MutoAction.
Publish: stack_state (JSON payload in std_msgs/String).
Optional/legacy: raw_stack input and composed_stack output for compose
  plugin flows.
Sources: src/composer/composer/subsystems/message_handler.py,
src/composer/composer/plugins/compose_plugin.py, src/composer/config/composer.yaml.

Services:
muto_compose (ComposePlugin)
muto_provision (ProvisionPlugin)
muto_start_stack (LaunchPlugin)
muto_kill_stack (LaunchPlugin)
muto_apply_stack (LaunchPlugin)
Service contracts use muto_msgs/PlanManifest for input/output.
Sources: src/messages/srv/ComposePlugin.srv, src/messages/srv/ProvisionPlugin.srv,
src/messages/srv/LaunchPlugin.srv, src/messages/msg/PlanManifest.msg.

CoreTwin service:
core_twin/get_stack_definition and core_twin/set_current_stack used for
  stack lookup and state updates.
Sources: src/messages/srv/CoreTwin.srv,
src/composer/composer/subsystems/message_handler.py,
src/composer/composer/subsystems/digital_twin_integration.py,
src/composer/composer/plugins/base_plugin.py.

## Pipeline Requirements

PR-1. Pipeline config must validate against the JSON schema and load from
config/pipeline.yaml in the package share directory.
Sources: src/composer/composer/workflow/schemas/pipeline_schema.py,
src/composer/composer/subsystems/pipeline_engine.py.

PR-2. Each pipeline includes a sequence of steps with name, service, and
plugin, plus optional condition expressions evaluated by SafeEvaluator.
Sources: src/composer/composer/workflow/pipeline.py,
src/composer/composer/workflow/safe_evaluator.py, src/composer/config/pipeline.yaml.

PR-3. Compensation steps must be executed on failure to best-effort recover.
Sources: src/composer/composer/workflow/pipeline.py, src/composer/config/pipeline.yaml.

## Configuration Requirements

CR-1. Composer parameters are provided via ROS 2 parameters, typically from
config/composer.yaml.
Sources: src/composer/config/composer.yaml, src/composer/composer/muto_composer.py.

CR-2. Required parameters include:
stack_topic
twin_topic
thing_messages_topic
ignored_packages
namespace, name, prefix, attributes, anonymous
Sources: src/composer/config/composer.yaml.

CR-3. ignored_packages is used by provisioning logic to exclude packages from
colcon builds.
Sources: src/composer/composer/stack_handlers/archive_handler.py,
src/composer/composer/stack_handlers/registry.py.

## Stack Definition Requirements

SR-1. Direct stack JSON payloads must include metadata.content_type and a
stack body such as launch with node/composable definitions for stack/json.
Sources: src/composer/composer/utils/stack_parser.py,
src/composer/composer/stack_handlers/json_handler.py.

SR-2. Archive stacks (stack/archive) must provide either:
launch.data (base64), or
launch.url (file/http/https)
and must include launch.properties.launch_file.
Sources: src/composer/composer/stack_handlers/archive_handler.py.

SR-3. Legacy/ditto stacks may use node, composable, launch,
launch_description_source, or on_start/on_kill patterns.
Sources: src/composer/composer/stack_handlers/ditto_handler.py.

SR-4. Stack references with stackId or value.stackId must be resolvable via
CoreTwin to a JSON manifest string.
Sources: src/composer/composer/plugins/base_plugin.py, src/messages/srv/CoreTwin.srv.

SR-5. Stack expressions must be resolved for supported macros:
$(find pkg), $(env VAR), $(optenv VAR), and $(arg name).
Sources: src/composer/composer/model/stack.py,
src/composer/composer/subsystems/stack_manager.py.

SR-6. Parameters may be defined as literal values, file-based values (from),
or command-based values (command).
Sources: src/composer/composer/model/param.py.

## Plugin Requirements

PL-1. Compose plugin must parse and route stack requests via the stack handler
registry and return a PlanManifest response.
Sources: src/composer/composer/plugins/compose_plugin.py,
src/composer/composer/plugins/base_plugin.py,
src/composer/composer/stack_handlers/registry.py.

PL-2. Provision plugin must prepare workspaces under
/tmp/muto/muto_workspaces/<stack_name>, install dependencies with rosdep, and
build with colcon.
Sources: src/composer/composer/plugins/provision_plugin.py,
src/composer/composer/stack_handlers/archive_handler.py.

PL-3. Launch plugin must start/kill/apply stacks, source workspaces via
StackManifest source or install/setup.bash, and manage subprocess lifecycle.
Sources: src/composer/composer/plugins/launch_plugin.py.

PL-4. Stack handlers must implement double-dispatch and support json/archive
and legacy formats.
Sources: src/composer/composer/stack_handlers/registry.py,
src/composer/composer/plugins/base_plugin.py.

## Runtime and Environment Requirements

RR-1. ROS 2 (Foxy or later) with rclpy, launch, launch_ros,
ament_index_python, and lifecycle message support.
Sources: src/composer/README.md, src/composer/composer/workflow/launcher.py,
src/composer/composer/model/node.py.

RR-2. Python 3.8+ with jsonschema, yaml, requests, and docker available.
Sources: src/composer/README.md, src/composer/setup.py,
src/composer/composer/subsystems/pipeline_engine.py,
src/composer/composer/stack_handlers/archive_handler.py.

RR-3. System tools: ros2, colcon, rosdep, git, bash, and OS support
for /tmp and process signaling.
Sources: src/composer/README.md,
src/composer/composer/stack_handlers/archive_handler.py,
src/composer/composer/plugins/launch_plugin.py.

## Operational and Non-Functional Requirements

NF-1. Pipeline execution is sequential with conditional gating; on failure,
compensation steps must run and remaining steps must be skipped.
Sources: src/composer/composer/workflow/pipeline.py, src/composer/config/pipeline.yaml.

NF-2. Event bus publishing must remain non-blocking for ROS callbacks and log
handler failures.
Sources: src/composer/composer/events.py.

NF-3. Artifact downloads must respect timeouts and optionally verify checksums.
Sources: src/composer/composer/stack_handlers/archive_handler.py.

NF-4. Stack state publication must encode JSON into std_msgs/String with
timestamp and state type.
Sources: src/composer/composer/subsystems/message_handler.py.

## Assumptions and Constraints

A-1. Pipeline action names must match MutoAction method values
(e.g., start, kill, apply).
Sources: src/messages/msg/MutoAction.msg, src/composer/config/pipeline.yaml.

A-2. CoreTwin service response payloads are assumed to deliver JSON manifest
strings in output.
Sources: src/messages/srv/CoreTwin.srv,
src/composer/composer/subsystems/digital_twin_integration.py,
src/composer/composer/plugins/base_plugin.py.

A-3. Workspace and artifact state are stored under
/tmp/muto/muto_workspaces and .muto_artifact.json.
Sources: src/composer/composer/plugins/base_plugin.py,
src/composer/composer/stack_handlers/archive_handler.py
