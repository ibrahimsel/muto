#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SAMPLES_DIR="$ROOT_DIR/docs/samples"
TL_DIR="$SAMPLES_DIR/talker-listener"
SYMPHONY_DIR="$SAMPLES_DIR/symphony"
CONFIG_PATH="$ROOT_DIR/config/device_agent.json"
SOLUTION_JSON="$SYMPHONY_DIR/talker-listener.json"
INSTANCE_JSON="$SYMPHONY_DIR/instance.json"
AGENT_LOG="/tmp/muto-device-agent.log"
HTTP_LOG="/tmp/muto-artifact-server.log"
ARTIFACT_PORT="${ARTIFACT_PORT:-8000}"
STOP_SYMPHONY="${STOP_SYMPHONY:-0}"

HTTP_PID=""
AGENT_PID=""
SOLUTION_BAK=""

cleanup() {
  if [ -n "$HTTP_PID" ] && kill -0 "$HTTP_PID" >/dev/null 2>&1; then
    kill "$HTTP_PID" >/dev/null 2>&1 || true
  fi
  if [ -n "$AGENT_PID" ] && kill -0 "$AGENT_PID" >/dev/null 2>&1; then
    kill "$AGENT_PID" >/dev/null 2>&1 || true
  fi
  if [ -n "$SOLUTION_BAK" ] && [ -f "$SOLUTION_BAK" ]; then
    mv "$SOLUTION_BAK" "$SOLUTION_JSON"
  fi
  if [ "$STOP_SYMPHONY" = "1" ]; then
    (cd "$SYMPHONY_DIR" && $COMPOSE_CMD down) >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

require_cmd curl
require_cmd jq
require_cmd base64
require_cmd python3
require_cmd sha256sum
require_cmd muto_device_agent

if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
  COMPOSE_CMD="docker compose"
elif command -v docker-compose >/dev/null 2>&1; then
  COMPOSE_CMD="docker-compose"
else
  echo "Missing docker compose. Install docker compose or docker-compose." >&2
  exit 1
fi

echo "Starting Symphony services..."
(cd "$SYMPHONY_DIR" && $COMPOSE_CMD up -d)

echo "Waiting for Symphony API..."
for i in $(seq 1 60); do
  if curl -sS http://localhost:8082/v1alpha2/greetings >/dev/null 2>&1; then
    echo "Symphony API is up"
    break
  fi
  sleep 1
  if [ "$i" -eq 60 ]; then
    echo "Timed out waiting for Symphony API" >&2
    exit 1
  fi
done

RUN_SCRIPT="$TL_DIR/sample-stack/run.sh"
if [ ! -f "$RUN_SCRIPT" ]; then
  cat > "$RUN_SCRIPT" <<'EOF'
#!/usr/bin/env bash
set -e
echo "demo running"
sleep 300
EOF
  chmod +x "$RUN_SCRIPT"
fi

echo "Creating artifact..."
(cd "$TL_DIR" && ./create_archive.sh ./sample-stack .)
ARCHIVE_NAME="$(basename "$TL_DIR/sample-stack")-1.0.0.tar.gz"
ARCHIVE_PATH="$TL_DIR/$ARCHIVE_NAME"
CHECKSUM="$(sha256sum "$ARCHIVE_PATH" | awk '{print $1}')"
ARTIFACT_URI="http://localhost:${ARTIFACT_PORT}/${ARCHIVE_NAME}"

echo "Starting artifact server on port ${ARTIFACT_PORT}..."
(cd "$TL_DIR" && python3 -m http.server "$ARTIFACT_PORT" >"$HTTP_LOG" 2>&1) &
HTTP_PID=$!
sleep 1

echo "Updating Symphony solution payload..."
SOLUTION_BAK="${SOLUTION_JSON}.bak"
cp "$SOLUTION_JSON" "$SOLUTION_BAK"
jq --arg uri "$ARTIFACT_URI" \
   --arg checksum "sha256:${CHECKSUM}" \
   --arg version "1.0.0" \
   '.artifact_uri=$uri
    | .checksum=$checksum
    | .version=$version
    | .start_command="./run.sh"
    | .working_directory="."' \
   "$SOLUTION_JSON" > "${SOLUTION_JSON}.tmp"
mv "${SOLUTION_JSON}.tmp" "$SOLUTION_JSON"

echo "Starting device agent..."
export MUTO_CONFIG="$CONFIG_PATH"
muto_device_agent >"$AGENT_LOG" 2>&1 &
AGENT_PID=$!
sleep 2

echo "Deploying v1.0.0..."
(cd "$SYMPHONY_DIR" && ./define-solution.sh talker-listener.json)
(cd "$SYMPHONY_DIR" && ./define-instance.sh instance.json)
sleep 3

echo "Current state:"
cat /tmp/muto/stacks/demo-stack/state.json || true
ls -l /tmp/muto/stacks/demo-stack/current || true

echo "Triggering rollback with failing v1.0.1..."
jq --arg version "1.0.1" \
   '.version=$version
    | .start_command="./missing.sh"' \
   "$SOLUTION_JSON" > "${SOLUTION_JSON}.tmp"
mv "${SOLUTION_JSON}.tmp" "$SOLUTION_JSON"

(cd "$SYMPHONY_DIR" && ./define-solution.sh talker-listener.json)
sleep 5

echo "State after rollback attempt:"
cat /tmp/muto/stacks/demo-stack/state.json || true

echo "Demo complete."
echo "Agent log: $AGENT_LOG"
echo "Artifact server log: $HTTP_LOG"
echo "Set STOP_SYMPHONY=1 to auto-stop Symphony after the demo."
