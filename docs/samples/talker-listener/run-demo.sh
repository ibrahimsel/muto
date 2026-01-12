#!/usr/bin/env bash
# End-to-end demo wrapper for the talker-listener sample.
# Assumes Symphony is already running.
# Usage: ./run-demo.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYMPHONY_DIR="$SCRIPT_DIR/../symphony"
SOLUTION_JSON="$SYMPHONY_DIR/talker-listener.json"
INSTANCE_JSON="$SYMPHONY_DIR/instance.json"

for cmd in curl jq base64; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Required command not found: $cmd" >&2
    exit 1
  fi
done

API_URL="http://localhost:8082/v1alpha2/greetings"
echo "Waiting for Symphony API at $API_URL..."
for i in $(seq 1 60); do
  if curl -sS "$API_URL" >/dev/null 2>&1; then
    echo "Symphony API is up"
    break
  fi
  printf "."
  sleep 1
  if [ "$i" -eq 60 ]; then
    printf '\nTimed out waiting for Symphony API\n' >&2
    exit 2
  fi
done

echo "Defining solution using $SOLUTION_JSON"
"$SYMPHONY_DIR/define-solution.sh" "$SOLUTION_JSON"
sleep 1

echo "Creating instance using $INSTANCE_JSON"
"$SYMPHONY_DIR/define-instance.sh" "$INSTANCE_JSON"

echo "Demo finished. Check Symphony instances:"
echo "  http://localhost:8082/v1alpha2/instances"
