#!/bin/bash
#
# Check Muto Stack Status
# Reads deployment state from ~/.muto/state/*/state.json
#

set -e

MUTO_ROOT="${MUTO_ROOT:-$HOME/.muto}"
STATE_DIR="${MUTO_ROOT}/state"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo ""
    echo "======================================"
    echo "      Muto Stack Status Report"
    echo "======================================"
    echo ""
}

print_stack_status() {
    local stack_name=$1
    local state_file="${STATE_DIR}/${stack_name}/state.json"

    if [ ! -f "${state_file}" ]; then
        echo -e "${YELLOW}Stack: ${stack_name}${NC}"
        echo "  Status: No deployment state found"
        echo ""
        return
    fi

    # Parse JSON using jq if available, otherwise use python
    if command -v jq &> /dev/null; then
        local status=$(jq -r '.status // "unknown"' "${state_file}")
        local current_version=$(jq -r '.current_version // "N/A"' "${state_file}")
        local previous_version=$(jq -r '.previous_version // "N/A"' "${state_file}")
        local deployed_at=$(jq -r '.deployed_at // "N/A"' "${state_file}")
        local last_updated=$(jq -r '.last_updated // "N/A"' "${state_file}")
        local error_message=$(jq -r '.error_message // ""' "${state_file}")
        local rollback_count=$(jq -r '.rollback_count // 0' "${state_file}")
    else
        local status=$(python3 -c "import json; d=json.load(open('${state_file}')); print(d.get('status', 'unknown'))")
        local current_version=$(python3 -c "import json; d=json.load(open('${state_file}')); print(d.get('current_version', 'N/A'))")
        local previous_version=$(python3 -c "import json; d=json.load(open('${state_file}')); print(d.get('previous_version', 'N/A'))")
        local deployed_at=$(python3 -c "import json; d=json.load(open('${state_file}')); print(d.get('deployed_at', 'N/A'))")
        local last_updated=$(python3 -c "import json; d=json.load(open('${state_file}')); print(d.get('last_updated', 'N/A'))")
        local error_message=$(python3 -c "import json; d=json.load(open('${state_file}')); print(d.get('error_message', ''))")
        local rollback_count=$(python3 -c "import json; d=json.load(open('${state_file}')); print(d.get('rollback_count', 0))")
    fi

    # Color-code status
    local status_color="${NC}"
    case "${status}" in
        "running")
            status_color="${GREEN}"
            ;;
        "failed"|"error")
            status_color="${RED}"
            ;;
        "deploying"|"pending")
            status_color="${YELLOW}"
            ;;
        "rolled_back")
            status_color="${BLUE}"
            ;;
    esac

    echo -e "${BLUE}Stack: ${stack_name}${NC}"
    echo -e "  Status:           ${status_color}${status}${NC}"
    echo "  Current Version:  ${current_version}"
    echo "  Previous Version: ${previous_version}"
    echo "  Deployed At:      ${deployed_at}"
    echo "  Last Updated:     ${last_updated}"

    if [ "${rollback_count}" != "0" ]; then
        echo -e "  Rollback Count:   ${YELLOW}${rollback_count}${NC}"
    fi

    if [ -n "${error_message}" ] && [ "${error_message}" != "" ]; then
        echo -e "  Error:            ${RED}${error_message}${NC}"
    fi

    echo ""
}

# Main
print_header

# Check if state directory exists
if [ ! -d "${STATE_DIR}" ]; then
    echo "State directory not found: ${STATE_DIR}"
    echo "No deployments have been recorded yet."
    echo ""
    echo "The state directory will be created when the first stack is deployed."
    exit 0
fi

# Check for specific stack or list all
if [ -n "$1" ]; then
    # Check specific stack
    print_stack_status "$1"
else
    # List all stacks
    stack_count=0
    for stack_dir in "${STATE_DIR}"/*/; do
        if [ -d "${stack_dir}" ]; then
            stack_name=$(basename "${stack_dir}")
            print_stack_status "${stack_name}"
            ((stack_count++))
        fi
    done

    if [ ${stack_count} -eq 0 ]; then
        echo "No stacks found in ${STATE_DIR}"
        echo "Deploy a stack to see its status here."
    else
        echo "--------------------------------------"
        echo "Total stacks: ${stack_count}"
    fi
fi

echo ""
echo "State directory: ${STATE_DIR}"
