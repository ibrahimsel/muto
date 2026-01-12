#!/bin/bash
#
# Muto End-to-End Demo Script
# This script orchestrates the full deployment pipeline demo
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
SAMPLES_DIR="$SCRIPT_DIR/../talker-listener"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_step() {
    echo -e "\n${BLUE}=== $1 ===${NC}\n"
}

print_success() {
    echo -e "${GREEN}$1${NC}"
}

print_warning() {
    echo -e "${YELLOW}$1${NC}"
}

print_error() {
    echo -e "${RED}$1${NC}"
}

check_dependencies() {
    print_step "Checking dependencies"

    local missing=()

    command -v docker &>/dev/null || missing+=("docker")
    command -v curl &>/dev/null || missing+=("curl")
    command -v jq &>/dev/null || missing+=("jq")
    command -v python3 &>/dev/null || missing+=("python3")

    if [ ${#missing[@]} -ne 0 ]; then
        print_error "Missing dependencies: ${missing[*]}"
        exit 1
    fi

    print_success "All dependencies found"
}

start_infrastructure() {
    print_step "Starting infrastructure (Symphony API, MQTT)"

    cd "$SCRIPT_DIR"
    docker compose up -d

    echo "Waiting for services to be ready..."
    sleep 5

    # Check Symphony API
    for i in {1..30}; do
        if curl -s http://localhost:8082/v1alpha2/targets/registry >/dev/null 2>&1; then
            print_success "Symphony API is ready"
            break
        fi
        if [ $i -eq 30 ]; then
            print_error "Symphony API failed to start"
            exit 1
        fi
        sleep 1
    done
}

start_http_server() {
    print_step "Starting local HTTP server for artifacts"

    # Check if server already running
    if curl -s http://localhost:8000/ >/dev/null 2>&1; then
        print_warning "HTTP server already running on port 8000"
        return
    fi

    cd "$SAMPLES_DIR"
    python3 -m http.server 8000 &
    HTTP_SERVER_PID=$!
    echo $HTTP_SERVER_PID > /tmp/muto-demo-http-server.pid

    sleep 2
    if curl -s http://localhost:8000/sample-stack-1.0.0.tar.gz -o /dev/null; then
        print_success "HTTP server started (PID: $HTTP_SERVER_PID)"
    else
        print_error "HTTP server failed to serve artifact"
        exit 1
    fi
}

get_auth_token() {
    print_step "Getting authentication token"

    TOKEN=$(curl -s -X POST \
        -H "Content-Type: application/json" \
        -d '{"username":"admin","password":""}' \
        "http://localhost:8082/v1alpha2/users/auth" | jq -r '.accessToken')

    if [ -z "$TOKEN" ] || [ "$TOKEN" = "null" ]; then
        print_error "Failed to get authentication token"
        exit 1
    fi

    print_success "Got auth token: ${TOKEN:0:20}..."
    export SYMPHONY_TOKEN="$TOKEN"
}

register_target() {
    print_step "Registering target device"

    cd "$SCRIPT_DIR"

    # Delete existing target if any (ignore errors)
    curl -s -X DELETE \
        -H "Authorization: Bearer $SYMPHONY_TOKEN" \
        "http://localhost:8082/v1alpha2/targets/registry/muto-device-001" >/dev/null 2>&1 || true

    RESPONSE=$(curl -s -w "\n%{http_code}" -X POST \
        -H "Content-Type: application/json" \
        -H "Authorization: Bearer $SYMPHONY_TOKEN" \
        -d @target.json \
        "http://localhost:8082/v1alpha2/targets/registry/muto-device-001")

    HTTP_STATUS=$(echo "$RESPONSE" | tail -n1)

    if [ "$HTTP_STATUS" -eq 200 ] || [ "$HTTP_STATUS" -eq 201 ]; then
        print_success "Target 'muto-device-001' registered"
    else
        print_error "Failed to register target (HTTP $HTTP_STATUS)"
        echo "$RESPONSE"
        exit 1
    fi
}

create_solution() {
    print_step "Creating solution (release)"

    cd "$SCRIPT_DIR"
    ./define-solution.sh talker-listener.json
}

show_agent_command() {
    print_step "Agent startup command"

    echo "Run this command in a separate terminal to start the agent:"
    echo ""
    echo -e "${YELLOW}cd $REPO_ROOT/src/agent${NC}"
    echo -e "${YELLOW}MUTO_CONFIG=$SCRIPT_DIR/demo-config.json python3 -m agent.muto_agent${NC}"
    echo ""
    echo "Or with virtual environment:"
    echo -e "${YELLOW}cd $REPO_ROOT/src/agent${NC}"
    echo -e "${YELLOW}source venv/bin/activate${NC}"
    echo -e "${YELLOW}MUTO_CONFIG=$SCRIPT_DIR/demo-config.json python3 -m agent.muto_agent${NC}"
}

trigger_deployment() {
    print_step "Triggering deployment"

    cd "$SCRIPT_DIR"

    # Delete existing instance if any
    curl -s -X DELETE \
        -H "Authorization: Bearer $SYMPHONY_TOKEN" \
        "http://localhost:8082/v1alpha2/instances/talker-listener-v-1-instance" >/dev/null 2>&1 || true

    # Create instance to trigger deployment
    INSTANCE_DATA=$(cat <<EOF
{
    "metadata": {
        "name": "talker-listener-v-1-instance"
    },
    "spec": {
        "solution": "talker-listener-v-1",
        "target": {
            "name": "muto-device-001"
        }
    }
}
EOF
)

    RESPONSE=$(curl -s -w "\n%{http_code}" -X POST \
        -H "Content-Type: application/json" \
        -H "Authorization: Bearer $SYMPHONY_TOKEN" \
        -d "$INSTANCE_DATA" \
        "http://localhost:8082/v1alpha2/instances/talker-listener-v-1-instance")

    HTTP_STATUS=$(echo "$RESPONSE" | tail -n1)

    if [ "$HTTP_STATUS" -eq 200 ] || [ "$HTTP_STATUS" -eq 201 ]; then
        print_success "Deployment triggered!"
    else
        print_warning "Instance creation returned HTTP $HTTP_STATUS (may already exist)"
    fi
}

show_verification() {
    print_step "Verification commands"

    echo "Check agent storage:"
    echo -e "${YELLOW}ls -la /tmp/muto-demo/stacks/${NC}"
    echo ""
    echo "Check deployment state:"
    echo -e "${YELLOW}cat /tmp/muto-demo/stacks/demo-stack/state.json | jq .${NC}"
    echo ""
    echo "Check running processes:"
    echo -e "${YELLOW}ps aux | grep demo-stack${NC}"
    echo ""
    echo "Dashboard URL:"
    echo -e "${YELLOW}http://localhost:3001${NC}"
    echo ""
    echo "Symphony Portal URL:"
    echo -e "${YELLOW}http://localhost:3000${NC}"
}

cleanup() {
    print_step "Cleanup"

    echo "To stop everything:"
    echo ""
    echo "1. Stop agent: Ctrl+C in agent terminal"
    echo ""
    echo "2. Stop HTTP server:"
    echo -e "${YELLOW}kill \$(cat /tmp/muto-demo-http-server.pid)${NC}"
    echo ""
    echo "3. Stop infrastructure:"
    echo -e "${YELLOW}cd $SCRIPT_DIR && docker compose down${NC}"
    echo ""
    echo "4. Clean demo data:"
    echo -e "${YELLOW}rm -rf /tmp/muto-demo${NC}"
}

# Main execution
case "${1:-}" in
    "start")
        check_dependencies
        start_infrastructure
        start_http_server
        get_auth_token
        register_target
        create_solution
        show_agent_command
        ;;
    "deploy")
        get_auth_token
        trigger_deployment
        ;;
    "verify")
        show_verification
        ;;
    "cleanup")
        cleanup
        ;;
    "full")
        check_dependencies
        start_infrastructure
        start_http_server
        get_auth_token
        register_target
        create_solution
        show_agent_command
        echo ""
        print_warning "Press Enter after starting the agent to trigger deployment..."
        read -r
        trigger_deployment
        show_verification
        ;;
    *)
        echo "Muto End-to-End Demo"
        echo ""
        echo "Usage: $0 <command>"
        echo ""
        echo "Commands:"
        echo "  start   - Start infrastructure, register target, create solution"
        echo "  deploy  - Trigger deployment (after agent is running)"
        echo "  verify  - Show verification commands"
        echo "  cleanup - Show cleanup commands"
        echo "  full    - Run complete demo (waits for agent before deploying)"
        echo ""
        ;;
esac
