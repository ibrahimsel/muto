cd docs/samples/symphony

# Start infrastructure and set up
./run-demo.sh start

# In a separate terminal, start the agent
cd src/agent
source venv/bin/activate
MUTO_CONFIG=/home/sel/workspaces/mutoV2/docs/samples/symphony/demo-config.json python3 -m agent.device_agent

# Back in the first terminal, trigger deployment
./run-demo.sh deploy

# Verify
./run-demo.sh verify
