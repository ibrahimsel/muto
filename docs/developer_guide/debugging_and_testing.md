# Eclipse Muto - Debugging and Testing

## Unit Tests

```bash
pytest src/agent/test/test_deployment_manager.py
```

## Debugging Tips

- Check `state.json` under `/var/lib/muto/stacks/<stack_name>/`
- Inspect runtime logs under `/var/lib/muto/stacks/<stack_name>/logs/`
- Verify Symphony MQTT connectivity
