# Eclipse Muto - Composer Reference

The **Muto Composer** provides helpers for building and validating Symphony-compatible stack models. In v1, it focuses on assembling release metadata into JSON payloads that the device agent understands.

## Responsibilities

- Validate release metadata fields
- Produce JSON payloads for Symphony `component.properties.data`
- Provide helper scripts for solution/instance templates

## Release Payload Format

```json
{
  "name": "towtruck-autonomy",
  "version": "1.0.1",
  "artifact_uri": "https://artifacts.example.com/towtruck-autonomy-1.0.1.tar.gz",
  "checksum": "sha256:<sha256>",
  "start_command": "./run.sh",
  "stop_command": "pkill -f run.sh",
  "working_directory": ".",
  "environment": {
    "ROS_DOMAIN_ID": "42"
  }
}
```

## Related Docs

- `docs/reference/agent/readme.md`
- `docs/samples/symphony/`
