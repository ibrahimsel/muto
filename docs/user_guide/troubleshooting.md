# Eclipse Muto - Troubleshooting

## Device Agent Won't Start

- Verify config path: `echo $MUTO_CONFIG`
- Validate JSON syntax in your config file
- Check log output for missing fields

## Artifact Download Fails

- Ensure `artifact_uri` is reachable over HTTP(S)
- Confirm checksum matches:
  ```bash
  sha256sum your-archive.tar.gz
  ```
- Increase `downloads.retries` or `downloads.timeout_seconds`

## Startup Fails and Rolls Back

- Review log file in:
  ```
  /var/lib/muto/stacks/<stack_name>/logs/<version>.log
  ```
- Check that `start_command` is executable
- Ensure `working_directory` exists inside the archive

## Symphony Integration Issues

- Verify MQTT broker host/port
- Confirm Symphony `topic_prefix`, `request_topic`, `response_topic`
- Check that the target name matches Symphony instance target

## State Looks Inconsistent

- Stop the agent
- Remove `.tmp` directories under:
  ```
  /var/lib/muto/stacks/<stack_name>/releases/
  ```
- Restart the agent (it will cleanup and reconcile)
