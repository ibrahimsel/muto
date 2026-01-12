# Talker-Listener sample (Muto + Symphony)

This sample shows how to package a ROS2 workspace as an artifact and deploy it with Eclipse Symphony.

## Steps

1) Create an archive:

```bash
tar -czf talker-listener-1.0.0.tar.gz ./sample-stack
sha256sum talker-listener-1.0.0.tar.gz
```

2) Upload to your HTTP(S) server.

3) Update `docs/samples/symphony/talker-listener.json` with:

- `artifact_uri`
- `checksum`
- `start_command`

4) Use Symphony helper scripts:

```bash
cd docs/samples/symphony
./define-solution.sh talker-listener.json
./define-instance.sh instance.json
```

5) Verify logs on the device:

```
/var/lib/muto/stacks/demo-stack/logs/
```

For full setup, see `docs/user_guide/quick_start.md`.
