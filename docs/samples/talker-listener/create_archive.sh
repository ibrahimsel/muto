#!/bin/bash

# Create a release archive and a matching JSON payload.
# Usage: ./create_archive.sh <input_directory> <output_directory>

set -euo pipefail

if [ $# -ne 2 ]; then
    echo "Usage: $0 <input_directory> <output_directory>"
    exit 1
fi

INPUT_DIR="$1"
OUTPUT_DIR="$2"

if [ ! -d "$INPUT_DIR" ]; then
    echo "Error: $INPUT_DIR is not a directory"
    exit 1
fi

if [ ! -d "$OUTPUT_DIR" ]; then
    echo "Error: $OUTPUT_DIR is not a directory"
    exit 1
fi

ROOT_NAME=$(basename "$INPUT_DIR")
VERSION="1.0.0"
ARCHIVE_NAME="${ROOT_NAME}-${VERSION}.tar.gz"
ARCHIVE_PATH="${OUTPUT_DIR}/${ARCHIVE_NAME}"

tar -czf "$ARCHIVE_PATH" -C "$INPUT_DIR" .
CHECKSUM=$(sha256sum "$ARCHIVE_PATH" | awk '{print $1}')

JSON_FILE="${OUTPUT_DIR}/${ROOT_NAME}-release.json"
cat << EOF > "$JSON_FILE"
{
  "name": "${ROOT_NAME}",
  "version": "${VERSION}",
  "artifact_uri": "https://artifacts.example.com/${ARCHIVE_NAME}",
  "checksum": "sha256:${CHECKSUM}",
  "start_command": "./run.sh",
  "working_directory": "."
}
EOF

echo "Created archive: $ARCHIVE_PATH"
echo "Created release payload: $JSON_FILE"
