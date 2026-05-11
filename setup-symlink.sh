#!/usr/bin/env bash
# Run once from /config/esphome/ after cloning the repo there.
# Creates a symlink so ESPHome can find grinder.yaml in the expected root.
set -euo pipefail

REPO="smart-grind-by-weight-esphome"
TARGET="grinder.yaml"

cd "$(dirname "$0")/.."

if [ -e "$TARGET" ] && [ ! -L "$TARGET" ]; then
  echo "ERROR: $TARGET already exists and is not a symlink. Remove it first." >&2
  exit 1
fi

ln -sf "${REPO}/grinder.yaml" "$TARGET"
echo "Symlink created: /config/esphome/${TARGET} -> ${REPO}/grinder.yaml"
