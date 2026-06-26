#!/usr/bin/env bash
# Fetch UR5e + Robotiq 2F-85 from DeepMind MuJoCo Menagerie.
#
# We vendor only the two model directories we use to keep the repo lean,
# and the Apache-2.0 LICENSE files are preserved alongside them.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$ROOT/assets/menagerie"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "Cloning Menagerie (shallow)..."
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git "$TMP/menagerie"

mkdir -p "$DEST"
for model in universal_robots_ur5e robotiq_2f85; do
  echo "  → $model"
  rm -rf "$DEST/$model"
  cp -r "$TMP/menagerie/$model" "$DEST/$model"
done

echo "Done. Vendored models at: $DEST"
