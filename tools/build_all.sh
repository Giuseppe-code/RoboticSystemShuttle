#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BLENDER_BIN="${BLENDER_BIN:-/Applications/Blender.app/Contents/MacOS/Blender}"

cd "$ROOT"
python3 tools/fetch_osm_data.py \
  --config config/parco_gandhi_catania.json \
  --output-dir data/parco_gandhi_catania

"$BLENDER_BIN" --background --python-exit-code 1 --python tools/build_blender_scene.py -- \
  --input data/parco_gandhi_catania/scene_data.json \
  --blend outputs/parco_gandhi_catania.blend \
  --glb outputs/parco_gandhi_catania.glb

cp outputs/parco_gandhi_catania.glb godot/assets/parco_gandhi_catania.glb
cp data/parco_gandhi_catania/road_network.json godot/data/parco_gandhi_catania_road_network.json
echo "Ready: outputs/parco_gandhi_catania.blend and godot/main.tscn"
