#!/bin/bash

# Script to spawn one mobile tray loaded with relay parts on the feeder base (kiro).
# Usage: ./spawn_feeder_kiro.sh
# A single tray is spawned at the top feeder slot with five relay parts on it.

echo "Kiro relay tray spawn"

# Get the current workspace directory dynamically
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OBJECTS_DIR="$SCRIPT_DIR"

# Locate trolley_positions.yaml (same relative path in source tree and install share)
YAML_PATH="$SCRIPT_DIR/../../config/trolley_positions.yaml"
if [ ! -f "$YAML_PATH" ]; then
    echo "Error: trolley_positions.yaml not found at $YAML_PATH"
    exit 1
fi

# Read feeder pose from YAML so object spawn positions follow the configured feeder
read FEEDER_WORLD_X FEEDER_WORLD_Y FEEDER_WORLD_Z FEEDER_YAW <<< "$(python3 -c "
import yaml
with open('$YAML_PATH') as f:
    d = yaml.safe_load(f)['trolley_positions']['feeder']
print(d['x'], d['y'], d['z'], d['yaw'])
")"
echo "Feeder world pose: x=$FEEDER_WORLD_X y=$FEEDER_WORLD_Y z=$FEEDER_WORLD_Z yaw=$FEEDER_YAW"

# Convert a feeder-local offset (lx ly lz) to world coords, applying feeder yaw rotation.
# Roll/pitch are assumed 0 (they are in trolley_positions.yaml); only yaw affects XY.
world_xyz() {
    awk -v fx="$FEEDER_WORLD_X" -v fy="$FEEDER_WORLD_Y" -v fz="$FEEDER_WORLD_Z" -v yaw="$FEEDER_YAW" \
        -v lx="$1" -v ly="$2" -v lz="$3" \
        'BEGIN {
            c = cos(yaw); s = sin(yaw);
            printf "%.6f %.6f %.6f\n", fx + c*lx - s*ly, fy + s*lx + c*ly, fz + lz;
        }'
}

# Sum two scalars (for combining feeder-local offsets before rotation)
sum() {
    awk -v a="$1" -v b="$2" 'BEGIN { printf "%.6f\n", a + b }'
}

# Combine a tray-local offset with the tray's own feeder-local offset, then rotate into world.
world_from_tray() {
    # $1..$3 = tray feeder-local offset, $4..$6 = component offset from tray origin
    world_xyz "$(sum $1 $4)" "$(sum $2 $5)" "$(sum $3 $6)"
}

# Feeder-local tray offset (meters). Top feeder slot, same XY as the elec/heater trays.
OFFSET_X=-0.361
OFFSET_Y=-0.9685
OFFSET_R1_Z=0.495

# Tray world position
read TRAY_R1_X TRAY_R1_Y TRAY_R1_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $OFFSET_R1_Z)"

echo "Objects directory: $OBJECTS_DIR"

# Check if SDF files exist
if [ ! -f "$OBJECTS_DIR/mobile_tray_r1.sdf" ]; then
    echo "Error: mobile_tray_r1.sdf not found at $OBJECTS_DIR/mobile_tray_r1.sdf"
    exit 1
fi

# Relay part offsets from tray origin (meters, converted from mm)
RELAY01_OFFSET_X=-0.01625
RELAY01_OFFSET_Y=0.0215
RELAY01_OFFSET_Z=0.0439

RELAY02_OFFSET_X=-0.1065
RELAY02_OFFSET_Y=0.0145
RELAY02_OFFSET_Z=0.0521

RELAY02_2_OFFSET_X=-0.1065
RELAY02_2_OFFSET_Y=-0.0655
RELAY02_2_OFFSET_Z=0.0531

RELAY03_OFFSET_X=-0.01625
RELAY03_OFFSET_Y=-0.05925
RELAY03_OFFSET_Z=0.030

RELAY04_OFFSET_X=0.089
RELAY04_OFFSET_Y=-0.066
RELAY04_OFFSET_Z=0.026

RELAY04_2_OFFSET_X=0.089
RELAY04_2_OFFSET_Y=0.004
RELAY04_2_OFFSET_Z=0.026

# Relay part world positions
read RELAY01_X RELAY01_Y RELAY01_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_R1_Z $RELAY01_OFFSET_X $RELAY01_OFFSET_Y $RELAY01_OFFSET_Z)"
read RELAY02_X RELAY02_Y RELAY02_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_R1_Z $RELAY02_OFFSET_X $RELAY02_OFFSET_Y $RELAY02_OFFSET_Z)"
read RELAY02_2_X RELAY02_2_Y RELAY02_2_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_R1_Z $RELAY02_2_OFFSET_X $RELAY02_2_OFFSET_Y $RELAY02_2_OFFSET_Z)"
read RELAY03_X RELAY03_Y RELAY03_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_R1_Z $RELAY03_OFFSET_X $RELAY03_OFFSET_Y $RELAY03_OFFSET_Z)"
read RELAY04_X RELAY04_Y RELAY04_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_R1_Z $RELAY04_OFFSET_X $RELAY04_OFFSET_Y $RELAY04_OFFSET_Z)"
read RELAY04_2_X RELAY04_2_Y RELAY04_2_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_R1_Z $RELAY04_2_OFFSET_X $RELAY04_2_OFFSET_Y $RELAY04_2_OFFSET_Z)"

# Collect every object into one batch_spawn.py call (single process).
# Args per object: FILE ENTITY X Y Z R P Y. Parts inherit feeder yaw.
SPAWN_ARGS=()
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_r1.sdf" mobile_tray1 $TRAY_R1_X $TRAY_R1_Y $TRAY_R1_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_01.sdf" relay_part_01 $RELAY01_X $RELAY01_Y $RELAY01_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_02.sdf" relay_part_02 $RELAY02_X $RELAY02_Y $RELAY02_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_02_2.sdf" relay_part_02_2 $RELAY02_2_X $RELAY02_2_Y $RELAY02_2_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_03.sdf" relay_part_03 $RELAY03_X $RELAY03_Y $RELAY03_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_04.sdf" relay_part_04 $RELAY04_X $RELAY04_Y $RELAY04_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_04.sdf" relay_part_04_2 $RELAY04_2_X $RELAY04_2_Y $RELAY04_2_Z 0 0 $FEEDER_YAW)

# Spawn all objects from a single process
echo "Spawning all kiro objects (single batch)..."
python3 "$OBJECTS_DIR/batch_spawn.py" "${SPAWN_ARGS[@]}"

echo "Tray and relay parts spawned successfully!"

# Objects have gravity disabled, so they don't fall after spawn; a brief pause
# just lets the new models register before the link_attacher references them.
sleep 0.3

# Queue all attachments, then run them from one batch_attach.py process.
ATTACH_ARGS=()
attach_to_tray() {  # $1 = object, $2 = tray (both attach via their 'link')
    ATTACH_ARGS+=(--attach "$2" link "$1" link)
}

attach_to_tray relay_part_01 mobile_tray1
attach_to_tray relay_part_02 mobile_tray1
attach_to_tray relay_part_02_2 mobile_tray1
attach_to_tray relay_part_03 mobile_tray1
attach_to_tray relay_part_04 mobile_tray1
attach_to_tray relay_part_04_2 mobile_tray1

echo "Attaching all kiro objects to tray (single batch)..."
python3 "$OBJECTS_DIR/batch_attach.py" "${ATTACH_ARGS[@]}"
echo "All kiro objects attached to tray."
