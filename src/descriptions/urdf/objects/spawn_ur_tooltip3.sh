#!/bin/bash

# Script to spawn tools attached to UR quickchanger
# Usage: ./spawn_ur_tooltip3.sh

# Get the current workspace directory dynamically
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OBJECTS_DIR="$SCRIPT_DIR"

# Gripper mounted on the quickchanger: "krvg" (default) or "koras_2f100".
# Passed as $1 by the spawn GUI; defaults to krvg for standalone use.
GRIPPER="${1:-krvg}"

echo "Objects directory: $OBJECTS_DIR"
echo "Quickchanger gripper: $GRIPPER"

# Get current quickchanger_link pose from TF2
echo "Getting current quickchanger_link pose from TF2..."

# Capture tf2_echo output (timeout after 1.5 seconds to get at least one reading)
TF_OUTPUT=$(timeout 1.5 ros2 run tf2_ros tf2_echo world quickchanger_link 2>/dev/null | grep -A 10 "At time")

if [ -z "$TF_OUTPUT" ]; then
    echo "ERROR: Could not get TF transform. Make sure the robot is running."
    exit 1
fi

# Parse Translation [x, y, z]
TRANSLATION_LINE=$(echo "$TF_OUTPUT" | grep "Translation:" | head -1)
TOOL_X=$(echo "$TRANSLATION_LINE" | sed 's/.*\[//' | sed 's/\].*//' | cut -d',' -f1 | tr -d ' ')
TOOL_Y=$(echo "$TRANSLATION_LINE" | sed 's/.*\[//' | sed 's/\].*//' | cut -d',' -f2 | tr -d ' ')
TOOL_Z=$(echo "$TRANSLATION_LINE" | sed 's/.*\[//' | sed 's/\].*//' | cut -d',' -f3 | tr -d ' ')

# Parse RPY (radian) [roll, pitch, yaw] from TF
RPY_LINE=$(echo "$TF_OUTPUT" | grep "RPY (radian)" | head -1)
TF_ROLL=$(echo "$RPY_LINE" | sed 's/.*\[//' | sed 's/\].*//' | cut -d',' -f1 | tr -d ' ')
TF_PITCH=$(echo "$RPY_LINE" | sed 's/.*\[//' | sed 's/\].*//' | cut -d',' -f2 | tr -d ' ')
TF_YAW=$(echo "$RPY_LINE" | sed 's/.*\[//' | sed 's/\].*//' | cut -d',' -f3 | tr -d ' ')

echo "Current quickchanger_link pose:"
echo "  Translation: X=$TOOL_X, Y=$TOOL_Y, Z=$TOOL_Z"
echo "  TF Rotation (RPY): R=$TF_ROLL, P=$TF_PITCH, Y=$TF_YAW"

# Apply transformation: -roll, -pitch, yaw+π
TOOL_ROLL=$(awk "BEGIN {print -($TF_ROLL)}")
TOOL_PITCH=$(awk "BEGIN {print -($TF_PITCH)}")
TOOL_YAW=$(awk "BEGIN {print $TF_YAW + 3.141592}")

echo "  Adjusted (RPY): R=$TOOL_ROLL, P=$TOOL_PITCH, Y=$TOOL_YAW"

# Spawn all three tools (tooltip_03, tooltip_03_2, $GRIPPER) in one batch process,
# then attach each to the quickchanger.
echo ""
echo "Spawning tooltip_03, tooltip_03_2, $GRIPPER at ($TOOL_X, $TOOL_Y, $TOOL_Z) rot ($TOOL_ROLL, $TOOL_PITCH, $TOOL_YAW)"
python3 "$OBJECTS_DIR/batch_spawn.py" \
    --obj "$OBJECTS_DIR/tooltip_03.sdf"   tooltip_03   $TOOL_X $TOOL_Y $TOOL_Z $TOOL_ROLL $TOOL_PITCH $TOOL_YAW \
    --obj "$OBJECTS_DIR/tooltip_03_2.sdf" tooltip_03_2 $TOOL_X $TOOL_Y $TOOL_Z $TOOL_ROLL $TOOL_PITCH $TOOL_YAW \
    --obj "$OBJECTS_DIR/$GRIPPER.sdf"     $GRIPPER     $TOOL_X $TOOL_Y $TOOL_Z $TOOL_ROLL $TOOL_PITCH $TOOL_YAW

# Attach all three tools to quickchanger from one batch_attach.py process
echo "Attaching tooltip_03, tooltip_03_2, $GRIPPER to quickchanger_link..."
python3 "$OBJECTS_DIR/batch_attach.py" \
    --attach ur quickchanger_link tooltip_03   link \
    --attach ur quickchanger_link tooltip_03_2 link \
    --attach ur quickchanger_link $GRIPPER     link

echo ""
echo "Tools spawned successfully!"

