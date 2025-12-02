#!/bin/bash

# Script to spawn tooltip1 and tooltip2 objects
# Usage: ./spawn_tooltips.sh

# Fixed positions for each tooltip (adjust as needed)
TOOLTIP1_X=0.5
TOOLTIP1_Y=0.3
TOOLTIP1_Z=0.9

TOOLTIP2_X=0.5
TOOLTIP2_Y=-0.3
TOOLTIP2_Z=0.9

echo "Using fixed positions for each tooltip:"
echo "  tooltip1: ($TOOLTIP1_X, $TOOLTIP1_Y, $TOOLTIP1_Z)"
echo "  tooltip2: ($TOOLTIP2_X, $TOOLTIP2_Y, $TOOLTIP2_Z)"

echo "Spawning tooltip objects"

# Get the current workspace directory
OBJECTS_DIR="/root/workcell_ws_classic/src/descriptions/ur_description/urdf/objects"

echo "Objects directory: $OBJECTS_DIR"

# Check if SDF files exist
if [ ! -f "$OBJECTS_DIR/tooltip1.sdf" ]; then
    echo "Error: tooltip1.sdf not found at $OBJECTS_DIR/tooltip1.sdf"
    exit 1
fi

if [ ! -f "$OBJECTS_DIR/tooltip2.sdf" ]; then
    echo "Error: tooltip2.sdf not found at $OBJECTS_DIR/tooltip2.sdf"
    exit 1
fi

# Spawn tooltips
echo "Spawning tooltips..."
echo "  - tooltip1 at ($TOOLTIP1_X, $TOOLTIP1_Y, $TOOLTIP1_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip1.sdf" \
    -entity tooltip1 \
    -x $TOOLTIP1_X -y $TOOLTIP1_Y -z $TOOLTIP1_Z

echo "  - tooltip2 at ($TOOLTIP2_X, $TOOLTIP2_Y, $TOOLTIP2_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip2.sdf" \
    -entity tooltip2 \
    -x $TOOLTIP2_X -y $TOOLTIP2_Y -z $TOOLTIP2_Z

echo "All tooltip objects spawned successfully!"
echo "Objects created:"
echo "  - tooltip1"
echo "  - tooltip2"

