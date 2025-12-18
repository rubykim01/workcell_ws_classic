#!/bin/bash

# Script to spawn tools attached to UR quickchanger
# Usage: ./spawn_ur_attached_tools.sh

# Get the current workspace directory
OBJECTS_DIR="/root/workcell_ws_classic/src/descriptions/ur_description/urdf/objects"

echo "Objects directory: $OBJECTS_DIR"


# Tool positions (world coordinates) 
# Based on quickchanger_link home position
TOOL_X=0.359
TOOL_Y=0.282
TOOL_Z=1.715


# Quickchanger rotation in WORLD frame 
# Based on quickchanger_link home orientation
TOOL_ROLL=2.776
TOOL_PITCH=-0.147
TOOL_YAW=2.633


# Spawn Tooltip 01 
echo ""
echo "Spawning Tooltip 01 at ($TOOL_X, $TOOL_Y, $TOOL_Z) with rotation ($TOOL_ROLL, $TOOL_PITCH, $TOOL_YAW)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip_01.sdf" \
    -entity tooltip_01 \
    -x $TOOL_X -y $TOOL_Y -z $TOOL_Z \
    -R $TOOL_ROLL -P $TOOL_PITCH -Y $TOOL_YAW

# Spawn Tooltip 01_2 
echo ""
echo "Spawning Tooltip 01_2 at ($TOOL_X, $TOOL_Y, $TOOL_Z) with rotation ($TOOL_ROLL, $TOOL_PITCH, $TOOL_YAW)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip_01_2.sdf" \
    -entity tooltip_01_2 \
    -x $TOOL_X -y $TOOL_Y -z $TOOL_Z \
    -R $TOOL_ROLL -P $TOOL_PITCH -Y $TOOL_YAW

# Spawn KRVG 
echo ""
echo "Spawning KRVG at ($TOOL_X, $TOOL_Y, $TOOL_Z) with rotation ($TOOL_ROLL, $TOOL_PITCH, $TOOL_YAW)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/krvg.sdf" \
    -entity krvg \
    -x $TOOL_X -y $TOOL_Y -z $TOOL_Z \
    -R $TOOL_ROLL -P $TOOL_PITCH -Y $TOOL_YAW

echo ""
echo "Tools spawned successfully!"
