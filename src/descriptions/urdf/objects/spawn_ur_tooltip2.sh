#!/bin/bash

# Script to spawn tools attached to UR quickchanger
# Usage: ./spawn_ur_tooltip2.sh

# Get the current workspace directory
OBJECTS_DIR="/root/workcell_ws_classic/src/descriptions/urdf/objects"

echo "Objects directory: $OBJECTS_DIR"

# Get current quickchanger_link pose from TF2
echo "Getting current quickchanger_link pose from TF2..."

# Capture tf2_echo output (timeout after 3 seconds to get at least one reading)
TF_OUTPUT=$(timeout 2 ros2 run tf2_ros tf2_echo world quickchanger_link 2>/dev/null | grep -A 10 "At time")

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

# Spawn Tooltip 02 
echo ""
echo "Spawning Tooltip 02 at ($TOOL_X, $TOOL_Y, $TOOL_Z) with rotation ($TOOL_ROLL, $TOOL_PITCH, $TOOL_YAW)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip_02.sdf" \
    -entity tooltip_02 \
    -x $TOOL_X -y $TOOL_Y -z $TOOL_Z \
    -R $TOOL_ROLL -P $TOOL_PITCH -Y $TOOL_YAW

# Attach Tooltip 02 to quickchanger
echo "Attaching tooltip_02 to quickchanger_link..."
ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink "{model1_name: 'ur', link1_name: 'quickchanger_link', model2_name: 'tooltip_02', link2_name: 'link'}"


# Spawn Tooltip 02_2 
echo ""
echo "Spawning Tooltip 02_2 at ($TOOL_X, $TOOL_Y, $TOOL_Z) with rotation ($TOOL_ROLL, $TOOL_PITCH, $TOOL_YAW)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip_02_2.sdf" \
    -entity tooltip_02_2 \
    -x $TOOL_X -y $TOOL_Y -z $TOOL_Z \
    -R $TOOL_ROLL -P $TOOL_PITCH -Y $TOOL_YAW

# Attach Tooltip 02_2 to quickchanger
echo "Attaching tooltip_02_2 to quickchanger_link..."
ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink "{model1_name: 'ur', link1_name: 'quickchanger_link', model2_name: 'tooltip_02_2', link2_name: 'link'}"

# Spawn KRVG 
echo ""
echo "Spawning KRVG at ($TOOL_X, $TOOL_Y, $TOOL_Z) with rotation ($TOOL_ROLL, $TOOL_PITCH, $TOOL_YAW)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/krvg.sdf" \
    -entity krvg \
    -x $TOOL_X -y $TOOL_Y -z $TOOL_Z \
    -R $TOOL_ROLL -P $TOOL_PITCH -Y $TOOL_YAW

# Attach KRVG (suction) to quickchanger
echo "Attaching krvg to quickchanger_link..."
ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink "{model1_name: 'ur', link1_name: 'quickchanger_link', model2_name: 'krvg', link2_name: 'link'}"

echo ""
echo "Tools spawned successfully!"

