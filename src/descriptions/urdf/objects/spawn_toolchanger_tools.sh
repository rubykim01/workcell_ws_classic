#!/bin/bash

# Script to spawn tools on the tool changer base assembly stand
# Usage: ./spawn_toolchanger_tools.sh

# Get the current workspace directory dynamically
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OBJECTS_DIR="$SCRIPT_DIR"

# Locate trolley_positions.yaml
YAML_PATH="$SCRIPT_DIR/../../config/trolley_positions.yaml"
if [ ! -f "$YAML_PATH" ]; then
    echo "Error: trolley_positions.yaml not found at $YAML_PATH"
    exit 1
fi

# Read toolchanger pose from YAML
read CART_X CART_Y CART_Z TC_YAW <<< "$(python3 -c "
import yaml
with open('$YAML_PATH') as f:
    d = yaml.safe_load(f)['trolley_positions']['toolchanger']
print(d['x'], d['y'], d['z'], d['yaw'])
")"
echo "Toolchanger world pose: x=$CART_X y=$CART_Y z=$CART_Z yaw=$TC_YAW"

# Convert a toolchanger-local offset (lx ly lz) to world coords, applying yaw rotation.
world_xyz() {
    awk -v fx="$CART_X" -v fy="$CART_Y" -v fz="$CART_Z" -v yaw="$TC_YAW" \
        -v lx="$1" -v ly="$2" -v lz="$3" \
        'BEGIN {
            c = cos(yaw); s = sin(yaw);
            printf "%.6f %.6f %.6f\n", fx + c*lx - s*ly, fy + s*lx + c*ly, fz + lz;
        }'
}

# Compose toolchanger yaw with an object's local yaw to get world yaw
world_yaw() {
    awk -v fy="$TC_YAW" -v ly="$1" 'BEGIN { printf "%.6f\n", fy + ly }'
}

# Read tool changer stand offset and visual yaw from xacro
XACRO_PATH="$SCRIPT_DIR/../toolchanger.xacro"
if [ ! -f "$XACRO_PATH" ]; then
    echo "Error: toolchanger.xacro not found at $XACRO_PATH"
    exit 1
fi

read TC_X TC_Y TC_Z TC_STAND_YAW <<< "$(python3 -c "
import xml.etree.ElementTree as ET
tree = ET.parse('$XACRO_PATH')
ns = {'xacro': 'http://wiki.ros.org/xacro'}
root = tree.getroot()
# Joint origin: toolchanger stand offset from trolley
for joint in root.iter('joint'):
    if joint.get('name') == 'toolchanger_tool_changer_joint':
        xyz = joint.find('origin').get('xyz').split()
        break
# Visual origin: stand local yaw
for link in root.iter('link'):
    if link.get('name') == 'tool_changer_link':
        rpy = link.find('visual/origin').get('rpy').split()
        break
print(xyz[0], xyz[1], xyz[2], rpy[2])
")"
echo "Tool changer stand offset: x=$TC_X y=$TC_Y z=$TC_Z stand_yaw=$TC_STAND_YAW"

# Tool offsets from Tool Changer stand (in stand-local frame)
# KRVG
KRVG_X=-0.1208
KRVG_Y=0.311
KRVG_Z=0.4994

# Koras_2F100
KORAS_X=-0.1208
KORAS_Y=0.1615
KORAS_Z=0.4994

#Tooltip_01
TT01_X=-0.1085
TT01_Y=-0.0265
TT01_Z=0.5532
# Tooltip_01-2
TT01_2_Y=0.0335

# Tooltip_02
TT02_X=-0.1085
TT02_Y=-0.176
TT02_Z=0.5532
# Tooltip_02-2
TT02_2_Y=-0.116

# Tooltip_03
TT03_X=-0.1085
TT03_Y=-0.3255
TT03_Z=0.5532

TT03_2_Y=-0.2655

# Sum two scalars
sum() {
    awk -v a="$1" -v b="$2" 'BEGIN { printf "%.6f\n", a + b }'
}

# Rotate a tool-local offset by the stand yaw, then add stand offset, giving trolley-local coords
stand_to_trolley() {
    awk -v sx="$TC_X" -v sy="$TC_Y" -v sz="$TC_Z" -v yaw="$TC_STAND_YAW" \
        -v lx="$1" -v ly="$2" -v lz="$3" \
        'BEGIN {
            c = cos(yaw); s = sin(yaw);
            printf "%.6f %.6f %.6f\n", sx + c*lx - s*ly, sy + s*lx + c*ly, sz + lz;
        }'
}

# Compute world positions for each tool (stand-local → trolley-local → world)
read KRVG_WX KRVG_WY KRVG_WZ <<< "$(world_xyz $(stand_to_trolley $KRVG_X $KRVG_Y $KRVG_Z))"
read KORAS_WX KORAS_WY KORAS_WZ <<< "$(world_xyz $(stand_to_trolley $KORAS_X $KORAS_Y $KORAS_Z))"
read TT01_WX TT01_WY TT01_WZ <<< "$(world_xyz $(stand_to_trolley $TT01_X $TT01_Y $TT01_Z))"
read TT01_2_WX TT01_2_WY TT01_2_WZ <<< "$(world_xyz $(stand_to_trolley $TT01_X $TT01_2_Y $TT01_Z))"
read TT02_WX TT02_WY TT02_WZ <<< "$(world_xyz $(stand_to_trolley $TT02_X $TT02_Y $TT02_Z))"
read TT02_2_WX TT02_2_WY TT02_2_WZ <<< "$(world_xyz $(stand_to_trolley $TT02_X $TT02_2_Y $TT02_Z))"
read TT03_WX TT03_WY TT03_WZ <<< "$(world_xyz $(stand_to_trolley $TT03_X $TT03_Y $TT03_Z))"
read TT03_2_WX TT03_2_WY TT03_2_WZ <<< "$(world_xyz $(stand_to_trolley $TT03_X $TT03_2_Y $TT03_Z))"

# World yaws for each tool type (stand yaw + trolley yaw + tool local yaw)
STAND_WORLD_YAW=$(world_yaw $TC_STAND_YAW)
KRVG_WYAW=$(awk -v sy="$STAND_WORLD_YAW" -v ly="-1.570796" 'BEGIN { printf "%.6f\n", sy + ly }')
KORAS_WYAW=$(awk -v sy="$STAND_WORLD_YAW" -v ly="-1.570796" 'BEGIN { printf "%.6f\n", sy + ly }')
TT_WYAW=$(awk -v sy="$STAND_WORLD_YAW" -v ly="0" 'BEGIN { printf "%.6f\n", sy + ly }')

echo "Spawning tools on Tool Changer Stand..."

# KRVG
echo "Spawning KRVG..."
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/krvg.sdf" \
    -entity krvg \
    -x $KRVG_WX -y $KRVG_WY -z $KRVG_WZ \
    -R 3.141592 -P 0.785398 -Y $KRVG_WYAW

# Koras_2F100
echo "Spawning Koras_2F100..."
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/koras_2f100.sdf" \
    -entity koras_2f100 \
    -x $KORAS_WX -y $KORAS_WY -z $KORAS_WZ \
    -R 3.141592 -P 0.785398 -Y $KORAS_WYAW

# Tooltip_01
echo "Spawning Tooltip_01..."
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip_01.sdf" \
    -entity tooltip_01 \
    -x $TT01_WX -y $TT01_WY -z $TT01_WZ \
    -R 0 -P -2.356194 -Y $TT_WYAW

# Tooltip_01-2
echo "Spawning Tooltip_01_2..."
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip_01_2.sdf" \
    -entity tooltip_01_2 \
    -x $TT01_2_WX -y $TT01_2_WY -z $TT01_2_WZ \
    -R 0 -P -2.356194 -Y $TT_WYAW

# Tooltip_02
echo "Spawning Tooltip_02..."
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip_02.sdf" \
    -entity tooltip_02 \
    -x $TT02_WX -y $TT02_WY -z $TT02_WZ \
    -R 0 -P -2.356194 -Y $TT_WYAW

# Tooltip_02-2
echo "Spawning Tooltip_02_2..."
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip_02_2.sdf" \
    -entity tooltip_02_2 \
    -x $TT02_2_WX -y $TT02_2_WY -z $TT02_2_WZ \
    -R 0 -P -2.356194 -Y $TT_WYAW

# Tooltip_03
echo "Spawning Tooltip_03..."
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip_03.sdf" \
    -entity tooltip_03 \
    -x $TT03_WX -y $TT03_WY -z $TT03_WZ \
    -R 0 -P -2.356194 -Y $TT_WYAW

# Tooltip_03-2
echo "Spawning Tooltip_03_2..."
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip_03_2.sdf" \
    -entity tooltip_03_2 \
    -x $TT03_2_WX -y $TT03_2_WY -z $TT03_2_WZ \
    -R 0 -P -2.356194 -Y $TT_WYAW

echo "Done!"



