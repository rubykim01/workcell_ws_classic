#!/bin/bash

# Script to spawn tools on the tool changer base assembly stand
# Usage: ./spawn_toolchanger_tools.sh

OBJECTS_DIR="/root/workcell_ws_classic/src/descriptions/ur_description/urdf/objects"

# Cart position from World (m)
CART_X=-0.8
CART_Y=0
CART_Z=0.86

# Tool Changer position from Cart (m)
TC_X=0.3
TC_Y=0
TC_Z=0

# Tool offsets from Tool Changer (m)
# KRVG
KRVG_X=-0.1208
KRVG_Y=0.311
KRVG_Z=0.4994

# Koras_2F100
KORAS_X=-0.1208
KORAS_Y=0.1615
KORAS_Z=0.4994

# Tooltip_01-2
TT01_X=-0.1085
TT01_Y=0.0335
TT01_Z=0.5532

# Tooltip_02-2
TT02_X=-0.1085
TT02_Y=-0.116
TT02_Z=0.5532

# Tooltip_03-2
TT03_X=-0.1085
TT03_Y=-0.2655
TT03_Z=0.5532

echo "Spawning tools on Tool Changer Stand..."

# KRVG
echo "Spawning KRVG..."
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/krvg.sdf" \
    -entity krvg \
    -x $(awk "BEGIN {print $CART_X + $TC_X + $KRVG_X}") \
    -y $(awk "BEGIN {print $CART_Y + $TC_Y + $KRVG_Y}") \
    -z $(awk "BEGIN {print $CART_Z + $TC_Z + $KRVG_Z}") \
    -R 0 -P 0 -Y 0

# Koras_2F100
echo "Spawning Koras_2F100..."
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/koras_2f100.sdf" \
    -entity koras_2f100 \
    -x $(awk "BEGIN {print $CART_X + $TC_X + $KORAS_X}") \
    -y $(awk "BEGIN {print $CART_Y + $TC_Y + $KORAS_Y}") \
    -z $(awk "BEGIN {print $CART_Z + $TC_Z + $KORAS_Z}") \
    -R 0 -P 0 -Y 0

# Tooltip_01-2
echo "Spawning Tooltip_01_2..."
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip_01_2.sdf" \
    -entity tooltip_01_2 \
    -x $(awk "BEGIN {print $CART_X + $TC_X + $TT01_X}") \
    -y $(awk "BEGIN {print $CART_Y + $TC_Y + $TT01_Y}") \
    -z $(awk "BEGIN {print $CART_Z + $TC_Z + $TT01_Z}") \
    -R 0 -P 0 -Y 0

# Tooltip_02-2
echo "Spawning Tooltip_02_2..."
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip_02_2.sdf" \
    -entity tooltip_02_2 \
    -x $(awk "BEGIN {print $CART_X + $TC_X + $TT02_X}") \
    -y $(awk "BEGIN {print $CART_Y + $TC_Y + $TT02_Y}") \
    -z $(awk "BEGIN {print $CART_Z + $TC_Z + $TT02_Z}") \
    -R 0 -P 0 -Y 0

# Tooltip_03-2
echo "Spawning Tooltip_03_2..."
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/tooltip_03_2.sdf" \
    -entity tooltip_03_2 \
    -x $(awk "BEGIN {print $CART_X + $TC_X + $TT03_X}") \
    -y $(awk "BEGIN {print $CART_Y + $TC_Y + $TT03_Y}") \
    -z $(awk "BEGIN {print $CART_Z + $TC_Z + $TT03_Z}") \
    -R 0 -P 0 -Y 0

echo "Done!"
