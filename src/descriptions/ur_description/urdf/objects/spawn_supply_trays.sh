#!/bin/bash

# Script to spawn mobile trays on supply base
# Usage: ./spawn_supply_trays.sh

# Supply base position in world (meters)
SUPPLY_BASE_X=1.0
SUPPLY_BASE_Y=0.0
SUPPLY_BASE_Z=0.92

# Offsets from supply base origin (in meters)
# Tray positions relative to supply base:
# H1: (-361, -968.5, 544) mm = (-0.361, -0.9685, 0.544) m
# H2: (-361, -968.5, 434) mm = (-0.361, -0.9685, 0.434) m
# H3: (-361, -968.5, 324) mm = (-0.361, -0.9685, 0.324) m
# H4: (-361, -968.5, 214) mm = (-0.361, -0.9685, 0.214) m
# H5: (-361, -968.5, 104) mm = (-0.361, -0.9685, 0.104) m

# Offset values in meters
OFFSET_X=-0.361
OFFSET_Y=-0.9685
OFFSET_H1_Z=0.544
OFFSET_H2_Z=0.434
OFFSET_H3_Z=0.324
OFFSET_H4_Z=0.214
OFFSET_H5_Z=0.104

# Calculate world positions (base position + offset)
TRAY_H1_X=$(awk "BEGIN {print $SUPPLY_BASE_X + $OFFSET_X}")
TRAY_H1_Y=$(awk "BEGIN {print $SUPPLY_BASE_Y + $OFFSET_Y}")
TRAY_H1_Z=$(awk "BEGIN {print $SUPPLY_BASE_Z + $OFFSET_H1_Z}")

TRAY_H2_X=$(awk "BEGIN {print $SUPPLY_BASE_X + $OFFSET_X}")
TRAY_H2_Y=$(awk "BEGIN {print $SUPPLY_BASE_Y + $OFFSET_Y}")
TRAY_H2_Z=$(awk "BEGIN {print $SUPPLY_BASE_Z + $OFFSET_H2_Z}")

TRAY_H3_X=$(awk "BEGIN {print $SUPPLY_BASE_X + $OFFSET_X}")
TRAY_H3_Y=$(awk "BEGIN {print $SUPPLY_BASE_Y + $OFFSET_Y}")
TRAY_H3_Z=$(awk "BEGIN {print $SUPPLY_BASE_Z + $OFFSET_H3_Z}")

TRAY_H4_X=$(awk "BEGIN {print $SUPPLY_BASE_X + $OFFSET_X}")
TRAY_H4_Y=$(awk "BEGIN {print $SUPPLY_BASE_Y + $OFFSET_Y}")
TRAY_H4_Z=$(awk "BEGIN {print $SUPPLY_BASE_Z + $OFFSET_H4_Z}")

TRAY_H5_X=$(awk "BEGIN {print $SUPPLY_BASE_X + $OFFSET_X}")
TRAY_H5_Y=$(awk "BEGIN {print $SUPPLY_BASE_Y + $OFFSET_Y}")
TRAY_H5_Z=$(awk "BEGIN {print $SUPPLY_BASE_Z + $OFFSET_H5_Z}")

echo "Spawning mobile trays on supply base"
echo "Supply base position: ($SUPPLY_BASE_X, $SUPPLY_BASE_Y, $SUPPLY_BASE_Z)"
echo ""
echo "Tray positions (world coordinates):"
echo "  Mobile Tray_H1: ($TRAY_H1_X, $TRAY_H1_Y, $TRAY_H1_Z)"
echo "  Mobile Tray_H2: ($TRAY_H2_X, $TRAY_H2_Y, $TRAY_H2_Z)"
echo "  Mobile Tray_H3: ($TRAY_H3_X, $TRAY_H3_Y, $TRAY_H3_Z)"
echo "  Mobile Tray_H4: ($TRAY_H4_X, $TRAY_H4_Y, $TRAY_H4_Z)"
echo "  Mobile Tray_H5: ($TRAY_H5_X, $TRAY_H5_Y, $TRAY_H5_Z)"
echo ""

# Get the current workspace directory
OBJECTS_DIR="/root/workcell_ws_classic/src/descriptions/ur_description/urdf/objects"

echo "Objects directory: $OBJECTS_DIR"

# Check if SDF files exist
if [ ! -f "$OBJECTS_DIR/mobile_tray_h1.sdf" ]; then
    echo "Error: mobile_tray_h1.sdf not found at $OBJECTS_DIR/mobile_tray_h1.sdf"
    exit 1
fi

# Spawn trays
echo "Spawning trays..."
echo "  - Mobile Tray_H1 at ($TRAY_H1_X, $TRAY_H1_Y, $TRAY_H1_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/mobile_tray_h1.sdf" \
    -entity mobile_tray_h1 \
    -x $TRAY_H1_X -y $TRAY_H1_Y -z $TRAY_H1_Z

echo "  - Mobile Tray_H2 at ($TRAY_H2_X, $TRAY_H2_Y, $TRAY_H2_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/mobile_tray_h2.sdf" \
    -entity mobile_tray_h2 \
    -x $TRAY_H2_X -y $TRAY_H2_Y -z $TRAY_H2_Z

echo "  - Mobile Tray_H3 at ($TRAY_H3_X, $TRAY_H3_Y, $TRAY_H3_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/mobile_tray_h3.sdf" \
    -entity mobile_tray_h3 \
    -x $TRAY_H3_X -y $TRAY_H3_Y -z $TRAY_H3_Z

echo "  - Mobile Tray_H4 at ($TRAY_H4_X, $TRAY_H4_Y, $TRAY_H4_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/mobile_tray_h4.sdf" \
    -entity mobile_tray_h4 \
    -x $TRAY_H4_X -y $TRAY_H4_Y -z $TRAY_H4_Z

echo "  - Mobile Tray_H5 at ($TRAY_H5_X, $TRAY_H5_Y, $TRAY_H5_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/mobile_tray_h5.sdf" \
    -entity mobile_tray_h5 \
    -x $TRAY_H5_X -y $TRAY_H5_Y -z $TRAY_H5_Z

echo ""
echo "All mobile trays spawned successfully!"
echo "Objects created:"
echo "  - mobile_tray_h1"
echo "  - mobile_tray_h2"
echo "  - mobile_tray_h3"
echo "  - mobile_tray_h4"
echo "  - mobile_tray_h5"

