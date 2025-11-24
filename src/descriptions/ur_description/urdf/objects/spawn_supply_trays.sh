#!/bin/bash

# Script to spawn mobile trays on supply base
# Usage: ./spawn_supply_trays.sh

# Supply base position in world (meters)
SUPPLY_BASE_X=1.0
SUPPLY_BASE_Y=0.0
SUPPLY_BASE_Z=0.92

# Offset values in meters
OFFSET_X=-0.361
OFFSET_Y=-0.9685
OFFSET_H1_Z=0.544
OFFSET_H2_Z=0.434
OFFSET_H3_Z=0.324
OFFSET_H4_Z=0.214
OFFSET_H5_Z=0.104

# Cover offsets from tray origin (in meters, converted from mm)
# Cover_02 (First Level): (0, -31.5, 0) mm = (0, -0.0315, 0) m
# Cover_01 (Second Level): (0, -31.5, 18) mm = (0, -0.0315, 0.018) m
COVER_OFFSET_X=0.0
COVER_OFFSET_Y=-0.0315
COVER_1ST_LEVEL_Z=0.0
COVER_2ND_LEVEL_Z=0.018
PLATE_OFFSET_X=0.0
PLATE_OFFSET_Y=-0.0335
PLATE_1ST_LEVEL_Z=0.0
PLATE_2ND_LEVEL_Z=0.0055

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

# Calculate cover positions for Tray 1 and Tray 4
# Cover_02 (First Level) on Tray 1
COVER_T1_1ST_X=$(awk "BEGIN {print $TRAY_H1_X + $COVER_OFFSET_X}")
COVER_T1_1ST_Y=$(awk "BEGIN {print $TRAY_H1_Y + $COVER_OFFSET_Y}")
COVER_T1_1ST_Z=$(awk "BEGIN {print $TRAY_H1_Z + $COVER_1ST_LEVEL_Z}")

# Cover_01 (Second Level) on Tray 1
COVER_T1_2ND_X=$(awk "BEGIN {print $TRAY_H1_X + $COVER_OFFSET_X}")
COVER_T1_2ND_Y=$(awk "BEGIN {print $TRAY_H1_Y + $COVER_OFFSET_Y}")
COVER_T1_2ND_Z=$(awk "BEGIN {print $TRAY_H1_Z + $COVER_2ND_LEVEL_Z}")

# Cover_02 (First Level) on Tray 4
COVER_T4_1ST_X=$(awk "BEGIN {print $TRAY_H4_X + $COVER_OFFSET_X}")
COVER_T4_1ST_Y=$(awk "BEGIN {print $TRAY_H4_Y + $COVER_OFFSET_Y}")
COVER_T4_1ST_Z=$(awk "BEGIN {print $TRAY_H4_Z + $COVER_1ST_LEVEL_Z}")

# Cover_01 (Second Level) on Tray 4
COVER_T4_2ND_X=$(awk "BEGIN {print $TRAY_H4_X + $COVER_OFFSET_X}")
COVER_T4_2ND_Y=$(awk "BEGIN {print $TRAY_H4_Y + $COVER_OFFSET_Y}")
COVER_T4_2ND_Z=$(awk "BEGIN {print $TRAY_H4_Z + $COVER_2ND_LEVEL_Z}")

# Calculate plate positions for Tray 2 and 5
# Plate_02 (First Level) on Tray 2
Plate_T2_X_1ST=$(awk "BEGIN {print $TRAY_H2_X + $PLATE_OFFSET_X}")
Plate_T2_Y_1ST=$(awk "BEGIN {print $TRAY_H2_Y + $PLATE_OFFSET_Y}")
Plate_T2_Z_1ST=$(awk "BEGIN {print $TRAY_H2_Z + $PLATE_1ST_LEVEL_Z}")

# Plate_01 (Second Level) on Tray 2
Plate_T2_X_2ND=$(awk "BEGIN {print $TRAY_H2_X + $PLATE_OFFSET_X}")
Plate_T2_Y_2ND=$(awk "BEGIN {print $TRAY_H2_Y + $PLATE_OFFSET_Y}")
Plate_T2_Z_2ND=$(awk "BEGIN {print $TRAY_H2_Z + $PLATE_2ND_LEVEL_Z}")

# Plate_02 (First Level) on Tray 5
Plate_T5_X_1ST=$(awk "BEGIN {print $TRAY_H5_X + $PLATE_OFFSET_X}")
Plate_T5_Y_1ST=$(awk "BEGIN {print $TRAY_H5_Y + $PLATE_OFFSET_Y}")
Plate_T5_Z_1ST=$(awk "BEGIN {print $TRAY_H5_Z + $PLATE_1ST_LEVEL_Z}")

# Plate_01 (Second Level) on Tray 5
Plate_T5_X_2ND=$(awk "BEGIN {print $TRAY_H5_X + $PLATE_OFFSET_X}")
Plate_T5_Y_2ND=$(awk "BEGIN {print $TRAY_H5_Y + $PLATE_OFFSET_Y}")
Plate_T5_Z_2ND=$(awk "BEGIN {print $TRAY_H5_Z + $PLATE_2ND_LEVEL_Z}")

# Cover rotation: -90 degrees around Z axis (yaw)
COVER_YAW=-1.5708  # -90 degrees in radians

# Spawn heating plate covers on Tray 1
echo ""
echo "Spawning heating plate covers on Tray 1..."
echo "  - Cover_02 (First Level) at ($COVER_T1_1ST_X, $COVER_T1_1ST_Y, $COVER_T1_1ST_Z) with rotation -90° Z"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate_cover2.sdf" \
    -entity heating_plate_cover_t1_1st \
    -x $COVER_T1_1ST_X -y $COVER_T1_1ST_Y -z $COVER_T1_1ST_Z \
    -R 0 -P 0 -Y $COVER_YAW

echo "  - Cover_01 (Second Level) at ($COVER_T1_2ND_X, $COVER_T1_2ND_Y, $COVER_T1_2ND_Z) with rotation -90° Z"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate_cover1.sdf" \
    -entity heating_plate_cover_t1_2nd \
    -x $COVER_T1_2ND_X -y $COVER_T1_2ND_Y -z $COVER_T1_2ND_Z \
    -R 0 -P 0 -Y $COVER_YAW

# Spawn heating plate covers on Tray 4
echo ""
echo "Spawning heating plate covers on Tray 4..."
echo "  - Cover_02 (First Level) at ($COVER_T4_1ST_X, $COVER_T4_1ST_Y, $COVER_T4_1ST_Z) with rotation -90° Z"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate_cover1.sdf" \
    -entity heating_plate_cover_t4_1st \
    -x $COVER_T4_1ST_X -y $COVER_T4_1ST_Y -z $COVER_T4_1ST_Z \
    -R 0 -P 0 -Y $COVER_YAW

echo "  - Cover_01 (Second Level) at ($COVER_T4_2ND_X, $COVER_T4_2ND_Y, $COVER_T4_2ND_Z) with rotation -90° Z"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate_cover2.sdf" \
    -entity heating_plate_cover_t4_2nd \
    -x $COVER_T4_2ND_X -y $COVER_T4_2ND_Y -z $COVER_T4_2ND_Z \
    -R 0 -P 0 -Y $COVER_YAW

# Spawn heating plates on Tray 2
echo ""
echo "Spawning heating plates on Tray 2..."
echo "  - Plate_02 (First Level) at ($Plate_T2_X_1ST, $Plate_T2_Y_1ST, $Plate_T2_Z_1ST)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate1.sdf" \
    -entity heating_plate_t2_1st \
    -x $Plate_T2_X_1ST -y $Plate_T2_Y_1ST -z $Plate_T2_Z_1ST

echo "  - Plate_01 (Second Level) at ($Plate_T2_X_2ND, $Plate_T2_Y_2ND, $Plate_T2_Z_2ND)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate2.sdf" \
    -entity heating_plate_t2_2nd \
    -x $Plate_T2_X_2ND -y $Plate_T2_Y_2ND -z $Plate_T2_Z_2ND

# Spawn heating plates on Tray 5
echo ""
echo "Spawning heating plates on Tray 5..."
echo "  - Plate_02 (First Level) at ($Plate_T5_X_1ST, $Plate_T5_Y_1ST, $Plate_T5_Z_1ST)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate1.sdf" \
    -entity heating_plate_t5_1st \
    -x $Plate_T5_X_1ST -y $Plate_T5_Y_1ST -z $Plate_T5_Z_1ST

echo "  - Plate_01 (Second Level) at ($Plate_T5_X_2ND, $Plate_T5_Y_2ND, $Plate_T5_Z_2ND)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate2.sdf" \
    -entity heating_plate_t5_2nd \
    -x $Plate_T5_X_2ND -y $Plate_T5_Y_2ND -z $Plate_T5_Z_2ND

echo ""
echo "All mobile trays and covers spawned successfully!"
echo "Objects created:"
echo "  - mobile_tray_h1"
echo "  - mobile_tray_h2"
echo "  - mobile_tray_h3"
echo "  - mobile_tray_h4"
echo "  - mobile_tray_h5"
echo "  - heating_plate_cover_t1_1st "
echo "  - heating_plate_cover_t1_2nd "
echo "  - heating_plate_cover_t4_1st "
echo "  - heating_plate_cover_t4_2nd "
echo "  - heating_plate_t2_1st "
echo "  - heating_plate_t2_2nd "
echo "  - heating_plate_t5_1st "
echo "  - heating_plate_t5_2nd "

