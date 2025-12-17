#!/bin/bash

# Script to spawn mobile trays E1-E6 on feeder base (for electronics)
# Usage: ./spawn_feeder_elec.sh

# Feeder base position in world (meters)
FEEDER_BASE_X=1.0
FEEDER_BASE_Y=0.0
FEEDER_BASE_Z=0.92

# Offset values in meters (converted from mm)
# (-361, -968.5, Z) for all trays
OFFSET_X=-0.361
OFFSET_Y=-0.9685
OFFSET_E1_Z=0.544
OFFSET_E2_Z=0.434
OFFSET_E3_Z=0.324
OFFSET_E4_Z=0.214
OFFSET_E5_Z=0.104
OFFSET_E6_Z=-0.006

# Calculate world positions (base position + offset)
TRAY_E1_X=$(awk "BEGIN {print $FEEDER_BASE_X + $OFFSET_X}")
TRAY_E1_Y=$(awk "BEGIN {print $FEEDER_BASE_Y + $OFFSET_Y}")
TRAY_E1_Z=$(awk "BEGIN {print $FEEDER_BASE_Z + $OFFSET_E1_Z}")

TRAY_E2_X=$(awk "BEGIN {print $FEEDER_BASE_X + $OFFSET_X}")
TRAY_E2_Y=$(awk "BEGIN {print $FEEDER_BASE_Y + $OFFSET_Y}")
TRAY_E2_Z=$(awk "BEGIN {print $FEEDER_BASE_Z + $OFFSET_E2_Z}")

TRAY_E3_X=$(awk "BEGIN {print $FEEDER_BASE_X + $OFFSET_X}")
TRAY_E3_Y=$(awk "BEGIN {print $FEEDER_BASE_Y + $OFFSET_Y}")
TRAY_E3_Z=$(awk "BEGIN {print $FEEDER_BASE_Z + $OFFSET_E3_Z}")

TRAY_E4_X=$(awk "BEGIN {print $FEEDER_BASE_X + $OFFSET_X}")
TRAY_E4_Y=$(awk "BEGIN {print $FEEDER_BASE_Y + $OFFSET_Y}")
TRAY_E4_Z=$(awk "BEGIN {print $FEEDER_BASE_Z + $OFFSET_E4_Z}")

TRAY_E5_X=$(awk "BEGIN {print $FEEDER_BASE_X + $OFFSET_X}")
TRAY_E5_Y=$(awk "BEGIN {print $FEEDER_BASE_Y + $OFFSET_Y}")
TRAY_E5_Z=$(awk "BEGIN {print $FEEDER_BASE_Z + $OFFSET_E5_Z}")

TRAY_E6_X=$(awk "BEGIN {print $FEEDER_BASE_X + $OFFSET_X}")
TRAY_E6_Y=$(awk "BEGIN {print $FEEDER_BASE_Y + $OFFSET_Y}")
TRAY_E6_Z=$(awk "BEGIN {print $FEEDER_BASE_Z + $OFFSET_E6_Z}")

# Get the current workspace directory
OBJECTS_DIR="/root/workcell_ws_classic/src/descriptions/ur_description/urdf/objects"

echo "Objects directory: $OBJECTS_DIR"

# Check if SDF files exist
if [ ! -f "$OBJECTS_DIR/mobile_tray_e1.sdf" ]; then
    echo "Error: mobile_tray_e1.sdf not found at $OBJECTS_DIR/mobile_tray_e1.sdf"
    exit 1
fi

# Spawn trays
echo "Spawning E trays..."
echo "  - Mobile Tray_E1 at ($TRAY_E1_X, $TRAY_E1_Y, $TRAY_E1_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/mobile_tray_e1.sdf" \
    -entity mobile_tray_e1 \
    -x $TRAY_E1_X -y $TRAY_E1_Y -z $TRAY_E1_Z

echo "  - Mobile Tray_E2 at ($TRAY_E2_X, $TRAY_E2_Y, $TRAY_E2_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/mobile_tray_e2.sdf" \
    -entity mobile_tray_e2 \
    -x $TRAY_E2_X -y $TRAY_E2_Y -z $TRAY_E2_Z

echo "  - Mobile Tray_E3 at ($TRAY_E3_X, $TRAY_E3_Y, $TRAY_E3_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/mobile_tray_e3.sdf" \
    -entity mobile_tray_e3 \
    -x $TRAY_E3_X -y $TRAY_E3_Y -z $TRAY_E3_Z

echo "  - Mobile Tray_E4 at ($TRAY_E4_X, $TRAY_E4_Y, $TRAY_E4_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/mobile_tray_e4.sdf" \
    -entity mobile_tray_e4 \
    -x $TRAY_E4_X -y $TRAY_E4_Y -z $TRAY_E4_Z

echo "  - Mobile Tray_E5 at ($TRAY_E5_X, $TRAY_E5_Y, $TRAY_E5_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/mobile_tray_e5.sdf" \
    -entity mobile_tray_e5 \
    -x $TRAY_E5_X -y $TRAY_E5_Y -z $TRAY_E5_Z

echo "  - Mobile Tray_E6 at ($TRAY_E6_X, $TRAY_E6_Y, $TRAY_E6_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/mobile_tray_e6.sdf" \
    -entity mobile_tray_e6 \
    -x $TRAY_E6_X -y $TRAY_E6_Y -z $TRAY_E6_Z


# Electrical components offsets from tray origin (in meters, converted from mm)

# MCCB_ABE_32B_30A: (-88.5, 14.5, -3) mm
MCCB_OFFSET_X=-0.0885
MCCB_OFFSET_Y=0.0145
MCCB_OFFSET_Z=-0.003

# PDU_SPS25-M66XM4 #1: (34.5, 31, -3) mm
PDU1_OFFSET_X=0.0345
PDU1_OFFSET_Y=0.031
PDU1_OFFSET_Z=-0.003

# PDU_SPS25-M66XM4 #2: (34.5, -17, -3) mm
PDU2_OFFSET_X=0.0345
PDU2_OFFSET_Y=-0.017
PDU2_OFFSET_Z=-0.003

# Noise_filter_RMS-2030-DIN: (-90.7, -66, 10) mm
NOISE_FILTER_OFFSET_X=-0.0907
NOISE_FILTER_OFFSET_Y=-0.066
NOISE_FILTER_OFFSET_Z=0.01

# Plug_socket_DRC-220V-16A: (33.5, -73.5, 3) mm
PLUG_SOCKET_OFFSET_X=0.0335
PLUG_SOCKET_OFFSET_Y=-0.0735
PLUG_SOCKET_OFFSET_Z=0.003

# Busbar_6P: (126, -32, -3) mm
BUSBAR_OFFSET_X=0.126
BUSBAR_OFFSET_Y=-0.032
BUSBAR_OFFSET_Z=-0.003


# E2 & E5 tray components offsets

# Single-MC_GMC_30P2 #1: (-113.6, 14.9, -3) mm
SINGLE_MC1_OFFSET_X=-0.1136
SINGLE_MC1_OFFSET_Y=0.0149
SINGLE_MC1_OFFSET_Z=-0.003

# Single-MC_GMC_30P2 #2: (-41.6, 14.9, -3) mm
SINGLE_MC2_OFFSET_X=-0.0416
SINGLE_MC2_OFFSET_Y=0.0149
SINGLE_MC2_OFFSET_Z=-0.003

# Single-MC_GMC_30P2 #3: (-77.6, -72.3, -3) mm
SINGLE_MC3_OFFSET_X=-0.0776
SINGLE_MC3_OFFSET_Y=-0.0723
SINGLE_MC3_OFFSET_Z=-0.003

# SMPS_WDR_120_24V: (79.7, -24.5, 15) mm
SMPS_OFFSET_X=0.0797
SMPS_OFFSET_Y=-0.0245
SMPS_OFFSET_Z=0.015


# Calculate positions for E1 tray components
MCCB_E1_X=$(awk "BEGIN {print $TRAY_E1_X + $MCCB_OFFSET_X}")
MCCB_E1_Y=$(awk "BEGIN {print $TRAY_E1_Y + $MCCB_OFFSET_Y}")
MCCB_E1_Z=$(awk "BEGIN {print $TRAY_E1_Z + $MCCB_OFFSET_Z}")

PDU1_E1_X=$(awk "BEGIN {print $TRAY_E1_X + $PDU1_OFFSET_X}")
PDU1_E1_Y=$(awk "BEGIN {print $TRAY_E1_Y + $PDU1_OFFSET_Y}")
PDU1_E1_Z=$(awk "BEGIN {print $TRAY_E1_Z + $PDU1_OFFSET_Z}")

PDU2_E1_X=$(awk "BEGIN {print $TRAY_E1_X + $PDU2_OFFSET_X}")
PDU2_E1_Y=$(awk "BEGIN {print $TRAY_E1_Y + $PDU2_OFFSET_Y}")
PDU2_E1_Z=$(awk "BEGIN {print $TRAY_E1_Z + $PDU2_OFFSET_Z}")

NOISE_FILTER_E1_X=$(awk "BEGIN {print $TRAY_E1_X + $NOISE_FILTER_OFFSET_X}")
NOISE_FILTER_E1_Y=$(awk "BEGIN {print $TRAY_E1_Y + $NOISE_FILTER_OFFSET_Y}")
NOISE_FILTER_E1_Z=$(awk "BEGIN {print $TRAY_E1_Z + $NOISE_FILTER_OFFSET_Z}")

PLUG_SOCKET_E1_X=$(awk "BEGIN {print $TRAY_E1_X + $PLUG_SOCKET_OFFSET_X}")
PLUG_SOCKET_E1_Y=$(awk "BEGIN {print $TRAY_E1_Y + $PLUG_SOCKET_OFFSET_Y}")
PLUG_SOCKET_E1_Z=$(awk "BEGIN {print $TRAY_E1_Z + $PLUG_SOCKET_OFFSET_Z}")

BUSBAR_E1_X=$(awk "BEGIN {print $TRAY_E1_X + $BUSBAR_OFFSET_X}")
BUSBAR_E1_Y=$(awk "BEGIN {print $TRAY_E1_Y + $BUSBAR_OFFSET_Y}")
BUSBAR_E1_Z=$(awk "BEGIN {print $TRAY_E1_Z + $BUSBAR_OFFSET_Z}")


# Calculate positions for E4 tray components
MCCB_E4_X=$(awk "BEGIN {print $TRAY_E4_X + $MCCB_OFFSET_X}")
MCCB_E4_Y=$(awk "BEGIN {print $TRAY_E4_Y + $MCCB_OFFSET_Y}")
MCCB_E4_Z=$(awk "BEGIN {print $TRAY_E4_Z + $MCCB_OFFSET_Z}")

PDU1_E4_X=$(awk "BEGIN {print $TRAY_E4_X + $PDU1_OFFSET_X}")
PDU1_E4_Y=$(awk "BEGIN {print $TRAY_E4_Y + $PDU1_OFFSET_Y}")
PDU1_E4_Z=$(awk "BEGIN {print $TRAY_E4_Z + $PDU1_OFFSET_Z}")

PDU2_E4_X=$(awk "BEGIN {print $TRAY_E4_X + $PDU2_OFFSET_X}")
PDU2_E4_Y=$(awk "BEGIN {print $TRAY_E4_Y + $PDU2_OFFSET_Y}")
PDU2_E4_Z=$(awk "BEGIN {print $TRAY_E4_Z + $PDU2_OFFSET_Z}")

NOISE_FILTER_E4_X=$(awk "BEGIN {print $TRAY_E4_X + $NOISE_FILTER_OFFSET_X}")
NOISE_FILTER_E4_Y=$(awk "BEGIN {print $TRAY_E4_Y + $NOISE_FILTER_OFFSET_Y}")
NOISE_FILTER_E4_Z=$(awk "BEGIN {print $TRAY_E4_Z + $NOISE_FILTER_OFFSET_Z}")

PLUG_SOCKET_E4_X=$(awk "BEGIN {print $TRAY_E4_X + $PLUG_SOCKET_OFFSET_X}")
PLUG_SOCKET_E4_Y=$(awk "BEGIN {print $TRAY_E4_Y + $PLUG_SOCKET_OFFSET_Y}")
PLUG_SOCKET_E4_Z=$(awk "BEGIN {print $TRAY_E4_Z + $PLUG_SOCKET_OFFSET_Z}")

BUSBAR_E4_X=$(awk "BEGIN {print $TRAY_E4_X + $BUSBAR_OFFSET_X}")
BUSBAR_E4_Y=$(awk "BEGIN {print $TRAY_E4_Y + $BUSBAR_OFFSET_Y}")
BUSBAR_E4_Z=$(awk "BEGIN {print $TRAY_E4_Z + $BUSBAR_OFFSET_Z}")


# Calculate positions for E2 tray components
SINGLE_MC1_E2_X=$(awk "BEGIN {print $TRAY_E2_X + $SINGLE_MC1_OFFSET_X}")
SINGLE_MC1_E2_Y=$(awk "BEGIN {print $TRAY_E2_Y + $SINGLE_MC1_OFFSET_Y}")
SINGLE_MC1_E2_Z=$(awk "BEGIN {print $TRAY_E2_Z + $SINGLE_MC1_OFFSET_Z}")

SINGLE_MC2_E2_X=$(awk "BEGIN {print $TRAY_E2_X + $SINGLE_MC2_OFFSET_X}")
SINGLE_MC2_E2_Y=$(awk "BEGIN {print $TRAY_E2_Y + $SINGLE_MC2_OFFSET_Y}")
SINGLE_MC2_E2_Z=$(awk "BEGIN {print $TRAY_E2_Z + $SINGLE_MC2_OFFSET_Z}")

SINGLE_MC3_E2_X=$(awk "BEGIN {print $TRAY_E2_X + $SINGLE_MC3_OFFSET_X}")
SINGLE_MC3_E2_Y=$(awk "BEGIN {print $TRAY_E2_Y + $SINGLE_MC3_OFFSET_Y}")
SINGLE_MC3_E2_Z=$(awk "BEGIN {print $TRAY_E2_Z + $SINGLE_MC3_OFFSET_Z}")

SMPS_E2_X=$(awk "BEGIN {print $TRAY_E2_X + $SMPS_OFFSET_X}")
SMPS_E2_Y=$(awk "BEGIN {print $TRAY_E2_Y + $SMPS_OFFSET_Y}")
SMPS_E2_Z=$(awk "BEGIN {print $TRAY_E2_Z + $SMPS_OFFSET_Z}")


# Calculate positions for E5 tray components
SINGLE_MC1_E5_X=$(awk "BEGIN {print $TRAY_E5_X + $SINGLE_MC1_OFFSET_X}")
SINGLE_MC1_E5_Y=$(awk "BEGIN {print $TRAY_E5_Y + $SINGLE_MC1_OFFSET_Y}")
SINGLE_MC1_E5_Z=$(awk "BEGIN {print $TRAY_E5_Z + $SINGLE_MC1_OFFSET_Z}")

SINGLE_MC2_E5_X=$(awk "BEGIN {print $TRAY_E5_X + $SINGLE_MC2_OFFSET_X}")
SINGLE_MC2_E5_Y=$(awk "BEGIN {print $TRAY_E5_Y + $SINGLE_MC2_OFFSET_Y}")
SINGLE_MC2_E5_Z=$(awk "BEGIN {print $TRAY_E5_Z + $SINGLE_MC2_OFFSET_Z}")

SINGLE_MC3_E5_X=$(awk "BEGIN {print $TRAY_E5_X + $SINGLE_MC3_OFFSET_X}")
SINGLE_MC3_E5_Y=$(awk "BEGIN {print $TRAY_E5_Y + $SINGLE_MC3_OFFSET_Y}")
SINGLE_MC3_E5_Z=$(awk "BEGIN {print $TRAY_E5_Z + $SINGLE_MC3_OFFSET_Z}")

SMPS_E5_X=$(awk "BEGIN {print $TRAY_E5_X + $SMPS_OFFSET_X}")
SMPS_E5_Y=$(awk "BEGIN {print $TRAY_E5_Y + $SMPS_OFFSET_Y}")
SMPS_E5_Z=$(awk "BEGIN {print $TRAY_E5_Z + $SMPS_OFFSET_Z}")


# Spawn electrical components on E1 tray
echo ""
echo "Spawning electrical components on E1 tray..."

echo "  - MCCB_ABE_32B_30A at ($MCCB_E1_X, $MCCB_E1_Y, $MCCB_E1_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/mccb_abe_32b_30a.sdf" \
    -entity mccb_abe_32b_30a_e1 \
    -x $MCCB_E1_X -y $MCCB_E1_Y -z $MCCB_E1_Z

echo "  - PDU_SPS25-M66XM4 #1 at ($PDU1_E1_X, $PDU1_E1_Y, $PDU1_E1_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/pdu_sps25_m66xm4.sdf" \
    -entity pdu_sps25_m66xm4_e1_1 \
    -x $PDU1_E1_X -y $PDU1_E1_Y -z $PDU1_E1_Z

echo "  - PDU_SPS25-M66XM4 #2 at ($PDU2_E1_X, $PDU2_E1_Y, $PDU2_E1_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/pdu_sps25_m66xm4.sdf" \
    -entity pdu_sps25_m66xm4_e1_2 \
    -x $PDU2_E1_X -y $PDU2_E1_Y -z $PDU2_E1_Z

echo "  - Noise_filter_RMS-2030-DIN at ($NOISE_FILTER_E1_X, $NOISE_FILTER_E1_Y, $NOISE_FILTER_E1_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/noise_filter_rms_2030_din.sdf" \
    -entity noise_filter_rms_2030_din_e1 \
    -x $NOISE_FILTER_E1_X -y $NOISE_FILTER_E1_Y -z $NOISE_FILTER_E1_Z

echo "  - Plug_socket_DRC-220V-16A at ($PLUG_SOCKET_E1_X, $PLUG_SOCKET_E1_Y, $PLUG_SOCKET_E1_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/plug_socket_drc_220v_16a.sdf" \
    -entity plug_socket_drc_220v_16a_e1 \
    -x $PLUG_SOCKET_E1_X -y $PLUG_SOCKET_E1_Y -z $PLUG_SOCKET_E1_Z

echo "  - Busbar_6P at ($BUSBAR_E1_X, $BUSBAR_E1_Y, $BUSBAR_E1_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/busbar_6p.sdf" \
    -entity busbar_6p_e1 \
    -x $BUSBAR_E1_X -y $BUSBAR_E1_Y -z $BUSBAR_E1_Z


# Spawn electrical components on E4 tray
echo ""
echo "Spawning electrical components on E4 tray..."

echo "  - MCCB_ABE_32B_30A at ($MCCB_E4_X, $MCCB_E4_Y, $MCCB_E4_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/mccb_abe_32b_30a.sdf" \
    -entity mccb_abe_32b_30a_e4 \
    -x $MCCB_E4_X -y $MCCB_E4_Y -z $MCCB_E4_Z

echo "  - PDU_SPS25-M66XM4 #1 at ($PDU1_E4_X, $PDU1_E4_Y, $PDU1_E4_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/pdu_sps25_m66xm4.sdf" \
    -entity pdu_sps25_m66xm4_e4_1 \
    -x $PDU1_E4_X -y $PDU1_E4_Y -z $PDU1_E4_Z

echo "  - PDU_SPS25-M66XM4 #2 at ($PDU2_E4_X, $PDU2_E4_Y, $PDU2_E4_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/pdu_sps25_m66xm4.sdf" \
    -entity pdu_sps25_m66xm4_e4_2 \
    -x $PDU2_E4_X -y $PDU2_E4_Y -z $PDU2_E4_Z

echo "  - Noise_filter_RMS-2030-DIN at ($NOISE_FILTER_E4_X, $NOISE_FILTER_E4_Y, $NOISE_FILTER_E4_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/noise_filter_rms_2030_din.sdf" \
    -entity noise_filter_rms_2030_din_e4 \
    -x $NOISE_FILTER_E4_X -y $NOISE_FILTER_E4_Y -z $NOISE_FILTER_E4_Z

echo "  - Plug_socket_DRC-220V-16A at ($PLUG_SOCKET_E4_X, $PLUG_SOCKET_E4_Y, $PLUG_SOCKET_E4_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/plug_socket_drc_220v_16a.sdf" \
    -entity plug_socket_drc_220v_16a_e4 \
    -x $PLUG_SOCKET_E4_X -y $PLUG_SOCKET_E4_Y -z $PLUG_SOCKET_E4_Z

echo "  - Busbar_6P at ($BUSBAR_E4_X, $BUSBAR_E4_Y, $BUSBAR_E4_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/busbar_6p.sdf" \
    -entity busbar_6p_e4 \
    -x $BUSBAR_E4_X -y $BUSBAR_E4_Y -z $BUSBAR_E4_Z


# Spawn electrical components on E2 tray
echo ""
echo "Spawning electrical components on E2 tray..."

echo "  - Single-MC_GMC #1 at ($SINGLE_MC1_E2_X, $SINGLE_MC1_E2_Y, $SINGLE_MC1_E2_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" \
    -entity single_mc_gmc_e2_1 \
    -x $SINGLE_MC1_E2_X -y $SINGLE_MC1_E2_Y -z $SINGLE_MC1_E2_Z

echo "  - Single-MC_GMC #2 at ($SINGLE_MC2_E2_X, $SINGLE_MC2_E2_Y, $SINGLE_MC2_E2_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" \
    -entity single_mc_gmc_e2_2 \
    -x $SINGLE_MC2_E2_X -y $SINGLE_MC2_E2_Y -z $SINGLE_MC2_E2_Z

echo "  - Single-MC_GMC #3 at ($SINGLE_MC3_E2_X, $SINGLE_MC3_E2_Y, $SINGLE_MC3_E2_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" \
    -entity single_mc_gmc_e2_3 \
    -x $SINGLE_MC3_E2_X -y $SINGLE_MC3_E2_Y -z $SINGLE_MC3_E2_Z

echo "  - SMPS_WDR_120_24V at ($SMPS_E2_X, $SMPS_E2_Y, $SMPS_E2_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/smps_wdr_120_24v.sdf" \
    -entity smps_wdr_120_24v_e2 \
    -x $SMPS_E2_X -y $SMPS_E2_Y -z $SMPS_E2_Z


# Spawn electrical components on E5 tray
echo ""
echo "Spawning electrical components on E5 tray..."

echo "  - Single-MC_GMC #1 at ($SINGLE_MC1_E5_X, $SINGLE_MC1_E5_Y, $SINGLE_MC1_E5_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" \
    -entity single_mc_gmc_e5_1 \
    -x $SINGLE_MC1_E5_X -y $SINGLE_MC1_E5_Y -z $SINGLE_MC1_E5_Z

echo "  - Single-MC_GMC #2 at ($SINGLE_MC2_E5_X, $SINGLE_MC2_E5_Y, $SINGLE_MC2_E5_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" \
    -entity single_mc_gmc_e5_2 \
    -x $SINGLE_MC2_E5_X -y $SINGLE_MC2_E5_Y -z $SINGLE_MC2_E5_Z

echo "  - Single-MC_GMC #3 at ($SINGLE_MC3_E5_X, $SINGLE_MC3_E5_Y, $SINGLE_MC3_E5_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" \
    -entity single_mc_gmc_e5_3 \
    -x $SINGLE_MC3_E5_X -y $SINGLE_MC3_E5_Y -z $SINGLE_MC3_E5_Z

echo "  - SMPS_WDR_120_24V at ($SMPS_E5_X, $SMPS_E5_Y, $SMPS_E5_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/smps_wdr_120_24v.sdf" \
    -entity smps_wdr_120_24v_e5 \
    -x $SMPS_E5_X -y $SMPS_E5_Y -z $SMPS_E5_Z


# TB_JOTN-15A grid for E3 and E6 trays
TB_OFFSET_Z=0.002

# X positions (38.5mm spacing)
TB_X1=-0.13475
TB_X2=-0.09625
TB_X3=-0.05775
TB_X4=-0.01925
TB_X5=0.01925
TB_X6=0.05775
TB_X7=0.09625
TB_X8=0.13475

# Y positions (75mm spacing)
TB_Y1=-0.0675
TB_Y2=0.0075


# Spawn TB_JOTN-15A on E3 tray (16 units)
echo ""
echo "Spawning TB_JOTN-15A grid on E3 tray (2x8 = 16 units)..."

# Row 1 (Y = -67.5mm)
for i in 1 2 3 4 5 6 7 8; do
    eval "TB_X=\$TB_X$i"
    TB_POS_X=$(awk "BEGIN {print $TRAY_E3_X + $TB_X}")
    TB_POS_Y=$(awk "BEGIN {print $TRAY_E3_Y + $TB_Y1}")
    TB_POS_Z=$(awk "BEGIN {print $TRAY_E3_Z + $TB_OFFSET_Z}")
    echo "  - TB_JOTN-15A_${i}_1 at ($TB_POS_X, $TB_POS_Y, $TB_POS_Z)"
    ros2 run gazebo_ros spawn_entity.py \
        -file "$OBJECTS_DIR/tb_jotn_15a.sdf" \
        -entity tb_jotn_15a_e3_${i}_1 \
        -x $TB_POS_X -y $TB_POS_Y -z $TB_POS_Z
done

# Row 2 (Y = 7.5mm)
for i in 1 2 3 4 5 6 7 8; do
    eval "TB_X=\$TB_X$i"
    TB_POS_X=$(awk "BEGIN {print $TRAY_E3_X + $TB_X}")
    TB_POS_Y=$(awk "BEGIN {print $TRAY_E3_Y + $TB_Y2}")
    TB_POS_Z=$(awk "BEGIN {print $TRAY_E3_Z + $TB_OFFSET_Z}")
    echo "  - TB_JOTN-15A_${i}_2 at ($TB_POS_X, $TB_POS_Y, $TB_POS_Z)"
    ros2 run gazebo_ros spawn_entity.py \
        -file "$OBJECTS_DIR/tb_jotn_15a.sdf" \
        -entity tb_jotn_15a_e3_${i}_2 \
        -x $TB_POS_X -y $TB_POS_Y -z $TB_POS_Z
done


# Spawn TB_JOTN-15A on E6 tray (16 units)
echo ""
echo "Spawning TB_JOTN-15A grid on E6 tray (2x8 = 16 units)..."

# Row 1 (Y = -67.5mm)
for i in 1 2 3 4 5 6 7 8; do
    eval "TB_X=\$TB_X$i"
    TB_POS_X=$(awk "BEGIN {print $TRAY_E6_X + $TB_X}")
    TB_POS_Y=$(awk "BEGIN {print $TRAY_E6_Y + $TB_Y1}")
    TB_POS_Z=$(awk "BEGIN {print $TRAY_E6_Z + $TB_OFFSET_Z}")
    echo "  - TB_JOTN-15A_${i}_1 at ($TB_POS_X, $TB_POS_Y, $TB_POS_Z)"
    ros2 run gazebo_ros spawn_entity.py \
        -file "$OBJECTS_DIR/tb_jotn_15a.sdf" \
        -entity tb_jotn_15a_e6_${i}_1 \
        -x $TB_POS_X -y $TB_POS_Y -z $TB_POS_Z
done

# Row 2 (Y = 7.5mm)
for i in 1 2 3 4 5 6 7 8; do
    eval "TB_X=\$TB_X$i"
    TB_POS_X=$(awk "BEGIN {print $TRAY_E6_X + $TB_X}")
    TB_POS_Y=$(awk "BEGIN {print $TRAY_E6_Y + $TB_Y2}")
    TB_POS_Z=$(awk "BEGIN {print $TRAY_E6_Z + $TB_OFFSET_Z}")
    echo "  - TB_JOTN-15A_${i}_2 at ($TB_POS_X, $TB_POS_Y, $TB_POS_Z)"
    ros2 run gazebo_ros spawn_entity.py \
        -file "$OBJECTS_DIR/tb_jotn_15a.sdf" \
        -entity tb_jotn_15a_e6_${i}_2 \
        -x $TB_POS_X -y $TB_POS_Y -z $TB_POS_Z
done

echo ""
echo "All E trays and electrical components spawned successfully!"
