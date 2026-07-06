from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton,
    QComboBox, QMessageBox, QLabel, QGridLayout, QHBoxLayout
)
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt, QProcess, QTimer
import math
import yaml
import subprocess
import signal
import sys
import time
from pathlib import Path

# Discrete yaw options exposed in the GUI (degrees) and the matching radian values written to YAML.
ROTATION_LABELS = ["0", "90", "180", "-90"]
ROTATION_LABEL_TO_RAD = {label: round(math.radians(int(label)), 4) for label in ROTATION_LABELS}

# Feeder world-frame (dx, dy) added to the trolley grid origin, per yaw label.
# Calibrated so yaw=-90° reproduces the historical (0.0745, 0.215) offset; the
# other rotations rotate that same body-frame mount point about the grid origin.
FEEDER_OFFSET_BY_LABEL = {
    "0":   (-0.215, 0.0745),
    "90":  (-0.0745, -0.215),
    "180": (0.215, -0.0745),
    "-90": (0.0745, 0.215),
}


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Spawn Configuration")

        # Set window size
        #self.resize(400, 500)
        self.setMinimumSize(950, 850)
        self.setStyleSheet("background-color: #ffffff;")
        # Create central widget 
        window = QWidget()
        self.setCentralWidget(window)

        # Main layout
        main_layout = QVBoxLayout()

        # Trolley options for dropdowns
        self.trolley_options = ["-", "toolchanger", "denso", "ur", "arf", "vision", "feeder", "arf_elec"]
        
        # Position to coordinates mapping
        self.position_coordinates = {
            1: {"x": -0.8, "y": 0.0, "z": 0.86},
            2: {"x": -0.8, "y": 0.8, "z": 0.86},
            3: {"x": -0.8, "y": 1.6, "z": 0.86},
            4: {"x": 0.0, "y": 0.0, "z": 0.86},
            5: {"x": 0.0, "y": 0.8, "z": 0.86},
            6: {"x": 0.0, "y": 1.6, "z": 0.86},
            7: {"x": 0.8, "y": 0.0, "z": 0.86},
            8: {"x": 0.8, "y": 0.8, "z": 0.86},
            9: {"x": 0.8, "y": 1.6, "z": 0.86}
        }
        
        # Path to trolley_positions.yaml
        script_dir = Path(__file__).parent.absolute()
        workspace_root = script_dir.parent.parent.parent
        self.yaml_path = workspace_root / "src" / "descriptions" / "config" / "trolley_positions.yaml"

        # Default trolley assignments (built from YAML by matching coords to grid positions).
        # Trolleys whose coords don't match any grid slot are tracked as custom and preserved on save.
        self.default_assignments = {
            1: "feeder",
            2: "toolchanger",
            3: "denso",
            4: "-",
            5: "ur",
            6: "arf",
            7: "-",
            8: "vision",
            9: "-"
        }
        self.custom_trolleys = set()

        # Per-trolley discrete yaw label ("0"/"90"/"180"/"-90"). Defaults below;
        # _load_defaults_from_yaml overrides any values present in YAML.
        self.default_trolley_rotation = {t: "0" for t in self.trolley_options[1:]}
        self.default_trolley_rotation['feeder'] = "-90"
        self.trolley_rotation = dict(self.default_trolley_rotation)

        self._load_defaults_from_yaml()
        
        combobox_label = QLabel("[Configure Spawn Positions]")
        combobox_label.setAlignment(Qt.AlignCenter)
        combobox_label.setStyleSheet("font-weight: bold; font-size: 16px; margin-top: 15px; color: #333333;")
        main_layout.addWidget(combobox_label)
        
        # Add spacing between label and grid
        main_layout.addSpacing(5)
        
        # Create 3x3 grid layout
        grid_layout = QGridLayout()
        grid_layout.setSpacing(15)
        main_layout.addLayout(grid_layout)
        
        # Store comboboxes and rotation comboboxes by position number
        self.position_comboboxes = {}
        self.position_rotation_comboboxes = {}

        # Create 9 boxes (3x3 grid) for positions 1-9
        position = 1
        for row in range(3):
            for col in range(3):
                # Create container widget for each position
                position_widget = QWidget()
                position_widget.setFixedSize(200, 190)
                # Add border to create lines between boxes
                position_widget.setStyleSheet("border: 10px solid #cccccc; background-color: #f0f0f0; padding: 5px;")
                position_layout = QVBoxLayout()
                position_layout.setSpacing(5)
                position_layout.setContentsMargins(1,1,1,1)

                # Position number label
                position_label = QLabel(f"Position {position}")
                position_label.setAlignment(Qt.AlignCenter)
                position_label.setStyleSheet("font-weight: bold; font-size: 18px; border: none; background: transparent; color: #000000;")
                position_layout.addWidget(position_label)

                # Dropdown for trolley selection
                combobox = QComboBox()
                combobox.setStyleSheet("border: 8px solid #cccccc; background-color: #f0f0f0; font-size: 18px;")
                combobox.addItems(self.trolley_options)

                # Connect signal to prevent duplicate selections (before setting default)
                combobox.currentIndexChanged.connect(lambda idx, pos=position: self.on_combobox_changed(pos))
                self.position_comboboxes[position] = combobox
                position_layout.addWidget(combobox)

                # Rotation (yaw) dropdown
                rotation_row = QHBoxLayout()
                rotation_row.setSpacing(5)
                rotation_row.setContentsMargins(0, 0, 0, 0)

                rotation_label = QLabel("Rotation")
                rotation_label.setStyleSheet("font-weight: bold; font-size: 14px; border: none; background: transparent; color: #000000;")
                rotation_row.addWidget(rotation_label)

                rotation_combobox = QComboBox()
                rotation_combobox.setStyleSheet("border: 4px solid #cccccc; background-color: #f0f0f0; font-size: 14px;")
                rotation_combobox.addItems(ROTATION_LABELS)
                rotation_combobox.currentTextChanged.connect(
                    lambda text, pos=position: self.on_rotation_changed(pos, text)
                )
                self.position_rotation_comboboxes[position] = rotation_combobox
                rotation_row.addWidget(rotation_combobox)

                position_layout.addLayout(rotation_row)

                position_widget.setLayout(position_layout)
                grid_layout.addWidget(position_widget, row, col)

                position += 1

        # Set default assignments after all comboboxes are created
        # Block signals during initialization
        for combobox in self.position_comboboxes.values():
            combobox.blockSignals(True)
        
        # Set default assignments
        for position, trolley in self.default_assignments.items():
            combobox = self.position_comboboxes[position]
            combobox.setCurrentText(trolley)
        
        # Unblock signals and update comboboxes to prevent duplicates
        for combobox in self.position_comboboxes.values():
            combobox.blockSignals(False)
        
        # Update all comboboxes to reflect current selections
        selected = {cb.currentText() for cb in self.position_comboboxes.values() if cb.currentText() != "-"}
        for combobox in self.position_comboboxes.values():
            current = combobox.currentText()
            combobox.blockSignals(True)
            combobox.clear()
            options = ["-"] + [opt for opt in self.trolley_options[1:] if opt not in selected or opt == current]
            combobox.addItems(options)
            combobox.setCurrentText(current if current in options else "-")
            combobox.blockSignals(False)

        # Populate rotation comboboxes for each position based on its assigned trolley
        for pos in self.position_comboboxes:
            self._sync_rotation_combobox(pos)

        # Button layout
        button_layout = QHBoxLayout()
        button_layout.setSpacing(30)
        # Stretch on both sides centers the two buttons horizontally.
        button_layout.addStretch()

        # Configure button
        configure_button = QPushButton("Configure")
        configure_button.clicked.connect(self.configure_button_clicked)
        configure_button.setStyleSheet("font-size: 18px; padding: 10px; background-color: #cce5ff; color: black; border: none; border-radius: 5px;")
        configure_button.setFixedSize(300, 40)
        button_layout.addWidget(configure_button)

        # Reset button
        reset_button = QPushButton("Reset to Default")
        reset_button.clicked.connect(self.reset_to_default)
        reset_button.setStyleSheet("font-size: 18px; padding: 10px; background-color: #ffcccc; color: black; border: none; border-radius: 5px;")
        reset_button.setFixedSize(300, 40)
        button_layout.addWidget(reset_button)
        button_layout.addStretch()
        
        main_layout.addSpacing(5)
        main_layout.addLayout(button_layout)
        main_layout.addSpacing(10)
        
        # Spawn Objects section: one header, then two columns side by side, each
        # with its own title above a dropdown + Spawn button.
        spawn_objects_label = QLabel("[Spawn Objects]")
        spawn_objects_label.setAlignment(Qt.AlignCenter)
        spawn_objects_label.setStyleSheet("font-weight: bold; font-size: 16px; margin-top: 15px; color: #333333;")
        main_layout.addWidget(spawn_objects_label)

        feeder_combo_style = "font-size: 16px; padding: 6px; max-width: 450px; background-color: #f0f0f0; color: black; border: 1px solid #aaaaaa; border-radius: 5px;"
        spawn_button_style = "font-size: 16px; padding: 10px; background-color: #cce5ff; color: black; border: none; border-radius: 5px;"
        # Same red as the "Reset to Default" button, for Clear/delete actions.
        clear_button_style = "font-size: 16px; padding: 10px; background-color: #ffcccc; color: black; border: none; border-radius: 5px;"
        column_title_style = "font-weight: bold; font-size: 15px; color: #333333;"

        spawn_objects_layout = QHBoxLayout()
        spawn_objects_layout.setSpacing(40)
        spawn_objects_layout.setContentsMargins(0, 5, 0, 0)
        # Stretch on both sides centers the two columns horizontally.
        spawn_objects_layout.addStretch()

        # --- Feeder column ---
        # One button spawns the full feeder load: 7 trays stacked from the top
        # (heater 1-3, elec 1-3, kiro), each with its own components. "Clear"
        # removes whatever was spawned. Re-spawning clears first automatically.
        feeder_column = QVBoxLayout()
        feeder_column.setSpacing(5)

        feeder_title = QLabel("Feeder")
        feeder_title.setAlignment(Qt.AlignCenter)
        feeder_title.setStyleSheet(column_title_style)
        feeder_column.addWidget(feeder_title)

        feeder_row = QHBoxLayout()
        feeder_row.setSpacing(10)

        feeder_spawn_button = QPushButton("Spawn")
        feeder_spawn_button.clicked.connect(self.spawn_all_feeder)
        feeder_spawn_button.setStyleSheet(spawn_button_style)
        feeder_spawn_button.setFixedWidth(180)
        feeder_row.addWidget(feeder_spawn_button)

        feeder_clear_button = QPushButton("Clear")
        feeder_clear_button.clicked.connect(self.delete_feeder_objects)
        feeder_clear_button.setStyleSheet(clear_button_style)
        feeder_clear_button.setFixedWidth(120)
        feeder_row.addWidget(feeder_clear_button)

        feeder_column.addLayout(feeder_row)
        spawn_objects_layout.addLayout(feeder_column)

        # --- Tooltip + Toolchanger column ---
        # Two comboboxes side by side: gripper (rides the UR quickchanger; the
        # other gripper stays on the rack) and tooltip selection. Spawn/Clear
        # buttons sit below. GRIPPER_ENTITY maps gripper labels to SDF/entity names.
        tooltip_column = QVBoxLayout()
        tooltip_column.setSpacing(5)

        tooltip_title = QLabel("Tooltip + Toolchanger")
        tooltip_title.setAlignment(Qt.AlignCenter)
        tooltip_title.setStyleSheet(column_title_style)
        tooltip_column.addWidget(tooltip_title)

        # Combobox row: gripper selector next to tooltip selector.
        combo_row = QHBoxLayout()
        combo_row.setSpacing(10)

        self.gripper_combobox = QComboBox()
        self.gripper_combobox.addItems(list(self.GRIPPER_ENTITY.keys()))
        self.gripper_combobox.setStyleSheet(feeder_combo_style)
        self.gripper_combobox.setFixedWidth(150)
        combo_row.addWidget(self.gripper_combobox)

        self.tooltip_combobox = QComboBox()
        self.tooltip_combobox.addItems(["Tooltip 1", "Tooltip 2", "Tooltip 3"])
        self.tooltip_combobox.setStyleSheet(feeder_combo_style)
        self.tooltip_combobox.setFixedWidth(200)
        combo_row.addWidget(self.tooltip_combobox)

        tooltip_column.addLayout(combo_row)

        # Button row below the comboboxes: Spawn and Clear.
        tooltip_button_row = QHBoxLayout()
        tooltip_button_row.setSpacing(10)

        tooltip_spawn_button = QPushButton("Spawn")
        tooltip_spawn_button.clicked.connect(self.spawn_selected_tooltip)
        tooltip_spawn_button.setStyleSheet(spawn_button_style)
        tooltip_spawn_button.setFixedWidth(180)
        tooltip_button_row.addWidget(tooltip_spawn_button)

        tooltip_clear_button = QPushButton("Clear")
        tooltip_clear_button.clicked.connect(self.delete_tooltips)
        tooltip_clear_button.setStyleSheet(clear_button_style)
        tooltip_clear_button.setFixedWidth(120)
        tooltip_button_row.addWidget(tooltip_clear_button)

        tooltip_column.addLayout(tooltip_button_row)
        spawn_objects_layout.addLayout(tooltip_column)
        spawn_objects_layout.addStretch()

        main_layout.addLayout(spawn_objects_layout)

        # Path to shell scripts
        self.scripts_dir = workspace_root / "src" / "descriptions" / "urdf" / "objects"
        
        # Track spawned objects
        self.tooltip_spawned = None  # None, "tooltip1", "tooltip2", or "tooltip3"
        self.tooltip_gripper_spawned = None  # entity name of gripper mounted on the arm ("krvg"/"koras_2f100")
        self.feeder_spawned = False  # True once the full feeder load is spawned
        
        window.setLayout(main_layout)
        self.show()

    def _load_defaults_from_yaml(self):
        """Populate default_assignments by matching YAML coords to grid positions.
        Trolleys with coords not on the grid (e.g. custom feeder pose) go in custom_trolleys."""
        if not self.yaml_path.exists():
            return
        try:
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            print(f"Error reading {self.yaml_path}: {e}")
            return

        tol = 1e-6
        for trolley, coords in (data.get('trolley_positions') or {}).items():
            if trolley not in self.trolley_options:
                continue
            matched_pos = None
            for pos, grid in self.position_coordinates.items():
                if (abs(coords.get('x', 0) - grid['x']) < tol
                        and abs(coords.get('y', 0) - grid['y']) < tol
                        and abs(coords.get('z', 0) - grid['z']) < tol
                        and pos not in self.default_assignments):
                    matched_pos = pos
                    break
            if matched_pos is not None:
                self.default_assignments[matched_pos] = trolley
            else:
                self.custom_trolleys.add(trolley)

    def on_combobox_changed(self, changed_position):
        """Update all comboboxes to prevent duplicate selections"""

        # Get all selected trolleys ((excluding "-"))
        selected = {cb.currentText() for cb in self.position_comboboxes.values() if cb.currentText() != "-"}

        # Update each combobox
        for combobox in self.position_comboboxes.values():
            current = combobox.currentText()
            combobox.blockSignals(True)
            combobox.clear()

            # add only not selected trolleys and "-"
            options = ["-"] + [opt for opt in self.trolley_options[1:] if opt not in selected or opt == current]
            combobox.addItems(options)
            combobox.setCurrentText(current if current in options else "-")
            combobox.blockSignals(False)

        # Refresh the rotation combobox at the changed position to show the
        # new trolley's stored rotation (or "0"/disabled when "-").
        self._sync_rotation_combobox(changed_position)

    def _sync_rotation_combobox(self, position):
        """Load the rotation of the trolley assigned to `position` into its dropdown.
        Disables the dropdown when no trolley is assigned."""
        rotation_combobox = self.position_rotation_comboboxes.get(position)
        if rotation_combobox is None:
            return
        trolley = self.position_comboboxes[position].currentText()
        if trolley == "-":
            label = "0"
            enabled = False
        else:
            label = self.trolley_rotation.setdefault(trolley, "0")
            enabled = True
        rotation_combobox.blockSignals(True)
        rotation_combobox.setCurrentText(label)
        rotation_combobox.blockSignals(False)
        rotation_combobox.setEnabled(enabled)

    def on_rotation_changed(self, position, label):
        """Persist a rotation-dropdown change into the per-trolley rotation map."""
        if label not in ROTATION_LABEL_TO_RAD:
            return
        trolley = self.position_comboboxes[position].currentText()
        if trolley == "-":
            return
        self.trolley_rotation[trolley] = label

    # configure button clicked
    def configure_button_clicked(self):
        # Collect trolleys assigned to each position
        configured_positions = []
        trolley_assignments = {}
        
        for position, combobox in sorted(self.position_comboboxes.items()):
            selected_trolley = combobox.currentText()
            if selected_trolley != "-":
                configured_positions.append(f"Position {position}: {selected_trolley.capitalize()}")
                trolley_assignments[selected_trolley] = position
        
        # Update trolley_positions.yaml file
        try:
            # Read existing YAML file if it exists
            if self.yaml_path.exists():
                with open(self.yaml_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
            else:
                data = {}
            
            # Initialize trolley_positions if it doesn't exist
            if 'trolley_positions' not in data:
                data['trolley_positions'] = {}
            
            # Get list of all possible trolleys (excluding "-")
            all_trolleys = set(self.trolley_options[1:])  # Exclude "-"
            selected_trolleys = set(trolley_assignments.keys())
            
            # Remove trolleys that are not selected from the YAML
            # Custom trolleys are also removed if not assigned to any grid position
            for trolley in list(data['trolley_positions'].keys()):
                if trolley not in selected_trolleys:
                    del data['trolley_positions'][trolley]
                    self.custom_trolleys.discard(trolley)
            
            # Update positions for assigned trolleys
            for trolley, position in trolley_assignments.items():
                coords = self.position_coordinates[position]
                is_feeder = trolley == 'feeder'
                rotation_label = self.trolley_rotation.get(trolley, "0")
                yaw_rad = ROTATION_LABEL_TO_RAD[rotation_label]
                if is_feeder:
                    dx, dy = FEEDER_OFFSET_BY_LABEL[rotation_label]
                    x = coords['x'] + dx
                    y = coords['y'] + dy
                    z = 0.92
                else:
                    x = coords['x']
                    y = coords['y']
                    z = coords['z']
                data['trolley_positions'][trolley] = {
                    'x': x,
                    'y': y,
                    'z': z,
                    'roll': 0.0,
                    'pitch': 0.0,
                    'yaw': yaw_rad,
                }
            
            # Write updated YAML file
            with open(self.yaml_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            
            print(f"Updated trolley_positions.yaml at {self.yaml_path}")
            
        except Exception as e:
            error_msg = f"Error updating YAML file: {str(e)}"
            print(error_msg)
            QMessageBox.critical(self, "Error", error_msg)
            return

        # Create message
        if configured_positions:
            message = "\n".join(configured_positions)
            if len(configured_positions) <= 0:
                message += "\n\nNeed to configure other positions as well"
                QMessageBox.warning(self, "Not Configured", message)
            else:
                message += "\n\nConfiguration saved to trolley_positions.yaml"
                QMessageBox.information(self, "Configured", "All positions configured" + "\n\n" + message)
        else:
            message = "No trolleys assigned to any position"
            QMessageBox.warning(self, "Not Configured", message)

        # Print to console
        print("Configuration Summary:")
        if configured_positions:
            for config in configured_positions:
                print(f"  - {config}")
        else:
            print("  - No trolleys assigned")

    def reset_to_default(self):
        """Reset all comboboxes to default assignments"""
        # Block signals to prevent duplicate selection checks during reset
        for combobox in self.position_comboboxes.values():
            combobox.blockSignals(True)

        # Reset all comboboxes to "-" first
        for position, combobox in self.position_comboboxes.items():
            combobox.clear()
            combobox.addItems(self.trolley_options)
            combobox.setCurrentText("-")

        # Now set default assignments
        for position, trolley in self.default_assignments.items():
            combobox = self.position_comboboxes[position]
            combobox.setCurrentText(trolley)

        # Unblock signals
        for combobox in self.position_comboboxes.values():
            combobox.blockSignals(False)

        # Update all comboboxes to prevent duplicates (same logic as on_combobox_changed)
        selected = {cb.currentText() for cb in self.position_comboboxes.values() if cb.currentText() != "-"}
        for combobox in self.position_comboboxes.values():
            current = combobox.currentText()
            combobox.blockSignals(True)
            combobox.clear()
            options = ["-"] + [opt for opt in self.trolley_options[1:] if opt not in selected or opt == current]
            combobox.addItems(options)
            combobox.setCurrentText(current if current in options else "-")
            combobox.blockSignals(False)

        # Reset per-trolley rotations and refresh all rotation comboboxes
        self.trolley_rotation = dict(self.default_trolley_rotation)
        for pos in self.position_comboboxes:
            self._sync_rotation_combobox(pos)

        QMessageBox.information(self, "Reset", "All positions reset to default assignments")

    # All entities spawned by the single feeder load: 7 stacked trays
    # (mobile_tray1..7) plus every component that rides on them. Kept in sync with
    # spawn_feeder_all.sh so Clear/re-spawn can remove the whole set.
    FEEDER_TRAYS = [f"mobile_tray{i}" for i in range(1, 8)]
    FEEDER_HEATER_OBJECTS = [
        "heating_plate_cover_1st", "heating_plate_cover_2nd",
        "heating_plate_1st", "heating_plate_2nd",
        "heating_plate_cover3",
    ]
    FEEDER_ELEC_OBJECTS = (
        ["mccb_abe_32b_30a", "pdu_sps25_m66xm4_1", "pdu_sps25_m66xm4_2",
         "noise_filter_rms_2030_din", "plug_socket_drc_220v_16a", "busbar_6p"]
        + [f"single_mc_gmc_{n}" for n in (1, 2, 3)]
        + ["smps_wdr_120_24v"]
        + ["tb_jotn_15a"]
    )
    FEEDER_KIRO_OBJECTS = [
        "relay_part_01", "relay_part_02", "relay_part_02_2",
        "relay_part_03", "relay_part_04", "relay_part_04_2",
    ]

    @property
    def FEEDER_ENTITIES(self):
        return (self.FEEDER_TRAYS + self.FEEDER_HEATER_OBJECTS
                + self.FEEDER_ELEC_OBJECTS + self.FEEDER_KIRO_OBJECTS)

    def _despawn_current_feeder(self):
        """Delete the full feeder load so it can be spawned again cleanly."""
        if not self.feeder_spawned:
            return
        print("Removing current feeder load...")
        self._delete_entities(self.FEEDER_ENTITIES)
        self.feeder_spawned = False

    def delete_feeder_objects(self):
        """Delete the full feeder load if it is currently spawned."""
        if not self.feeder_spawned:
            QMessageBox.information(self, "Nothing to Delete", "No feeder objects are currently spawned.")
            return
        self._despawn_current_feeder()
        QMessageBox.information(self, "Deleted", "All feeder objects deleted.")
        print("Deleted all feeder objects")

    def spawn_all_feeder(self):
        """Spawn the full feeder load (7 trays + components) via spawn_feeder_all.sh.

        If a load is already spawned, remove it first so entity names don't collide,
        then spawn a fresh one."""
        replacing = self.feeder_spawned
        if replacing:
            self._despawn_current_feeder()

        script_path = self.scripts_dir / "spawn_feeder_all.sh"
        if not script_path.exists():
            QMessageBox.critical(self, "Error", f"Script not found: {script_path}")
            return

        try:
            print(f"Running: {script_path}")
            process = subprocess.Popen(
                ['bash', str(script_path)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=str(self.scripts_dir)
            )
            process.wait()  # Wait for all objects to spawn and attach
            self.feeder_spawned = True
            complete_msg = ("Re-spawned all feeder objects (7 trays + components)"
                            if replacing else
                            "All feeder objects spawned successfully (7 trays + components)")
            QMessageBox.information(self, "Spawn Complete", complete_msg)
            print("Spawned Objects: full feeder load")
        except Exception as e:
            error_msg = f"Error running script: {str(e)}"
            print(error_msg)
            QMessageBox.critical(self, "Error", error_msg)

    def delete_tooltips(self):
        """Delete whichever tooltip + toolchanger tool set is currently spawned."""
        if self.tooltip_spawned is None:
            QMessageBox.information(self, "Nothing to Delete", "No tooltips are currently spawned.")
            return
        removed = self.tooltip_spawned
        self._despawn_current_tooltip()
        QMessageBox.information(self, "Deleted", f"All {removed.capitalize()} + Toolchanger Tools deleted.")
        print(f"Deleted tooltip objects: {removed}")

    def spawn_tooltip1(self):
        """Run spawn_ur_tooltip1.sh and spawn_toolchanger_tools.sh"""
        self._spawn_tooltip_and_tools("spawn_ur_tooltip1.sh", "Tooltip 1", "tooltip1")

    def spawn_tooltip2(self):
        """Run spawn_ur_tooltip2.sh and spawn_toolchanger_tools.sh"""
        self._spawn_tooltip_and_tools("spawn_ur_tooltip2.sh", "Tooltip 2", "tooltip2")

    def spawn_tooltip3(self):
        """Run spawn_ur_tooltip3.sh and spawn_toolchanger_tools.sh"""
        self._spawn_tooltip_and_tools("spawn_ur_tooltip3.sh", "Tooltip 3", "tooltip3")

    def spawn_selected_tooltip(self):
        """Dispatch the tooltip dropdown selection to the matching spawn action.
        (Delete is handled by the separate Clear button -> delete_tooltips.)"""
        actions = {
            "Tooltip 1": self.spawn_tooltip1,
            "Tooltip 2": self.spawn_tooltip2,
            "Tooltip 3": self.spawn_tooltip3,
        }
        actions[self.tooltip_combobox.currentText()]()

    # All entities spawned by a tooltip click: the per-tooltip script puts the
    # selected pair (+ krvg) on the UR arm, and spawn_toolchanger_tools.sh spawns
    # the full tool set on the rack. Re-spawning needs a clean slate, so a replace
    # deletes this whole set before spawning the new selection.
    TOOLTIP_ENTITIES = [
        "krvg", "koras_2f100",
        "tooltip_01", "tooltip_01_2",
        "tooltip_02", "tooltip_02_2",
        "tooltip_03", "tooltip_03_2",
    ]

    # Gripper dropdown label -> SDF/entity name. The selected gripper is mounted on
    # the UR quickchanger; the other one stays on the toolchanger rack.
    GRIPPER_ENTITY = {
        "krvg": "krvg",
        "koras": "koras_2f100",
    }

    def _detach_link(self, ur_link, entity_name):
        """Detach an entity's 'link' from the given UR link. Failures are ignored
        (the entity may not be attached)."""
        try:
            subprocess.run(
                ['ros2', 'service', 'call', '/DETACHLINK',
                 'linkattacher_msgs/srv/DetachLink',
                 f"{{model1_name: 'ur', link1_name: '{ur_link}', "
                 f"model2_name: '{entity_name}', link2_name: 'link'}}"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=3
            )
        except Exception as e:
            print(f"Detach {entity_name} from {ur_link} failed/ignored: {e}")

    def _delete_entity(self, entity_name):
        """Delete a spawned entity from Gazebo via the /delete_entity service."""
        try:
            result = subprocess.run(
                ['ros2', 'service', 'call', '/delete_entity',
                 'gazebo_msgs/srv/DeleteEntity', f"{{name: '{entity_name}'}}"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5
            )
            if result.returncode == 0:
                print(f"Deleted entity {entity_name}")
            else:
                print(f"Could not delete {entity_name} (may not exist): {result.stdout}")
        except subprocess.TimeoutExpired:
            print(f"Delete service call timed out for {entity_name}")
        except Exception as e:
            print(f"Error deleting {entity_name}: {e}")

    def _delete_entities(self, entity_names):
        """Delete many entities in one process (batch_delete.py) instead of a
        separate `ros2 service call` per entity, which was slow for large sets."""
        names = list(entity_names)
        if not names:
            return
        script_path = self.scripts_dir / "batch_delete.py"
        try:
            result = subprocess.run(
                ['python3', str(script_path), *names],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=120
            )
            if result.stdout:
                print(result.stdout, end="")
            if result.returncode != 0 and result.stderr:
                print(result.stderr, end="")
        except Exception as e:
            print(f"Batch delete failed: {e}")

    def _detach_links(self, pairs):
        """Detach many (ur_link, entity) pairs from model 'ur' in one process
        (batch_detach.py). Failures are non-fatal."""
        pairs = list(pairs)
        if not pairs:
            return
        args = []
        for ur_link, entity in pairs:
            args += ['--detach', 'ur', ur_link, entity, 'link']
        script_path = self.scripts_dir / "batch_detach.py"
        try:
            result = subprocess.run(
                ['python3', str(script_path), *args],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=60
            )
            if result.stdout:
                print(result.stdout, end="")
        except Exception as e:
            print(f"Batch detach failed/ignored: {e}")

    def _despawn_current_tooltip(self):
        """Detach and delete the currently spawned tooltip/toolchanger tool set so
        a different tooltip can be spawned in its place."""
        if self.tooltip_spawned is None:
            return
        print(f"Removing current tooltip set ({self.tooltip_spawned})...")
        suffix = self.tooltip_spawned[-1]  # "1", "2" or "3"
        # Detach the pair currently held on the toolchanger adapters and the mounted
        # gripper from the quickchanger before deleting, so no dangling attachment
        # joints remain.
        gripper = self.tooltip_gripper_spawned or "krvg"
        self._detach_links([
            ("tooltip_adapter_1_link", f"tooltip_0{suffix}"),
            ("tooltip_adapter_2_link", f"tooltip_0{suffix}_2"),
            ("quickchanger_link", gripper),
        ])
        self._delete_entities(self.TOOLTIP_ENTITIES)
        self.tooltip_spawned = None
        self.tooltip_gripper_spawned = None

    def attach_tooltip_to_adapter(self, tooltip_entity, adapter_link):
        """Attach a tooltip to a tooltip adapter using ROS2 service"""
        try:
            # First, detach from quickchanger_link if already attached (spawn scripts attach to quickchanger_link)
            print(f"Detaching {tooltip_entity} from quickchanger_link (if attached)...")
            detach_call = [
                'ros2', 'service', 'call', '/DETACHLINK',
                'linkattacher_msgs/srv/DetachLink',
                f"{{model1_name: 'ur', link1_name: 'quickchanger_link', model2_name: '{tooltip_entity}', link2_name: 'link'}}"
            ]
            
            detach_result = subprocess.run(
                detach_call,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=3
            )
            # Don't fail if detach fails (might not be attached)
            if detach_result.returncode == 0:
                print(f"Detached {tooltip_entity} from quickchanger_link")
            
            # Brief delay to ensure detach completes before re-attaching
            # (tools have gravity disabled, so no settling is needed).
            time.sleep(0.2)
            
            # Now attach to adapter link
            print(f"Attaching {tooltip_entity} to {adapter_link}...")
            attach_call = [
                'ros2', 'service', 'call', '/ATTACHLINK',
                'linkattacher_msgs/srv/AttachLink',
                f"{{model1_name: 'ur', link1_name: '{adapter_link}', model2_name: '{tooltip_entity}', link2_name: 'link'}}"
            ]
            
            result = subprocess.run(
                attach_call,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0:
                print(f"Successfully attached {tooltip_entity} to {adapter_link}")
                print(f"Attachment stdout: {result.stdout}")
                return True
            else:
                error_msg = f"Failed to attach {tooltip_entity} to {adapter_link}"
                print(error_msg)
                print(f"Return code: {result.returncode}")
                print(f"Stdout: {result.stdout}")
                print(f"Stderr: {result.stderr}")
                print(f"Command: {' '.join(attach_call)}")
                return False
        except subprocess.TimeoutExpired:
            error_msg = f"Attachment service call timed out for {tooltip_entity}"
            print(error_msg)
            return False
        except Exception as e:
            error_msg = f"Error attaching tooltip {tooltip_entity}: {str(e)}"
            print(error_msg)
            return False

    def _spawn_tooltip_and_tools(self, tooltip_script, tooltip_name, tooltip_id):
        """Run tooltip script first, then toolchanger_tools script.

        If the requested tooltip AND gripper are already spawned, do nothing. If a
        different tooltip or gripper is spawned, replace it: remove the current set
        first, then spawn the new selection."""
        # Gripper mounted on the quickchanger, chosen in the gripper dropdown.
        gripper_label = self.gripper_combobox.currentText()
        gripper_entity = self.GRIPPER_ENTITY[gripper_label]

        if self.tooltip_spawned == tooltip_id and self.tooltip_gripper_spawned == gripper_entity:
            QMessageBox.information(
                self, "Already Spawned",
                f"{tooltip_name} with {gripper_label} is already spawned."
            )
            return

        replacing = self.tooltip_spawned is not None
        if replacing:
            previous = self.tooltip_spawned
            self._despawn_current_tooltip()

        tooltip_path = self.scripts_dir / tooltip_script
        tools_path = self.scripts_dir / "spawn_toolchanger_tools.sh"

        if not tooltip_path.exists():
            QMessageBox.critical(self, "Error", f"Script not found: {tooltip_path}")
            return
        
        if not tools_path.exists():
            QMessageBox.critical(self, "Error", f"Script not found: {tools_path}")
            return
        
        try:
            # Run tooltip script first and wait for completion. Pass the chosen
            # gripper entity so it (not the hardcoded krvg) is mounted on the arm.
            print(f"Running: {tooltip_path} {gripper_entity}")
            tooltip_process = subprocess.Popen(
                ['bash', str(tooltip_path), gripper_entity],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=str(self.scripts_dir)
            )
            tooltip_process.wait()  # Wait for tooltip to finish
            
            # Then run toolchanger tools and wait for completion
            print(f"Running: {tools_path}")
            tools_process = subprocess.Popen(
                ['bash', str(tools_path)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=str(self.scripts_dir)
            )
            tools_process.wait()  # Wait for all tools to spawn
            
            # Brief pause to let the spawned tools register before attaching
            # (they have gravity disabled, so no settling is needed).
            print("Preparing to attach tooltips...")
            time.sleep(0.3)
            
            # Set flags only after successful spawn
            self.tooltip_spawned = tooltip_id
            self.tooltip_gripper_spawned = gripper_entity
            print(f"Spawned Objects: {tooltip_name} + {gripper_label} + Toolchanger Tools")

            # Automatically attach tooltips to adapters
            # Determine which tooltips were spawned based on tooltip_name
            if "1" in tooltip_name:
                tooltip1_entity = "tooltip_01"
                tooltip2_entity = "tooltip_01_2"
            elif "2" in tooltip_name:
                tooltip1_entity = "tooltip_02"
                tooltip2_entity = "tooltip_02_2"
            elif "3" in tooltip_name:
                tooltip1_entity = "tooltip_03"
                tooltip2_entity = "tooltip_03_2"
            else:
                return  # Unknown tooltip name

            # Attach first tooltip to adapter 1
            print(f"Attaching {tooltip1_entity} to tooltip_adapter_1_link...")
            success1 = self.attach_tooltip_to_adapter(tooltip1_entity, "tooltip_adapter_1_link")
            if not success1:
                print(f"Warning: Failed to attach {tooltip1_entity} to tooltip_adapter_1_link")
            else:
                print(f"Successfully attached {tooltip1_entity} to tooltip_adapter_1_link")

            # Brief delay between attachments to ensure the first completes
            time.sleep(0.3)

            # Attach second tooltip to adapter 2
            print(f"Attaching {tooltip2_entity} to tooltip_adapter_2_link...")
            success2 = self.attach_tooltip_to_adapter(tooltip2_entity, "tooltip_adapter_2_link")
            if not success2:
                print(f"Warning: Failed to attach {tooltip2_entity} to tooltip_adapter_2_link")
            else:
                print(f"Successfully attached {tooltip2_entity} to tooltip_adapter_2_link")

            # Now that spawning AND attaching are done, report the final result.
            action = "Replaced" if replacing else "Spawned"
            tooltip_desc = f"{tooltip_name} ({gripper_label})"
            prefix = f"{action} {previous.capitalize()} with {tooltip_desc}" if replacing else tooltip_desc
            if success1 and success2:
                QMessageBox.information(
                    self, "Spawn Complete",
                    f"{prefix} + Toolchanger Tools spawned and both tooltips attached to adapters."
                )
            else:
                failed = []
                if not success1:
                    failed.append(f"{tooltip1_entity} → adapter 1")
                if not success2:
                    failed.append(f"{tooltip2_entity} → adapter 2")
                QMessageBox.warning(
                    self, "Spawned, Attach Incomplete",
                    f"{prefix} + Toolchanger Tools spawned, but failed to attach:\n"
                    + "\n".join(failed)
                )
        except Exception as e:
            error_msg = f"Error running scripts: {str(e)}"
            print(error_msg)
            QMessageBox.critical(self, "Error", error_msg)


if __name__ == "__main__":
    app = QApplication([])
    window = MainWindow()
    window.show()
    
    # Handle Ctrl+C to close the GUI
    # Use a list to hold the flag (works around Python closure limitations)
    interrupted = [False]
    
    def signal_handler(signum, frame):
        interrupted[0] = True
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Timer to check for interrupt signal
    def check_interrupt():
        if interrupted[0]:
            app.quit()
    
    timer = QTimer()
    timer.timeout.connect(check_interrupt)
    timer.start(100)  # Check every 100ms
    
    app.exec_()

    