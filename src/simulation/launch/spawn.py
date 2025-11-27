from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, 
    QComboBox, QMessageBox, QLabel, QGridLayout
)
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
import yaml
from pathlib import Path

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Configure Spawn Positions")
        
        # Set window size 
        #self.resize(400, 500)
        self.setMinimumSize(700, 600)
        self.setStyleSheet("background-color: #ffffff;")
        # Create central widget 
        window = QWidget()
        self.setCentralWidget(window)

        # Main layout
        main_layout = QVBoxLayout()

        # Trolley options for dropdowns
        self.trolley_options = ["-", "toolchanger", "denso", "ur", "arf", "vision", "feeder"]
        
        # Position to coordinates mapping
        self.position_coordinates = {
            1: {"x": -0.8, "y": 0.0, "z": 0.86},
            2: {"x": -0.8, "y": 0.8, "z": 0.86},
            3: {"x": -0.8, "y": 1.6, "z": 0.86},
            4: {"x": 0.0, "y": 0.0, "z": 0.86},
            5: {"x": 0.0, "y": 0.8, "z": 0.86},
            6: {"x": 0.0, "y": 1.6, "z": 0.86},
            7: {"x": 1.0, "y": 0.0, "z": 0.92},
            8: {"x": 1.0, "y": 0.8, "z": 0.92},
            9: {"x": 1.0, "y": 1.6, "z": 0.92}
        }
        
        # Default trolley assignments
        self.default_assignments = {
            1: "toolchanger",
            2: "denso",
            4: "ur",
            5: "arf",
            6: "vision",
            7: "feeder"
        }
        
        # Path to trolley_positions.yaml
        script_dir = Path(__file__).parent.absolute()
        workspace_root = script_dir.parent.parent.parent
        self.yaml_path = workspace_root / "src" / "descriptions" / "ur_description" / "config" / "trolley_positions.yaml"
        
        # Create 3x3 grid layout
        grid_layout = QGridLayout()
        grid_layout.setSpacing(15)
        
        # Store comboboxes by position number
        self.position_comboboxes = {}
        
        # Create 9 boxes (3x3 grid) for positions 1-9
        position = 1
        for row in range(3):
            for col in range(3):
                # Create container widget for each position
                position_widget = QWidget()
                position_widget.setFixedSize(180, 150)
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

        main_layout.addLayout(grid_layout)

        # Button layout
        button_layout = QVBoxLayout()
        button_layout.setSpacing(10)
        
        # Configure button
        configure_button = QPushButton("Configure")
        configure_button.clicked.connect(self.configure_button_clicked)
        configure_button.setStyleSheet("font-size: 18px; padding: 10px; background-color: #ccffcc; color: black; border: none; border-radius: 5px;")
        button_layout.addWidget(configure_button)
        
        # Reset button
        reset_button = QPushButton("Reset to Default")
        reset_button.clicked.connect(self.reset_to_default)
        reset_button.setStyleSheet("font-size: 18px; padding: 10px; background-color: #ffcccc; color: black; border: none; border-radius: 5px;")
        button_layout.addWidget(reset_button)
        
        main_layout.addLayout(button_layout)
        
        window.setLayout(main_layout)
        self.show()

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
            
            # Update positions for assigned trolleys
            for trolley, position in trolley_assignments.items():
                coords = self.position_coordinates[position]
                data['trolley_positions'][trolley] = {
                    'x': coords['x'],
                    'y': coords['y'],
                    'z': coords['z'],
                    'roll': 0.0,
                    'pitch': 0.0,
                    'yaw': 0.0
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
        
        QMessageBox.information(self, "Reset", "All positions reset to default assignments")


if __name__ == "__main__":
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()

    