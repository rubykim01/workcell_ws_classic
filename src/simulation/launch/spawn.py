from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, 
    QComboBox, QMessageBox, QLabel, QGridLayout
)
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt

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
        self.trolley_options = ["-", "toolchanger", "denso", "ur", "arf", "vision"]
        
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
                combobox.setCurrentText("-")  # Default to "-"
                
                # Connect signal to prevent duplicate selections
                combobox.currentIndexChanged.connect(lambda idx, pos=position: self.on_combobox_changed(pos))
                self.position_comboboxes[position] = combobox
                position_layout.addWidget(combobox)
                
                position_widget.setLayout(position_layout)
                grid_layout.addWidget(position_widget, row, col)
                
                position += 1

        main_layout.addLayout(grid_layout)

        # Configure button
        configure_button = QPushButton("Configure")
        configure_button.clicked.connect(self.configure_button_clicked)
        configure_button.setStyleSheet("font-size: 18px; padding: 10px; background-color: #cccccc; color: black; border: none; border-radius: 5px;")
        main_layout.addWidget(configure_button)
        
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
        for position, combobox in sorted(self.position_comboboxes.items()):
            selected_trolley = combobox.currentText()
            if selected_trolley != "-":
                configured_positions.append(f"Position {position}: {selected_trolley.capitalize()}")
        
        # Create message
        if configured_positions:
            message = "\n".join(configured_positions)
            if len(configured_positions) <= 4:
                message += "\n\nNeed to configure other positions as well"
                QMessageBox.warning(self, "Not Configured", message)
            else:
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


if __name__ == "__main__":
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()

    