from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QTabWidget,
      QWidget, QVBoxLayout, QLabel, 
        QComboBox, QPushButton, QMessageBox,
          QLineEdit, QHBoxLayout,QStackedWidget,
            QListWidget, QListWidgetItem)
from PyQt5.QtGui import QIcon, QPixmap
from gui.viewer.parsing import (get_functions_from_file, get_config_parameters_from_file, 
                     save_config_parameters, get_selection_names_from_init, get_files_from_path)
import subprocess
import os
import sys

class RobotEvolutionGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Evolution System")
        self.setGeometry(100, 100, 800, 600)
        
        self.simulation_process = None

        self.tab_widget = QTabWidget(self)
        self.setCentralWidget(self.tab_widget)

        # Get the absolute path to the project root (revolve2/)
        self.PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))

        # Use absolute paths
        self.fitness_functions = get_functions_from_file(os.path.join(self.PROJECT_ROOT, "gui/backend_example/fitness_functions.py"))
        self.terrains = get_functions_from_file(os.path.join(self.PROJECT_ROOT, "gui/backend_example/terrains.py"))

        self.path_simulation_parameters = os.path.join(self.PROJECT_ROOT, "gui/backend_example/config_simulation_parameters.py")
        self.simulation_parameters = get_config_parameters_from_file(self.path_simulation_parameters)

        self.path_evolution_parameters = os.path.join(self.PROJECT_ROOT, "gui/backend_example/config.py")
        self.evolution_parameters = get_config_parameters_from_file(self.path_evolution_parameters)
        self.is_generational = self.evolution_parameters.get("GENERATIONAL")

        self.selection_functions = get_selection_names_from_init()

        self.database_path = os.path.join(self.PROJECT_ROOT, "gui/resources/databases/")


        if not os.path.exists(self.database_path):
            os.makedirs(self.database_path)
            print(f"Folder '{self.database_path}' created successfully.")
        else:
            pass

        self.tab_widget.addTab(self.create_environment_tab(), "Environment")

        self.tab_widget.addTab(self.create_genotype_tab(), "Robot Genotypes")

        self.tab_widget.addTab(self.create_fitness_tab(), "Task and Fitness Function")

        self.tab_widget.addTab(self.create_ea_tab(), "Evolutionary Algorithm")

        self.tab_widget.addTab(self.create_selection_tab(), "Selection Algorithms (UNDER DEVELOPMENT)")

        self.tab_widget.addTab(self.create_simulation_parameters_tab(), "Physics Simulator")

        self.tab_widget.addTab(self.create_run_simulation_tab(), "Run Simulation")

        self.tab_widget.addTab(self.create_rerun_tab(), "Visualize Best Individual")

        self.tab_widget.addTab(self.create_plot_tab(), "Plot Results")


    def run_simulation(self):
        current_terrain = self.environment_list.currentItem()
        if current_terrain:
            terrain = current_terrain.text()
        else:
            terrain = "flat"
        fitness_function = self.fitness_dropdown.currentText()
        self.simulation_process = subprocess.Popen(["python", "gui/backend_example/main_from_gui.py", terrain, fitness_function])

    def stop_simulation(self):
        """Stop the running simulation."""
        if self.simulation_process and self.simulation_process.poll() is None:
            self.simulation_process.terminate()  
            self.simulation_process.wait()
            print("Simulation stopped successfully.")
        else:
            print("No active simulation to stop.")

    def plot_results(self):
        selected_file = self.database_dropdown_plot.currentText()
        if selected_file:  # Ensure a file is selected
            subprocess.Popen(["python", "gui/backend_example/plot.py", selected_file])
            QMessageBox.information(self, "Success", "Figure saved to 'gui/resources/figures/'")
        else:
            print("No database selected.")

    def rerun(self):
        selected_file = self.database_dropdown_rerun.currentText()
        if selected_file:  # Ensure a file is selected
            subprocess.Popen(["python", "gui/backend_example/rerun.py", selected_file])
        else:
            print("No database selected.")

    def save_config_changes(self, file_path, inputs):
        """Update a config with new values from the GUI."""
        new_values = {}
        for key, input_field in inputs.items():
            if type(input_field) == bool:
                new_values[key] = input_field
                continue
            else:
                text_value = input_field.text()

            try:
                new_values[key] = eval(text_value)  # Careful with eval in untrusted inputs!
            except:
                new_values[key] = text_value

        save_config_parameters(file_path, new_values)

        QMessageBox.information(self, "Success", "Config file updated!")

    def create_genotype_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("<b>UNDER DEVELOPMENT</b> - \n Define Mutation and Crossover Operators"))
        # Mutation operator
        mutation_dropdown = QComboBox()
        mutation_dropdown.addItems(["Operator A", "Operator B", "Operator C"])
        layout.addWidget(QLabel("Mutation Operator:"))
        layout.addWidget(mutation_dropdown)
        # Crossover operator
        crossover_dropdown = QComboBox()
        crossover_dropdown.addItems(["Operator X", "Operator Y", "Operator Z"])
        layout.addWidget(QLabel("Crossover Operator:"))
        layout.addWidget(crossover_dropdown)
        widget.setLayout(layout)
        return widget
    
    def create_selection_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("<b>UNDER DEVELOPMENT<b>"))
        layout.addWidget(QLabel("Define Parent and Surivor Selection Types"))
        parent_dropdown = QComboBox()
        parent_dropdown.addItems(self.selection_functions)
        layout.addWidget(QLabel("Parent Selection: "))
        layout.addWidget(parent_dropdown)
        # Crossover operator
        survivor_dropdown = QComboBox()
        survivor_dropdown.addItems(self.selection_functions)
        layout.addWidget(QLabel("Survivor Selection:"))
        layout.addWidget(survivor_dropdown)
        widget.setLayout(layout)
        return widget

    def create_ea_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()

        # Stacked widget for toggling between views
        self.stacked_widget = QStackedWidget()

        # Generational View
        self.view1 = QWidget()
        v1_layout = QVBoxLayout()
        v1_layout.addWidget(QLabel("<b>Non-Overlapping (comma) Generations</b>"))
        self.view1.setLayout(v1_layout)

        # Steady-State View
        self.view2 = QWidget()
        v2_layout = QVBoxLayout()
        v2_layout.addWidget(QLabel("<b>Overlapping (plus) Generations</b>"))
        self.view2.setLayout(v2_layout)

        self.inputs_evolution = {}

        # Add parameter fields for generational views
        for key, value in self.evolution_parameters.items():
            input_layout = QHBoxLayout()
            input_label = QLabel(f"{key}:")
            input_field = QLineEdit(str(value))
            self.inputs_evolution[key] = input_field
            if key in ["GENERATIONAL", "STEADY_STATE"]:
                self.inputs_evolution["GENERATIONAL"] = True
                self.inputs_evolution["STEADY_STATE"] = False
                continue
            else:
                input_layout.addWidget(input_label)
                input_layout.addWidget(input_field)
            v1_layout.addLayout(input_layout)

        # Add parameter fields for steady_state view
        for key, value in self.evolution_parameters.items():
            input_layout = QHBoxLayout()
            input_label = QLabel(f"{key}:")
            input_field = QLineEdit(str(value))
            self.inputs_evolution[key] = input_field
            if key in ["GENERATIONAL", "STEADY_STATE"]:  
                self.inputs_evolution["GENERATIONAL"] = False
                self.inputs_evolution["STEADY_STATE"] = True
                continue
            else:
                input_layout.addWidget(input_label)
                input_layout.addWidget(input_field)
            v2_layout.addLayout(input_layout)

        self.stacked_widget.addWidget(self.view1)
        self.stacked_widget.addWidget(self.view2)

        # Button to switch modes
        self.switch_button = QPushButton("")
        self.switch_button.clicked.connect(self.toggle_view)
        layout.addWidget(self.switch_button)
        layout.addWidget(self.stacked_widget)

        # Save Button
        save_button = QPushButton("Save Changes")
        save_button.clicked.connect(lambda: self.save_config_changes(self.path_evolution_parameters, self.inputs_evolution))
        layout.addWidget(save_button)

        widget.setLayout(layout)

        # Set initial state
        self.update_view()
        
        return widget

    def toggle_view(self):
        """Toggle between Generational and Steady-State mode."""
        self.is_generational = not self.is_generational

        # Save the updated mode to config
        self.inputs_evolution["GENERATIONAL"] = self.is_generational
        self.inputs_evolution["STEADY_STATE"] = not self.is_generational

        # Update UI
        self.update_view()

    def update_view(self):
        """Update the UI based on the current mode."""
        if self.is_generational:
            self.stacked_widget.setCurrentWidget(self.view1)
            self.switch_button.setText("Switch to Overlapping")
        else:
            self.stacked_widget.setCurrentWidget(self.view2)
            self.switch_button.setText("Switch to Non-Overlapping")

    def create_environment_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("<b>UNDER DEVELOPMENT</b> - Define Environment"))
        # Environment settings
        layout.addWidget(QLabel("Environment Terrains: "))

        # Create a horizontal layout for the list and image
        h_layout = QHBoxLayout()

        self.environment_list = QListWidget()  # Example: Environment parameter input

        # Path to the directory containing terrain images
        terrain_images_path = os.path.join(self.PROJECT_ROOT, "gui/resources/terrain_images")

        # Add items with icons to the list
        for terrain_name in self.terrains.keys():
            icon_path = os.path.join(terrain_images_path, f"{terrain_name}.png")
            item = QListWidgetItem(terrain_name)
            if os.path.exists(icon_path):
                icon = QIcon(QPixmap(icon_path).scaled(32, 32, aspectRatioMode=1))
                item.setIcon(icon)
            self.environment_list.addItem(item)

        h_layout.addWidget(self.environment_list, 1)  # Set stretch factor for the list

        # QLabel to display the larger image
        self.terrain_image_label = QLabel()
        h_layout.addWidget(self.terrain_image_label, 3)  # Set higher stretch factor for the image

        # Add the horizontal layout to the main layout
        layout.addLayout(h_layout)

        # Connect the list to the update function
        self.environment_list.itemClicked.connect(self.update_terrain_image)

        widget.setLayout(layout)

        # Set the initial image to the first item in the list
        if self.environment_list.count() > 0:
            self.environment_list.setCurrentRow(0)
            self.update_terrain_image(self.environment_list.item(0))

        return widget
    
    def update_terrain_image(self, item):
        terrain_name = item.text()
        terrain_images_path = os.path.join(self.PROJECT_ROOT, "gui/resources/terrain_images")
        image_path = os.path.join(terrain_images_path, f"{terrain_name}.png")
        if os.path.exists(image_path):
            pixmap = QPixmap(image_path).scaled(self.terrain_image_label.size(), aspectRatioMode=1)
            self.terrain_image_label.setPixmap(pixmap)
        else:
            self.terrain_image_label.clear()

    def create_fitness_tab(self):  # under development
        widget = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("<b>UNDER DEVELOPMENT</b> - Define Task and Fitness Function"))

        # Task selection
        task_dropdown = QComboBox()
        task_dropdown.addItems(["Task 1", "Task 2", "Task 3"])
        layout.addWidget(QLabel("Target Task:"))
        layout.addWidget(task_dropdown)

        # Fitness function selection
        self.fitness_dropdown = QComboBox()
        self.fitness_dropdown.addItems(self.fitness_functions.keys())
        layout.addWidget(QLabel("Fitness Function:"))
        layout.addWidget(self.fitness_dropdown)

        widget.setLayout(layout)
        return widget

    def create_simulation_parameters_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Edit Simulation Parameters"))
        self.inputs_simulation = {}
        for key, value in self.simulation_parameters.items():
            input_layout = QHBoxLayout()
            input_label = QLabel(f"{key}:")
            input_field = QLineEdit(str(value))
            self.inputs_simulation[key] = input_field
            input_layout.addWidget(input_label)
            input_layout.addWidget(input_field)
            layout.addLayout(input_layout)
        
        save_button = QPushButton("Save Changes")
        save_button.clicked.connect(lambda: self.save_config_changes(self.path_simulation_parameters, self.inputs_simulation))

        layout.addWidget(save_button)
        widget.setLayout(layout)
        return widget

    def create_run_simulation_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Run Simulation"))
        run_button = QPushButton("Run Simulation")
        run_button.clicked.connect(self.run_simulation)
        layout.addWidget(run_button)
        widget.setLayout(layout)

        layout.addWidget(QLabel("Stop Simulation"))
        stop_button = QPushButton("Stop Simulation")
        stop_button.clicked.connect(self.stop_simulation)
        layout.addWidget(stop_button)
        widget.setLayout(layout)
        return widget
        
    def create_plot_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        input_layout = QHBoxLayout()
        
        label = QLabel("Plot Results from Database:")
        self.database_dropdown_plot = QComboBox()
        self.database_dropdown_plot.addItems(get_files_from_path(self.database_path))

        input_layout.addWidget(label)
        input_layout.addWidget(self.database_dropdown_plot)

        input_layout.setStretch(0, 1)  # Label takes less space
        input_layout.setStretch(1, 3)  # Dropdown takes more space

        plot_button = QPushButton("Plot Results")
        plot_button.clicked.connect(self.plot_results)

        layout.addLayout(input_layout)
        layout.addWidget(plot_button)

        widget.setLayout(layout)
        return widget
    
    def create_rerun_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        input_layout = QHBoxLayout()
        
        label = QLabel("Visualize Best Individual from Database:")
        self.database_dropdown_rerun = QComboBox()
        self.database_dropdown_rerun.addItems(get_files_from_path(self.database_path))

        input_layout.addWidget(label)
        input_layout.addWidget(self.database_dropdown_rerun)

        input_layout.setStretch(0, 1)  # Label takes less space
        input_layout.setStretch(1, 3)  # Dropdown takes more space

        plot_button = QPushButton("Visualize")
        plot_button.clicked.connect(self.rerun)

        layout.addLayout(input_layout)
        layout.addWidget(plot_button)

        widget.setLayout(layout)
        return widget

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    window = RobotEvolutionGUI()
    window.show()
    sys.exit(app.exec_())
