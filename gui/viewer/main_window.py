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
import signal
import time

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

        self.tab_widget.addTab(self.create_fitness_tab(), "Task and Fitness Function")
        
        self.tab_widget.addTab(self.create_genotype_tab(), "Robot Genotypes")

        self.tab_widget.addTab(self.create_ea_tab(), "Evolutionary Algorithm")

        self.tab_widget.addTab(self.create_selection_tab(), "Selection Algorithms (UNDER DEVELOPMENT)")

        self.tab_widget.addTab(self.create_simulation_parameters_tab(), "Physics Simulator")

        self.tab_widget.addTab(self.create_run_simulation_tab(), "Run Simulation")

        self.tab_widget.addTab(self.create_rerun_tab(), "Visualize Best Individual")

        self.tab_widget.addTab(self.create_plot_tab(), "Plot Results")


    def run_simulation(self):        
        fitness_function = self.fitness_dropdown.currentText()
        terrain = self.gather_terrain_params()
        parent_selection, parent_selection_params = self.gather_selection_params(self.parent_dropdown, self.parent_params_layout)
        survival_selection, survival_selection_params = self.gather_selection_params(self.survivor_dropdown, self.survivor_params_layout)

        command = f"python gui/backend_example/main_from_gui.py {terrain} {fitness_function} {parent_selection} {parent_selection_params} {survival_selection} {survival_selection_params}"
        print(f"Running simulation with command: {command}")
        self.simulation_process = subprocess.Popen(command, shell=True)


    def stop_simulation(self):
        """Stop the running simulation."""
        if self.simulation_process and self.simulation_process.poll() is None:
            try:
                # On Windows
                if os.name == 'nt':
                    # Forcefully terminate the process tree
                    subprocess.run(['taskkill', '/F', '/T', '/PID', str(self.simulation_process.pid)], 
                                stderr=subprocess.PIPE, stdout=subprocess.PIPE)
                # On Unix/Linux/Mac
                else:
                    # Try to kill the process group
                    os.killpg(os.getpgid(self.simulation_process.pid), signal.SIGTERM)
                    # Give it a moment to terminate
                    time.sleep(0.5)
                    # If it's still running, force kill
                    if self.simulation_process.poll() is None:
                        os.killpg(os.getpgid(self.simulation_process.pid), signal.SIGKILL)
                
                # Wait with timeout to avoid hanging
                try:
                    self.simulation_process.wait(timeout=2)
                    print("Simulation stopped successfully.")
                except subprocess.TimeoutExpired:
                    print("Simulation process did not terminate within timeout, but was forcefully killed.")
                    
            except Exception as e:
                print(f"Error stopping simulation: {e}")
                # Try one more time with basic terminate
                try:
                    self.simulation_process.terminate()
                    self.simulation_process.wait(timeout=1)
                    print("Simulation stopped using fallback method.")
                except:
                    print("Failed to stop simulation process. It may still be running.")
                    
            # Reset the process reference regardless of outcome
            self.simulation_process = None
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
        
        self.parent_dropdown = QComboBox()
        self.parent_dropdown.addItems(self.selection_functions)
        layout.addWidget(QLabel("Parent Selection: "))
        layout.addWidget(self.parent_dropdown)

        # Parent selection parameters
        self.parent_params_layout = QVBoxLayout()
        layout.addLayout(self.parent_params_layout)
        
        # Connect parent dropdown to update function
        self.parent_dropdown.currentIndexChanged.connect(
            lambda: self.update_selection_params(self.parent_dropdown, self.parent_params_layout)
            )
        
        self.survivor_dropdown = QComboBox()
        self.survivor_dropdown.addItems(self.selection_functions)
        layout.addWidget(QLabel("Survivor Selection:"))
        layout.addWidget(self.survivor_dropdown)
        
        # Survivor selection parameters
        self.survivor_params_layout = QVBoxLayout()
        layout.addLayout(self.survivor_params_layout)
        
        # Connect survivor dropdown to update function
        self.survivor_dropdown.currentIndexChanged.connect(
            lambda: self.update_selection_params(self.survivor_dropdown, self.survivor_params_layout)
            )

        widget.setLayout(layout)

        return widget
    
    def update_selection_params(self, item, params_layout):
        """Update the parameter input fields based on the selected function."""
        if item is None:
            return
        
        # Clear existing parameter input fields and layouts
        while params_layout.count():
            layout_item = params_layout.takeAt(0)
            
            # If the item is a widget
            if layout_item.widget():
                layout_item.widget().deleteLater()
            # If the item is a layout
            elif layout_item.layout():
                # Clear the child layout
                child_layout = layout_item.layout()
                while child_layout.count():
                    child_item = child_layout.takeAt(0)
                    if child_item.widget():
                        child_item.widget().deleteLater()
                # Now we can delete the layout
                child_layout.deleteLater()
        
        selection_params = {
            "tournament": {"k" : 2}
        }

        selected_function = item.currentText()
        if selected_function in selection_params:
            for param, value in selection_params[selected_function].items():
                input_layout = QHBoxLayout()
                input_label = QLabel(f"{param}:")
                input_field = QLineEdit(str(value))
                input_layout.addWidget(input_label)
                input_layout.addWidget(input_field)
                params_layout.addLayout(input_layout)

    def gather_selection_params(self, selection_dropdown, selection_params_layout):
        """Gather selection parameters from the GUI."""
        current_selection = selection_dropdown.currentText()
        if current_selection:
            selection_params = {}
            
            # We need to iterate through all layouts in the selection_params_layout
            for i in range(selection_params_layout.count()):
                layout_item = selection_params_layout.itemAt(i)
                
                # If the item is a layout (which it should be based on your update_selection_params method)
                if layout_item.layout():
                    param_layout = layout_item.layout()
                    # First widget is label, second is the input field
                    if param_layout.count() >= 2:
                        label_item = param_layout.itemAt(0)
                        input_item = param_layout.itemAt(1)
                        
                        if label_item and label_item.widget() and input_item and input_item.widget():
                            label = label_item.widget()
                            input_field = input_item.widget()
                            
                            # Extract parameter name from label (remove the ":" at the end)
                            param_name = label.text().rstrip(":")
                            
                            # Get parameter value from QLineEdit
                            if isinstance(input_field, QLineEdit):
                                param_value = input_field.text()
                                selection_params[param_name] = param_value
            
            selection = f'"{current_selection}"'
            # Convert selection parameters to a string format
            selection_params_str = str(",".join([f"{key}={value}" for key, value in selection_params.items()]))
            selection_params_str = f'"{selection_params_str}"'

            return selection, selection_params_str

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

        # Terrain parameters layout
        self.terrain_params_layout = QVBoxLayout()
        layout.addLayout(self.terrain_params_layout)

        # Connect the list to the update function
        self.environment_list.currentItemChanged.connect(lambda: self.update_terrain_image(self.environment_list.currentItem()))
        self.environment_list.currentItemChanged.connect(lambda: self.update_terrain_params(self.environment_list.currentItem(), self.terrain_params_layout))
        widget.setLayout(layout)

        # Set the initial image to the first item in the list
        if self.environment_list.count() > 0:
            self.environment_list.setCurrentRow(0)
            self.update_terrain_image(self.environment_list.item(0))
            self.update_terrain_params(self.environment_list.item(0), self.terrain_params_layout)
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

    def update_terrain_params(self, item, params_layout):
        """Update the parameter input fields based on the selected terrain."""
        if item is None:
            return
        
        # Clear existing parameter input fields and layouts
        while params_layout.count():
            layout_item = params_layout.takeAt(0)
            
            # If the item is a widget
            if layout_item.widget():
                layout_item.widget().deleteLater()
            # If the item is a layout
            elif layout_item.layout():
                # Clear the child layout
                child_layout = layout_item.layout()
                while child_layout.count():
                    child_item = child_layout.takeAt(0)
                    if child_item.widget():
                        child_item.widget().deleteLater()
                # Now we can delete the layout
                child_layout.deleteLater()

        terrain_params = {
            "flat": {
                "size": [20, 20]
            },
            "crater": {
                "size": [20, 20],
                "ruggedness": 0.3,
                "curviness": 5,
                "granularity_multiplier": 0.5
            },
            "rugged_heightmap": {
                "size": [20, 20],
                "num_edges": 100,
                "density": 0.5,
                "hillyness": 0.5
            },
            "bowl_heightmap": {
                "size": [20, 20],
                "granularity_multiplier": 0.5
            },
            # Add more terrains and their parameters here
        }
        
        selected_terrain = item.text()
        if selected_terrain in terrain_params:
            for param, value in terrain_params[selected_terrain].items():
                input_layout = QHBoxLayout()
                input_label = QLabel(f"{param}:")
                input_field = QLineEdit(str(value))
                input_layout.addWidget(input_label)
                input_layout.addWidget(input_field)
                params_layout.addLayout(input_layout)

    def gather_terrain_params(self):
        """Gather terrain parameters from the GUI."""
        current_terrain = self.environment_list.currentItem()
        if current_terrain:
            terrain = current_terrain.text()
        else:
            terrain = "flat"
        
        # Collect terrain parameters
        terrain_params = {}
        
        # We need to iterate through all layouts in the terrain_params_layout
        for i in range(self.terrain_params_layout.count()):
            layout_item = self.terrain_params_layout.itemAt(i)
            
            # If the item is a layout (which it should be based on your update_terrain_params method)
            if layout_item.layout():
                param_layout = layout_item.layout()
                # First widget is label, second is the input field
                if param_layout.count() >= 2:
                    label_item = param_layout.itemAt(0)
                    input_item = param_layout.itemAt(1)
                    
                    if label_item and label_item.widget() and input_item and input_item.widget():
                        label = label_item.widget()
                        input_field = input_item.widget()
                        
                        # Extract parameter name from label (remove the ":" at the end)
                        param_name = label.text().rstrip(":")
                        
                        # Get parameter value from QLineEdit
                        if isinstance(input_field, QLineEdit):
                            param_value = input_field.text()
                            terrain_params[param_name] = param_value
        
        # Convert terrain parameters to a string format
        terrain_params_str = str(",".join([f"{key}={value}" for key, value in terrain_params.items()]))

        terrain = f'"{terrain}({terrain_params_str})"'

        return terrain

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
