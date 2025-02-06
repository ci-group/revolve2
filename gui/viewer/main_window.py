from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QTabWidget,
      QWidget, QVBoxLayout, QLabel,
        QComboBox, QPushButton, QMessageBox,
          QLineEdit, QHBoxLayout,QStackedWidget)
from parsing import (get_functions_from_file, get_config_parameters_from_file, 
                     save_config_parameters, get_selection_names_from_init, get_files_from_path)
import subprocess


class RobotEvolutionGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Evolution System")
        self.setGeometry(100, 100, 800, 600)
        
        self.simulation_process = None

        self.tab_widget = QTabWidget(self)
        self.setCentralWidget(self.tab_widget)

        self.fitness_functions = get_functions_from_file("../backend_example/fitness_functions.py")
        
        self.terrains = get_functions_from_file("../backend_example/terrains.py")
        
        self.path_simulation_parameters = "../backend_example/config_simulation_parameters.py"
        self.simulation_parameters = get_config_parameters_from_file(self.path_simulation_parameters)
        
        self.path_evolution_parameters = "../backend_example/config.py"
        self.evolution_parameters = get_config_parameters_from_file(self.path_evolution_parameters)
        self.is_generational = self.evolution_parameters.get("GENERATIONAL")
        
        self.selection_functions = get_selection_names_from_init()

        self.database_path = "../resources/databases/"

        # # Step 1: Define Robot Phenotypes
        # self.tab_widget.addTab(self.create_phenotype_tab(), "Robot Phenotypes")

        # Step 2: Define Environment
        self.tab_widget.addTab(self.create_environment_tab(), "Environment & Task")

        # Step 3: Define Genotypes
        self.tab_widget.addTab(self.create_genotype_tab(), "Robot Genotypes")

        # Step 4: Fitness Function
        self.tab_widget.addTab(self.create_fitness_tab(), "Fitness Function")

        # Step 5: Evolution Parameters
        self.tab_widget.addTab(self.create_ea_tab(), "Evolutionary Algorithm")

        # Step 6: Selection
        self.tab_widget.addTab(self.create_selection_tab(), "Selection Algorithms (UNDER DEVELOPMENT)")

        # Step 7: Simulator Selection
        self.tab_widget.addTab(self.create_simulation_parameters_tab(), "Physics Simulator")

        # Step 8: Run Simulation
        self.tab_widget.addTab(self.create_run_simulation_tab(), "Run Simulation")

        self.tab_widget.addTab(self.create_rerun_tab(), "Visualize Best Individual")

        # Step 9: Plot Results
        self.tab_widget.addTab(self.create_plot_tab(), "Plot Results")


    def run_simulation(self):
        # Run the simulation
        self.simulation_process = subprocess.Popen(["python", "../backend_example/main_from_gui.py"])

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
            subprocess.Popen(["python", "../backend_example/plot.py", selected_file])
        else:
            print("No database selected.")

    def rerun(self):
        selected_file = self.database_dropdown_rerun.currentText()
        if selected_file:  # Ensure a file is selected
            subprocess.Popen(["python", "../backend_example/rerun.py", selected_file])
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

    # def create_phenotype_tab(self):
    #     widget = QWidget()
    #     layout = QVBoxLayout()
    #     layout.addWidget(QLabel("Define Robot Phenotypes"))
    #     # Dropdown for phenotypes
    #     phenotypes_dropdown = QComboBox()
    #     phenotypes_dropdown.addItems(["Phenotype A", "Phenotype B", "Phenotype C"])
    #     layout.addWidget(phenotypes_dropdown)
    #     widget.setLayout(layout)
    #     return widget

    def create_genotype_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("UNDER CONSTRUCTION:\n Define Mutation and Crossover Operators"))
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
        layout.addWidget(QLabel("Define Environment and Task"))
        # Environment settings
        layout.addWidget(QLabel("Environment Terrains: "))
        environment_dropdown = QComboBox()  # Example: Environment parameter input
        environment_dropdown.addItems(self.terrains.keys())
        layout.addWidget(environment_dropdown)
        widget.setLayout(layout)
        layout
        # Task selection
        task_dropdown = QComboBox()
        task_dropdown.addItems(["Task 1", "Task 2", "Task 3"])
        layout.addWidget(QLabel("Target Task:"))
        layout.addWidget(task_dropdown)
        widget.setLayout(layout)
        return widget

    def create_fitness_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Define Fitness Function"))
        fitness_dropdown = QComboBox()
        fitness_dropdown.addItems(self.fitness_functions.keys())
        layout.addWidget(fitness_dropdown)
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
