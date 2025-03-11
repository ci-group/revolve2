"""Basic test script to assess the sim-to-real gap of the framework."""

from pyrr import Vector3
import tkinter as tk
from tkinter import Scale, Button, Frame, Label, Entry, IntVar, Radiobutton
import threading

from revolve2.experimentation.logging import setup_logging
from revolve2.modular_robot import ModularRobot, ModularRobotControlInterface
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain import Brain, BrainInstance
from revolve2.modular_robot.sensor_state import ModularRobotSensorState
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.standards import terrains
from revolve2.standards.modular_robots_v2 import gecko_v2, ant_v2, spider_v2, snake_v2
from revolve2.standards.simulation_parameters import make_standard_batch_parameters


class ManualControlBrainInstance(BrainInstance):
    """Brain instance that allows manual control of each active hinge."""

    active_hinges: list[ActiveHinge]
    hinge_targets: dict[ActiveHinge, float]

    def __init__(self, active_hinges: list[ActiveHinge]) -> None:
        """
        Initialize the brain instance with active hinges.

        Args:
            active_hinges: List of active hinges to control.
        """
        self.active_hinges = active_hinges
        self.hinge_targets = {hinge: 0.0 for hinge in active_hinges}

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        """
        Control the modular robot using manual targets.

        Args:
            dt: Elapsed seconds since last call to this function.
            sensor_state: Interface for reading the current sensor state.
            control_interface: Interface for controlling the robot.
        """
        # Apply the current target positions to all hinges
        for hinge, target in self.hinge_targets.items():
            control_interface.set_active_hinge_target(hinge, target)

    def set_hinge_target(self, hinge_index: int, target: float) -> None:
        """
        Set the target position for a specific hinge.

        Args:
            hinge_index: Index of the hinge in the active_hinges list.
            target: Target position in radians (-1.0 to 1.0).
        """
        if 0 <= hinge_index < len(self.active_hinges):
            self.hinge_targets[self.active_hinges[hinge_index]] = target


class ManualControlBrain(Brain):
    """Brain that allows manual control of each active hinge."""

    active_hinges: list[ActiveHinge]
    _instance: ManualControlBrainInstance | None

    def __init__(self, active_hinges: list[ActiveHinge]) -> None:
        """
        Initialize the brain with active hinges.

        Args:
            active_hinges: List of active hinges to control.
        """
        self.active_hinges = active_hinges
        self._instance = None

    def make_instance(self) -> BrainInstance:
        """
        Create an instance of this brain.

        Returns:
            The created instance.
        """
        self._instance = ManualControlBrainInstance(self.active_hinges)
        return self._instance

    def set_hinge_target(self, hinge_index: int, target: float) -> None:
        """
        Set the target position for a specific hinge.

        Args:
            hinge_index: Index of the hinge in the active_hinges list.
            target: Target position in radians (-1.0 to 1.0).
        """
        if self._instance is not None:
            self._instance.set_hinge_target(hinge_index, target)


def create_control_gui(brain: ManualControlBrain, num_hinges: int):
    """Create a GUI for controlling the robot's hinges."""
    root = tk.Tk()
    root.title("Robot Hinge Control")
    root.geometry("650x450")  # Width x Height
    
    # Configure row and column weights for responsive resizing
    root.grid_columnconfigure(0, weight=1)
    root.grid_rowconfigure(0, weight=0)  # Unit frame
    root.grid_rowconfigure(1, weight=1)  # Hinges frame
    root.grid_rowconfigure(2, weight=0)  # Preset frame
    
    # Add unit selection (radians or degrees)
    unit_frame = Frame(root)
    unit_frame.grid(row=0, column=0, pady=10, padx=10, sticky="ew")
    
    Label(unit_frame, text="Units:").pack(side=tk.LEFT)
    
    unit_var = IntVar(value=0)  # 0 for radians, 1 for degrees
    
    # Add sliders and entry fields for each hinge
    sliders = []
    entries = []
    
    def update_slider_from_entry(entry_index):
        """Update slider value from the entry field"""
        try:
            entry_value = entries[entry_index].get()
            if entry_value.strip() == "":
                return
                
            value = float(entry_value)
            
            # Convert to radians if using degrees
            if unit_var.get() == 1:  # Degrees
                # Convert degrees to radians (60 degrees = 1.048 radians)
                radian_value = value * (1.048 / 60.0)
                radian_value = max(min(radian_value, 1.048), -1.048)
            else:  # Radians
                radian_value = max(min(value, 1.048), -1.048)
            
            # Update slider without triggering its callback
            sliders[entry_index].set(radian_value)
            # Update brain directly
            brain.set_hinge_target(entry_index, radian_value)
        except ValueError:
            update_entry_from_slider(entry_index)
    
    def update_entry_from_slider(slider_index):
        """Update entry field value from the slider"""
        slider_value = sliders[slider_index].get()
        
        # Convert to degrees if using degrees
        if unit_var.get() == 1:  # Degrees
            display_value = slider_value * (60.0 / 1.048)
            entries[slider_index].delete(0, tk.END)
            entries[slider_index].insert(0, f"{display_value:.1f}")
        else:  # Radians
            entries[slider_index].delete(0, tk.END)
            entries[slider_index].insert(0, f"{slider_value:.3f}")
        
        # Update brain
        brain.set_hinge_target(slider_index, slider_value)
    

    def update_all_displays(*args):
        """Function to update all displays when unit changes"""
        for i in range(len(sliders)):
            update_entry_from_slider(i)
    
    # Track unit changes
    unit_var.trace_add("write", update_all_displays)
    
    Radiobutton(unit_frame, text="Radians (±1.048)", variable=unit_var, 
                value=0).pack(side=tk.LEFT)
    Radiobutton(unit_frame, text="Degrees (±60°)", variable=unit_var, 
                value=1).pack(side=tk.LEFT)
    
    # Create a frame and canvas
    hinges_frame = Frame(root)
    hinges_frame.grid(row=1, column=0, pady=5, padx=10, sticky="nsew")
    canvas = tk.Canvas(hinges_frame)
    scrollbar = tk.Scrollbar(hinges_frame, orient="vertical", command=canvas.yview)
    scrollable_frame = Frame(canvas)
    
    scrollable_frame.bind(
        "<Configure>",
        lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
    )
    
    canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
    canvas.configure(yscrollcommand=scrollbar.set)
    
    # Configure the canvas to expand with the window
    hinges_frame.grid_columnconfigure(0, weight=1)
    hinges_frame.grid_rowconfigure(0, weight=1)
    canvas.grid(row=0, column=0, sticky="nsew")
    scrollbar.grid(row=0, column=1, sticky="ns")
    
    for i in range(num_hinges):
        frame = Frame(scrollable_frame)
        frame.pack(pady=5, fill=tk.X, padx=10)
        
        Label(frame, text=f"Hinge {i}", width=10).pack(side=tk.LEFT)
        
        # Entry field for direct numeric input
        entry = Entry(frame, width=8)
        entry.insert(0, "0.0")
        entry.pack(side=tk.LEFT, padx=5)
        entries.append(entry)
        
        # Bind Enter key to update from entry
        entry.bind("<Return>", lambda event, i=i: update_slider_from_entry(i))
        entry.bind("<FocusOut>", lambda event, i=i: update_slider_from_entry(i))
        
        # Using the actual range of ±1.048 radians
        slider = Scale(frame, from_=-1.048, to=1.048, resolution=0.01, orient=tk.HORIZONTAL, 
                      length=300, command=lambda v, i=i: update_entry_from_slider(i))
        slider.set(0)
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
        sliders.append(slider)
        
        # Add reset button for this slider
        Button(frame, text="Reset", command=lambda s=slider, e=entry, i=i: [
            s.set(0), 
            update_entry_from_slider(i),
            brain.set_hinge_target(i, 0.0)
        ]).pack(side=tk.LEFT, padx=5)
    
    # Add preset buttons
    preset_frame = Frame(root)
    preset_frame.grid(row=2, column=0, pady=10, padx=10, sticky="ew")
    
    Label(preset_frame, text="Presets:").pack(side=tk.LEFT)
    
    # Example preset: all hinges to maximum positive
    def preset_all_positive():
        for i, slider in enumerate(sliders):
            slider.set(1.048)
            update_entry_from_slider(i)
            brain.set_hinge_target(i, 1.048)
    
    # Example preset: alternating hinges
    def preset_alternating():
        for i, slider in enumerate(sliders):
            value = 1.048 if i % 2 == 0 else -1.048
            slider.set(value)
            update_entry_from_slider(i)
            brain.set_hinge_target(i, value)
    
    # Example preset: reset all
    def preset_reset_all():
        for i, slider in enumerate(sliders):
            slider.set(0)
            update_entry_from_slider(i)
            brain.set_hinge_target(i, 0)
    
    Button(preset_frame, text="All Positive", command=preset_all_positive).pack(side=tk.LEFT, padx=5)
    Button(preset_frame, text="Alternating", command=preset_alternating).pack(side=tk.LEFT, padx=5)
    Button(preset_frame, text="Reset All", command=preset_reset_all).pack(side=tk.LEFT, padx=5)
    
    # Make sure the window appears in front
    root.lift()
    root.attributes('-topmost', True)
    root.after_idle(root.attributes, '-topmost', False)
    
    # Update the window to ensure proper sizing before mainloop
    root.update_idletasks()
    
    root.mainloop()


def main() -> None:
    """Run the simulation with manual control."""
    # Setup
    setup_logging()
    body = gecko_v2()

    # Print information about the active hinges for reference
    active_hinges = body.find_modules_of_type(ActiveHinge)
    print(f"Found {len(active_hinges)} active hinges in the gecko_v2 body")
    for i, hinge in enumerate(active_hinges):
        print(f"Hinge {i}: {hinge}")

    # Create robot and brain
    brain = ManualControlBrain(active_hinges)
    robot = ModularRobot(body, brain)

    # Create the scene with a flat terrain
    scene = ModularRobotScene(terrain=terrains.flat())
    scene.add_robot(robot)

    # Set up the simulator with manual control enabled
    simulator = LocalSimulator(
        viewer_type="custom",  
        headless=False,
        manual_control=True,   
    )

    # Configure simulation parameters
    batch_parameters = make_standard_batch_parameters()
    batch_parameters.simulation_time = 60  

    # Start GUI in a separate thread
    threading.Thread(target=lambda: create_control_gui(brain, len(active_hinges)), daemon=True).start()

    # Run the sim
    simulate_scenes(
        simulator=simulator,
        batch_parameters=batch_parameters,
        scenes=scene,
    )


if __name__ == "__main__":
    main()
