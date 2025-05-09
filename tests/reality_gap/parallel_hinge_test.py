"""
Script for simultaneously controlling both a simulated and physical gecko robot
to compare and analyze the sim-to-real gap.
"""

import socket
import time
import logging
import threading
import tkinter as tk
from tkinter import Scale, Button, Frame, Label, Entry, IntVar, Radiobutton, LabelFrame
from typing import Dict, List, Optional

from pyrr import Vector3

from revolve2.experimentation.logging import setup_logging
from revolve2.modular_robot import ModularRobot, ModularRobotControlInterface
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain import Brain, BrainInstance
from revolve2.modular_robot.sensor_state import ModularRobotSensorState
from revolve2.modular_robot_physical import Config, UUIDKey
from revolve2.modular_robot_physical.remote import run_remote
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.standards import terrains
from revolve2.standards.modular_robots_v2 import gecko_v2
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
            target: Target position in radians (-1.048 to 1.048).
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
            target: Target position in radians (-1.048 to 1.048).
        """
        if self._instance is not None:
            self._instance.set_hinge_target(hinge_index, target)


class ParallelRobotController:
    """Controller for both simulated and physical robots."""

    def __init__(self):
        """Initialize the controller."""
        # Setup logging
        setup_logging()
        logging.getLogger().setLevel(logging.DEBUG)
        
        # Connection settings
        self.ROBOT_IP = "10.15.3.39"
        
        # Create gecko body and get its active hinges
        self.body = gecko_v2()
        self.active_hinges = self.body.find_modules_of_type(ActiveHinge)
        print(f"Found {len(self.active_hinges)} active hinges in the gecko_v2 body")
        
        # Create brains for both robots
        self.sim_brain = ManualControlBrain(self.active_hinges)
        self.phys_brain = ManualControlBrain(self.active_hinges)
        
        # Create robots
        self.sim_robot = ModularRobot(self.body, self.sim_brain)
        self.phys_robot = ModularRobot(self.body, self.phys_brain)
        
        # Physical robot configuration
        self.hinge_mapping = {
            UUIDKey(self.active_hinges[0]): 0,
            UUIDKey(self.active_hinges[1]): 1,
            UUIDKey(self.active_hinges[2]): 2,
            UUIDKey(self.active_hinges[3]): 13,
            UUIDKey(self.active_hinges[4]): 14,
            UUIDKey(self.active_hinges[5]): 15,
        }
        
        self.phys_config = Config(
            modular_robot=self.phys_robot,
            hinge_mapping=self.hinge_mapping,
            run_duration=600,  # 10 minutes
            control_frequency=20,
            initial_hinge_positions={UUIDKey(hinge): 0.0 for hinge in self.active_hinges},
            inverse_servos={},
        )
        
        # Simulation configuration
        self.simulator = LocalSimulator(
            viewer_type="custom",
            headless=False,
        )
        
        self.batch_parameters = make_standard_batch_parameters()
        self.batch_parameters.simulation_time = 600  # 10 minutes
        
        self.scene = ModularRobotScene(terrain=terrains.flat())
        self.scene.add_robot(self.sim_robot)
        
        # Status flags
        self.sim_running = False
        self.phys_running = False
        self.sim_thread = None
        self.phys_thread = None
        
        # Create the UI
        self.create_control_gui()
    
    def on_prepared(self) -> None:
        """Called when the physical robot is prepared."""
        self.phys_running = True
    
    def start_simulation(self):
        """Start the simulation in a separate thread."""
        if not self.sim_running:
            self.sim_running = True
            self.sim_thread = threading.Thread(
                target=self.run_simulation,
                daemon=True
            )
            self.sim_thread.start()
            print("Simulation started")
    
    def run_simulation(self):
        """Run the simulation."""
        simulate_scenes(
            simulator=self.simulator,
            batch_parameters=self.batch_parameters,
            scenes=self.scene,
        )
        self.sim_running = False
    
    def start_physical_robot(self):
        """Start the physical robot in a separate thread."""
        if not self.phys_running:
            self.phys_thread = threading.Thread(
                target=self.run_physical_robot,
                daemon=True
            )
            self.phys_thread.start()
            print("Connecting to physical robot...")
    
    def run_physical_robot(self):
        """Run the physical robot."""
        try:
            run_remote(
                config=self.phys_config,
                hostname=self.ROBOT_IP,
                debug=True,
                on_prepared=self.on_prepared,
                display_camera_view=False,
            )
        except Exception as e:
            print(f"Error during physical robot control: {str(e)}")
            import traceback
            traceback.print_exc()
        finally:
            self.phys_running = False
    
    def create_control_gui(self):
        """Create a GUI for controlling both robots simultaneously."""
        self.root = tk.Tk()
        self.root.title("Parallel Robot Control - Sim-to-Real Comparison")
        self.root.geometry("800x600")
        
        # Configure row and column weights for responsive resizing
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=0)  # Control panel
        self.root.grid_rowconfigure(1, weight=0)  # Unit frame
        self.root.grid_rowconfigure(2, weight=1)  # Hinges frame
        self.root.grid_rowconfigure(3, weight=0)  # Preset frame
        
        # Control panel for starting/stopping robots
        control_panel = LabelFrame(self.root, text="Robot Control")
        control_panel.grid(row=0, column=0, pady=10, padx=10, sticky="ew")
        
        Button(control_panel, text="Start Simulation", 
               command=self.start_simulation).grid(row=0, column=0, padx=10, pady=5)
        
        Button(control_panel, text="Start Physical Robot", 
               command=self.start_physical_robot).grid(row=0, column=1, padx=10, pady=5)
        
        # Add unit selection (radians or degrees)
        unit_frame = Frame(self.root)
        unit_frame.grid(row=1, column=0, pady=10, padx=10, sticky="ew")
        
        Label(unit_frame, text="Units:").pack(side=tk.LEFT)
        
        self.unit_var = IntVar(value=0)  # 0 for radians, 1 for degrees
        
        # Add sliders and entry fields for each hinge
        self.sliders = []
        self.entries = []
        
        # Create a frame and canvas for scrolling
        hinges_frame = Frame(self.root)
        hinges_frame.grid(row=2, column=0, pady=5, padx=10, sticky="nsew")
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
        
        # Track unit changes
        self.unit_var.trace_add("write", self.update_all_displays)
        
        Radiobutton(unit_frame, text="Radians (±1.048)", variable=self.unit_var, 
                    value=0).pack(side=tk.LEFT)
        Radiobutton(unit_frame, text="Degrees (±60°)", variable=self.unit_var, 
                    value=1).pack(side=tk.LEFT)
        
        # Create controls for each hinge
        for i in range(len(self.active_hinges)):
            frame = Frame(scrollable_frame)
            frame.pack(pady=5, fill=tk.X, padx=10)
            
            Label(frame, text=f"Hinge {i}", width=10).pack(side=tk.LEFT)
            
            # Entry field for direct numeric input
            entry = Entry(frame, width=8)
            entry.insert(0, "0.0")
            entry.pack(side=tk.LEFT, padx=5)
            self.entries.append(entry)
            
            # Bind Enter key to update from entry
            entry.bind("<Return>", lambda event, i=i: self.update_slider_from_entry(i))
            entry.bind("<FocusOut>", lambda event, i=i: self.update_slider_from_entry(i))
            
            # Using the actual range of ±1.048 radians
            slider = Scale(frame, from_=-1.048, to=1.048, resolution=0.01, orient=tk.HORIZONTAL, 
                          length=300, command=lambda v, i=i: self.update_entry_from_slider(i))
            slider.set(0)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self.sliders.append(slider)
            
            # Add reset button for this slider
            Button(frame, text="Reset", command=lambda s=slider, e=entry, i=i: [
                s.set(0), 
                self.update_entry_from_slider(i)
            ]).pack(side=tk.LEFT, padx=5)
        
        # Add preset buttons
        preset_frame = Frame(self.root)
        preset_frame.grid(row=3, column=0, pady=10, padx=10, sticky="ew")
        
        Label(preset_frame, text="Presets:").pack(side=tk.LEFT)
        
        Button(preset_frame, text="All Positive", command=self.preset_all_positive).pack(side=tk.LEFT, padx=5)
        Button(preset_frame, text="Alternating", command=self.preset_alternating).pack(side=tk.LEFT, padx=5)
        Button(preset_frame, text="Reset All", command=self.preset_reset_all).pack(side=tk.LEFT, padx=5)
        
        # Make sure the window appears in front
        self.root.lift()
        self.root.attributes('-topmost', True)
        self.root.after_idle(self.root.attributes, '-topmost', False)
        
        # Update the window to ensure proper sizing before mainloop
        self.root.update_idletasks()
    
    def update_slider_from_entry(self, entry_index):
        """Update slider value from the entry field."""
        try:
            entry_value = self.entries[entry_index].get()
            if entry_value.strip() == "":
                return
                
            value = float(entry_value)
            
            # Convert to radians if using degrees
            if self.unit_var.get() == 1:  # Degrees
                # Convert degrees to radians (60 degrees = 1.048 radians)
                radian_value = value * (1.048 / 60.0)
                radian_value = max(min(radian_value, 1.048), -1.048)
            else:  # Radians
                radian_value = max(min(value, 1.048), -1.048)
            
            # Update slider without triggering its callback
            self.sliders[entry_index].set(radian_value)
            # Update both brains
            self.sim_brain.set_hinge_target(entry_index, radian_value)
            self.phys_brain.set_hinge_target(entry_index, radian_value)
        except ValueError:
            self.update_entry_from_slider(entry_index)
    
    def update_entry_from_slider(self, slider_index):
        """Update entry field value from the slider."""
        slider_value = self.sliders[slider_index].get()
        
        # Convert to degrees if using degrees
        if self.unit_var.get() == 1:  # Degrees
            display_value = slider_value * (60.0 / 1.048)
            self.entries[slider_index].delete(0, tk.END)
            self.entries[slider_index].insert(0, f"{display_value:.1f}")
        else:  # Radians
            self.entries[slider_index].delete(0, tk.END)
            self.entries[slider_index].insert(0, f"{slider_value:.3f}")
        
        # Update both brains
        self.sim_brain.set_hinge_target(slider_index, slider_value)
        self.phys_brain.set_hinge_target(slider_index, slider_value)
    
    def update_all_displays(self, *args):
        """Update all displays when unit changes."""
        for i in range(len(self.sliders)):
            self.update_entry_from_slider(i)
    
    def preset_all_positive(self):
        """Set all hinges to maximum positive position."""
        for i, slider in enumerate(self.sliders):
            slider.set(1.048)
            self.update_entry_from_slider(i)
    
    def preset_alternating(self):
        """Set alternating hinges to opposite positions."""
        for i, slider in enumerate(self.sliders):
            value = 1.048 if i % 2 == 0 else -1.048
            slider.set(value)
            self.update_entry_from_slider(i)
    
    def preset_reset_all(self):
        """Reset all hinges to zero position."""
        for i, slider in enumerate(self.sliders):
            slider.set(0)
            self.update_entry_from_slider(i)
    
    def run(self):
        """Run the main application."""
        self.root.mainloop()


def main():
    """Run the parallel robot controller."""
    controller = ParallelRobotController()
    controller.run()


if __name__ == "__main__":
    main()
