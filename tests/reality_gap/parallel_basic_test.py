"""
Script for simultaneously testing distance traveled by both simulated and physical gecko robots
over a specified time period to quantify the sim-to-real gap.
"""

import socket
import time
import math
import logging
import threading
import tkinter as tk
from tkinter import Scale, Button, Frame, Label, Entry, StringVar, LabelFrame
from typing import Dict, List, Optional, Tuple
import numpy as np

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
from revolve2.standards import terrains, fitness_functions
from revolve2.standards.modular_robots_v2 import gecko_v2
from revolve2.standards.simulation_parameters import make_standard_batch_parameters


class DistanceTravelBrainInstance(BrainInstance):
    """Brain instance that implements gecko walking for distance measurement."""

    active_hinges: list[ActiveHinge]
    frequency: float
    amplitude: float
    phase_offset: float
    time_passed: float
    is_active: bool
    
    def __init__(
        self, 
        active_hinges: list[ActiveHinge],
        frequency: float = 1.0,
        amplitude: float = 0.7,
        phase_offset: float = 0.5,
        is_active: bool = False,
    ) -> None:
        """
        Initialize the brain instance with active hinges.

        Args:
            active_hinges: List of active hinges to control.
            frequency: Oscillation frequency in Hz.
            amplitude: Maximum angle amplitude (0.0-1.0).
            phase_offset: Phase offset between adjacent hinges.
            is_active: Whether movement is active.
        """
        self.active_hinges = active_hinges
        self.frequency = frequency
        self.amplitude = amplitude
        self.phase_offset = phase_offset
        self.time_passed = 0.0
        self.is_active = is_active

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        """
        Control the modular robot using the gecko walking pattern.

        Args:
            dt: Elapsed seconds since last call to this function.
            sensor_state: Interface for reading the current sensor state.
            control_interface: Interface for controlling the robot.
        """
        self.time_passed += dt
        
        # Only apply movement patterns if active
        if self.is_active:
            self._apply_gecko_walk(control_interface)
        else:
            # When inactive, set all hinges to neutral position (0)
            for hinge in self.active_hinges:
                control_interface.set_active_hinge_target(hinge, 0.0)
    
    def _apply_gecko_walk(self, control_interface: ModularRobotControlInterface) -> None:
        """
        Apply a specialized walking gait for the gecko robot.
        
        For gecko_v2, the hinge indices are typically:
        - (0, 1): Backbone 
        - (4, 5): (Right, Left) Front Legs 
        - (2, 3): (Right, Left) Back Legs 
        
        This pattern creates a diagonal gait where diagonal legs move together:
        - Front left + Back right 
        - Front right + Back left 
        - Backbone moves in matching pattern
        """
        # Base timing
        t = 2 * math.pi * self.frequency * self.time_passed
        
        # Backbone (0 and 1) moves in matching pattern
        backbone_one = self.amplitude * math.sin(t)
        backbone_two = self.amplitude * math.sin(t) 
        
        # Legs - make extremities on same side move in opposite phases
        # Right side: back (2) and front (4) are opposite
        back_left = self.amplitude * math.sin(t + math.pi * 0.5)  # 90° phase shift
        front_left = self.amplitude * math.sin(t + math.pi * 1.5)  # 270° phase shift (opposite)
        
        # Right side: outer (3) and inner (5) are opposite
        back_right = self.amplitude * math.sin(t + math.pi * 1.5)  # 270° phase shift
        front_right = self.amplitude * math.sin(t + math.pi * 0.5)  # 90° phase shift (opposite)
        
        # Apply the calculated angles to the hinges
        if len(self.active_hinges) >= 6:
            control_interface.set_active_hinge_target(self.active_hinges[0], backbone_one * 1.048)
            control_interface.set_active_hinge_target(self.active_hinges[1], backbone_two * 1.048)
            control_interface.set_active_hinge_target(self.active_hinges[2], back_right * 1.048)
            control_interface.set_active_hinge_target(self.active_hinges[3], back_left * 1.048)
            control_interface.set_active_hinge_target(self.active_hinges[4], front_right * 1.048)
            control_interface.set_active_hinge_target(self.active_hinges[5], front_left * 1.048)
        else:
            # Fallback if we don't have enough hinges
            for i, hinge in enumerate(self.active_hinges):
                angle = self.amplitude * math.sin(t + i * self.phase_offset * math.pi) * 1.048
                control_interface.set_active_hinge_target(hinge, angle)


class DistanceTravelBrain(Brain):
    """Brain that implements gecko walking for distance measurement."""

    active_hinges: list[ActiveHinge]
    frequency: float
    amplitude: float
    phase_offset: float
    is_active: bool
    _instance: DistanceTravelBrainInstance | None
    
    def __init__(
        self, 
        active_hinges: list[ActiveHinge],
        frequency: float = 1.0,
        amplitude: float = 0.7,
        phase_offset: float = 0.5,
        is_active: bool = False,
    ) -> None:
        """
        Initialize the brain with active hinges.

        Args:
            active_hinges: List of active hinges to control.
            frequency: Oscillation frequency in Hz.
            amplitude: Maximum angle amplitude (0.0-1.0).
            phase_offset: Phase offset between adjacent hinges.
            is_active: Whether movement is active.
        """
        self.active_hinges = active_hinges
        self.frequency = frequency
        self.amplitude = amplitude
        self.phase_offset = phase_offset
        self.is_active = is_active
        self._instance = None

    def make_instance(self) -> DistanceTravelBrainInstance:
        """
        Create an instance of this brain.

        Returns:
            The brain instance.
        """
        self._instance = DistanceTravelBrainInstance(
            active_hinges=self.active_hinges,
            frequency=self.frequency,
            amplitude=self.amplitude,
            phase_offset=self.phase_offset,
            is_active=self.is_active,
        )
        return self._instance

    def update_parameters(self, **kwargs) -> None:
        """
        Update the brain parameters.

        Args:
            **kwargs: Parameters to update.
        """
        # Update brain parameters
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
                
                # Also update the instance if it exists
                if self._instance is not None and hasattr(self._instance, key):
                    setattr(self._instance, key, value)


class ParallelDistanceController:
    """Controller for both simulated and physical robots for distance testing."""

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
        self.sim_brain = DistanceTravelBrain(
            active_hinges=self.active_hinges,
            frequency=1.0,
            amplitude=0.7,
            phase_offset=0.5,
            is_active=False,
        )
        
        self.phys_brain = DistanceTravelBrain(
            active_hinges=self.active_hinges,
            frequency=1.0,
            amplitude=0.7,
            phase_offset=0.5,
            is_active=False,
        )
        
        # Create robots
        self.sim_robot = ModularRobot(self.body, self.sim_brain)
        self.phys_robot = ModularRobot(self.body, self.phys_brain)
        
        # Status flags
        self.sim_running = False
        self.phys_running = False
        self.sim_distance = 0.0
        self.phys_distance = 0.0
        
        # Create the UI
        self.create_ui()
    
    def start_simulation(self, duration: float) -> None:
        """
        Start the simulation with the specified duration.
        
        Args:
            duration: The duration of the simulation in seconds.
        """
        if self.sim_running:
            print("Simulation already running")
            return
            
        self.sim_running = True
        self.sim_status_label.config(text="Simulation: Running...", fg="blue")
        
        # Create the scene with a flat terrain
        scene = ModularRobotScene(terrain=terrains.flat())
        scene.add_robot(self.sim_robot)

        # Set up the simulator
        simulator = LocalSimulator(
            viewer_type="custom",
            headless=False,
            manual_control=False,
        )

        # Configure simulation parameters
        batch_parameters = make_standard_batch_parameters()
        batch_parameters.simulation_time = duration

        # Enable movement
        self.sim_brain.update_parameters(is_active=True)
        
        # Run the simulation
        scene_states = simulate_scenes(
            simulator=simulator,
            batch_parameters=batch_parameters,
            scenes=scene,
        )
        
        # Get the state at the beginning and end of the simulation
        initial_state = scene_states[0]
        initial_robot_state = initial_state.get_modular_robot_simulation_state(self.sim_robot)
        final_state = scene_states[-1]
        final_robot_state = final_state.get_modular_robot_simulation_state(self.sim_robot)
        
        # Calculate final distance traveled
        self.sim_distance = fitness_functions.xy_displacement(
            initial_robot_state, final_robot_state
        )
        
        # Update UI
        self.sim_running = False
        self.sim_status_label.config(text=f"Simulation: Completed - {self.sim_distance:.3f}m", fg="green")
        self.update_comparison()
    
    def start_physical_robot(self, duration: float) -> None:
        """
        Start the physical robot with the specified duration.
        
        Args:
            duration: The duration of the robot movement in seconds.
        """
        if self.phys_running:
            print("Physical robot already running")
            return
            
        self.phys_running = True
        self.phys_status_label.config(text="Physical Robot: Connecting...", fg="blue")
        
        # Create hinge mapping for the physical robot
        hinge_mapping = {
            UUIDKey(self.active_hinges[0]): 0,
            UUIDKey(self.active_hinges[1]): 1,
            UUIDKey(self.active_hinges[2]): 2,
            UUIDKey(self.active_hinges[3]): 13,
            UUIDKey(self.active_hinges[4]): 14,
            UUIDKey(self.active_hinges[5]): 15,
        }

        # Configure the robot
        config = Config(
            modular_robot=self.phys_robot,
            hinge_mapping=hinge_mapping,
            run_duration=duration,
            control_frequency=20,
            initial_hinge_positions={UUIDKey(active_hinge): 0.0 for active_hinge in self.active_hinges},
            inverse_servos={},
        )

        try:
            # Update UI
            self.phys_status_label.config(text="Physical Robot: Initializing...", fg="blue")
            
            # Enable movement
            self.phys_brain.update_parameters(is_active=True)
            
            # Run the physical robot
            run_remote(
                config=config,
                hostname=self.ROBOT_IP,
                debug=True,
                on_prepared=self.on_prepared,
                display_camera_view=False,
            )
            
            # After completion, update UI
            self.phys_running = False
            
            # For physical robot, we need to manually measure and input the distance
            self.phys_status_label.config(text="Physical Robot: Completed - Measure distance", fg="green")
            self.show_distance_input_dialog()
            
        except Exception as e:
            self.phys_running = False
            self.phys_status_label.config(text=f"Physical Robot: Error - {str(e)}", fg="red")
            import traceback
            traceback.print_exc()
    
    def on_prepared(self) -> None:
        """Called when the physical robot is prepared."""
        self.phys_status_label.config(text="Physical Robot: Running...", fg="blue")
    
    def show_distance_input_dialog(self) -> None:
        """Show a dialog to input the measured physical distance."""
        dialog = tk.Toplevel(self.root)
        dialog.title("Enter Physical Distance")
        dialog.geometry("300x150")
        dialog.transient(self.root)
        dialog.grab_set()
        
        Label(dialog, text="Enter the measured distance (meters):", pady=10).pack()
        
        distance_var = StringVar()
        distance_entry = Entry(dialog, textvariable=distance_var, width=10)
        distance_entry.pack(pady=10)
        distance_entry.focus_set()
        
        def submit_distance():
            try:
                distance = float(distance_var.get())
                self.phys_distance = distance
                self.phys_status_label.config(text=f"Physical Robot: Completed - {distance:.3f}m", fg="green")
                self.update_comparison()
                dialog.destroy()
            except ValueError:
                Label(dialog, text="Please enter a valid number", fg="red").pack()
        
        Button(dialog, text="Submit", command=submit_distance).pack(pady=10)
    
    def update_comparison(self) -> None:
        """Update the comparison results in the UI."""
        if self.sim_distance > 0 and self.phys_distance > 0:
            ratio = self.phys_distance / self.sim_distance
            difference = self.phys_distance - self.sim_distance
            
            self.comparison_label.config(
                text=f"Comparison: Sim={self.sim_distance:.3f}m, Phys={self.phys_distance:.3f}m\n"
                     f"Difference: {difference:.3f}m, Ratio: {ratio:.2f}",
                fg="blue"
            )
    
    def create_ui(self) -> None:
        """Create the user interface."""
        self.root = tk.Tk()
        self.root.title("Parallel Distance Test")
        self.root.geometry("500x400")
        
        # Configure grid
        self.root.grid_columnconfigure(0, weight=1)
        
        # Duration input
        duration_frame = Frame(self.root)
        duration_frame.grid(row=0, column=0, pady=10, padx=10, sticky="ew")
        
        Label(duration_frame, text="Test Duration (seconds):").pack(side=tk.LEFT)
        
        self.duration_var = StringVar(value="30.0")
        duration_entry = Entry(duration_frame, textvariable=self.duration_var, width=10)
        duration_entry.pack(side=tk.LEFT, padx=10)
        
        # Control buttons
        control_frame = Frame(self.root)
        control_frame.grid(row=1, column=0, pady=10, padx=10, sticky="ew")
        
        # Simulation button
        def start_sim_thread():
            try:
                duration = float(self.duration_var.get())
                if duration <= 0 or duration > 60:
                    raise ValueError("Duration must be between 0 and 60 seconds")
                
                threading.Thread(
                    target=lambda: self.start_simulation(duration),
                    daemon=True
                ).start()
            except ValueError as e:
                self.sim_status_label.config(text=f"Simulation: Error - {str(e)}", fg="red")
        
        sim_button = Button(
            control_frame,
            text="Start Simulation",
            command=start_sim_thread,
            width=20,
            height=2
        )
        sim_button.grid(row=0, column=0, padx=10, pady=5)
        
        # Physical robot button
        def start_phys_thread():
            try:
                duration = float(self.duration_var.get())
                if duration <= 0 or duration > 60:
                    raise ValueError("Duration must be between 0 and 60 seconds")
                
                threading.Thread(
                    target=lambda: self.start_physical_robot(duration),
                    daemon=True
                ).start()
            except ValueError as e:
                self.phys_status_label.config(text=f"Physical Robot: Error - {str(e)}", fg="red")
        
        phys_button = Button(
            control_frame,
            text="Start Physical Robot",
            command=start_phys_thread,
            width=20,
            height=2
        )
        phys_button.grid(row=0, column=1, padx=10, pady=5)
        
        # Status labels
        status_frame = Frame(self.root)
        status_frame.grid(row=2, column=0, pady=10, padx=10, sticky="ew")
        
        self.sim_status_label = Label(status_frame, text="Simulation: Ready", font=("Arial", 12))
        self.sim_status_label.pack(pady=5)
        
        self.phys_status_label = Label(status_frame, text="Physical Robot: Ready", font=("Arial", 12))
        self.phys_status_label.pack(pady=5)
        
        # Comparison results
        comparison_frame = Frame(self.root)
        comparison_frame.grid(row=3, column=0, pady=10, padx=10, sticky="ew")
        
        self.comparison_label = Label(
            comparison_frame, 
            text="Comparison: Run both tests to see results",
            font=("Arial", 12, "bold")
        )
        self.comparison_label.pack(pady=10)
        
        # Instructions
        instructions_frame = Frame(self.root)
        instructions_frame.grid(row=4, column=0, pady=10, padx=10, sticky="ew")
        
        instructions = """
        Instructions:
        1. Enter a test duration (1-60 seconds)
        2. Run the simulation first to see expected distance
        3. Place the physical robot at a starting position and mark it
        4. Run the physical robot test
        5. Measure the actual distance traveled and enter it when prompted
        6. Compare the results to analyze the sim-to-real gap
        """
        
        Label(instructions_frame, text=instructions, justify=tk.LEFT).pack()
        
        # Make sure the window appears in front
        self.root.lift()
        self.root.attributes('-topmost', True)
        self.root.after_idle(self.root.attributes, '-topmost', False)
    
    def run(self):
        """Run the main application."""
        self.root.mainloop()


def main():
    """Run the parallel distance controller."""
    controller = ParallelDistanceController()
    controller.run()


if __name__ == "__main__":
    main()
