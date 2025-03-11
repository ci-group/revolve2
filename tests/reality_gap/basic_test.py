"""Test script to measure robot travel distance over a specified time."""

import time
import math
import numpy as np
import tkinter as tk
from tkinter import Frame, Label, Button, Entry, StringVar
import threading
import sys

from revolve2.experimentation.logging import setup_logging
from revolve2.modular_robot import ModularRobot, ModularRobotControlInterface
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain import Brain, BrainInstance
from revolve2.modular_robot.sensor_state import ModularRobotSensorState
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.standards import terrains, fitness_functions
from revolve2.standards.modular_robots_v2 import gecko_v2
from revolve2.standards.simulation_parameters import make_standard_batch_parameters

# Global variables for tracking robot state and distance
distance_traveled = 0.0

# Global variable for simulation duration
simulation_duration = 30.0  # Default value


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


def run_simulation(duration: float) -> None:
    """
    Run the simulation with the specified duration.
    
    Args:
        duration: The duration of the simulation in seconds.
    """
    # Setup
    setup_logging()
    body = gecko_v2()

    # Find all active hinges in the body
    active_hinges = body.find_modules_of_type(ActiveHinge)
    
    # Print information about the active hinges for reference
    print(f"Found {len(active_hinges)} active hinges in the gecko body")
    for i, hinge in enumerate(active_hinges):
        print(f"Hinge {i}: {hinge}")

    # Create the brain with movement enabled from the start
    brain = DistanceTravelBrain(
        active_hinges=active_hinges,
        frequency=1.0,
        amplitude=0.7,
        phase_offset=0.5,
        is_active=True,  # Start with movement enabled
    )
    
    # Create the modular robot
    robot = ModularRobot(body, brain)

    # Create the scene with a flat terrain
    scene = ModularRobotScene(terrain=terrains.flat())
    scene.add_robot(robot)

    # Set up the simulator
    simulator = LocalSimulator(
        viewer_type="custom",  # Use custom viewer for better visualization
        headless=False,
        manual_control=False,  # Disable manual control as we're using programmed movements
    )

    # Configure simulation parameters
    batch_parameters = make_standard_batch_parameters()
    batch_parameters.simulation_time = duration  # Use the user-specified duration

    # Run the simulation
    scene_states = simulate_scenes(
        simulator=simulator,
        batch_parameters=batch_parameters,
        scenes=scene,
    )
    
    # Get the state at the beginning and end of the simulation
    initial_state = scene_states[0]
    initial_robot_state = initial_state.get_modular_robot_simulation_state(robot)
    final_state = scene_states[-1]
    final_robot_state = final_state.get_modular_robot_simulation_state(robot)
    
    # Calculate final distance traveled
    global distance_traveled
    distance_traveled = fitness_functions.xy_displacement(
        initial_robot_state, final_robot_state
    )
    print(f"\nDistance traveled: {distance_traveled:.5f} meters")


def create_control_gui() -> None:
    """Create a GUI for controlling the robot movement."""
    root = tk.Tk()
    root.title("Distance Travel Test")
    root.geometry("400x300")
    
    # Target duration input
    target_frame = Frame(root)
    target_frame.grid(row=0, column=0, pady=10, padx=10, sticky="ew")
    
    Label(target_frame, text="Test Duration (s):").pack(side=tk.LEFT)
    
    target_var = StringVar(value="30.0")
    target_entry = Entry(target_frame, textvariable=target_var, width=10)
    target_entry.pack(side=tk.LEFT, padx=10)
    
    # Start button
    start_frame = Frame(root)
    start_frame.grid(row=1, column=0, pady=10, padx=10, sticky="ew")
    
    def start_simulation():
        try:
            # Parse and validate the target duration
            duration = float(target_var.get())
            if duration <= 0 or duration > 60:
                raise ValueError("Duration must be between 0 and 60 seconds")
            
            # Update UI
            status_label.config(text=f"Starting simulation for {duration:.1f} seconds...", fg="blue")
            root.update()
            
            # Close the UI window
            root.destroy()
            
            # Run the simulation in the main thread
            run_simulation(duration)
            
        except ValueError as e:
            status_label.config(text=f"Error: {e}", fg="red")
            return
    
    start_button = Button(
        start_frame, 
        text="START SIMULATION",  
        bg="green",    
        command=start_simulation,
        width=20,
        height=2
    )
    start_button.pack(padx=10)
    
    # Status label
    status_frame = Frame(root)
    status_frame.grid(row=2, column=0, pady=10, padx=10, sticky="ew")
    
    status_label = Label(status_frame, text="Enter duration and press START", font=("Arial", 12))
    status_label.pack()
    
    # Instructions
    instructions_frame = Frame(root)
    instructions_frame.grid(row=3, column=0, pady=20, padx=10, sticky="ew")
    
    instructions = """
    Instructions:
    1. Enter a test duration (1-60 seconds)
    2. Press START to run the simulation
    3. The robot will move for the specified time
    4. Final distance will be shown in the console
    """
    
    Label(instructions_frame, text=instructions, justify=tk.LEFT).pack()
    
    # Make sure the window appears in front
    root.lift()
    root.attributes('-topmost', True)
    root.after_idle(root.attributes, '-topmost', False)
    
    # Start the UI main loop
    root.mainloop()


def main() -> None:
    """Run the distance travel test simulation."""
    # Start with just the UI
    create_control_gui()


if __name__ == "__main__":
    main()
