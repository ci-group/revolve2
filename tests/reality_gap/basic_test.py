"""Test script to measure robot travel distance."""

import time
import math
import numpy as np
import tkinter as tk
from tkinter import Frame, Label, Button, Entry, StringVar
import threading

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
                phase = i * self.phase_offset
                angle = self.amplitude * math.sin(2 * math.pi * self.frequency * self.time_passed + phase)
                control_interface.set_active_hinge_target(hinge, angle * 1.048)


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
        """Create an instance of this brain."""
        self._instance = DistanceTravelBrainInstance(
            active_hinges=self.active_hinges,
            frequency=self.frequency,
            amplitude=self.amplitude,
            phase_offset=self.phase_offset,
            is_active=self.is_active,
        )
        return self._instance
    
    def update_parameters(
        self, 
        frequency: float = None,
        amplitude: float = None,
        phase_offset: float = None,
        is_active: bool = None,
    ) -> None:
        """
        Update the movement parameters.
        
        Args:
            frequency: Oscillation frequency in Hz.
            amplitude: Maximum angle amplitude (0.0-1.0).
            phase_offset: Phase offset between adjacent hinges.
            is_active: Whether movement is active.
        """
        if frequency is not None:
            self.frequency = frequency
            if self._instance is not None:
                self._instance.frequency = frequency
        
        if amplitude is not None:
            self.amplitude = amplitude
            if self._instance is not None:
                self._instance.amplitude = amplitude
        
        if phase_offset is not None:
            self.phase_offset = phase_offset
            if self._instance is not None:
                self._instance.phase_offset = phase_offset
                
        if is_active is not None:
            self.is_active = is_active
            if self._instance is not None:
                self._instance.is_active = is_active


# Global variables to track distance
initial_robot_state = None
current_robot_state = None
distance_traveled = 0.0
robot_reference = None


def create_control_gui(brain: DistanceTravelBrain):
    """Create a GUI for controlling the distance travel test."""
    root = tk.Tk()
    root.title("Distance Travel Test")
    root.geometry("500x350")
    
    # Configure grid
    root.grid_columnconfigure(0, weight=1)
    
    # Target distance input
    target_frame = Frame(root)
    target_frame.grid(row=0, column=0, pady=10, padx=10, sticky="ew")
    
    Label(target_frame, text="Target Distance (m):").pack(side=tk.LEFT)
    
    target_var = StringVar(value="2.0")
    target_entry = Entry(target_frame, textvariable=target_var, width=10)
    target_entry.pack(side=tk.LEFT, padx=10)
    
    # On/Off toggle button
    toggle_frame = Frame(root)
    toggle_frame.grid(row=1, column=0, pady=10, padx=10, sticky="ew")
    
    Label(toggle_frame, text="Movement:").pack(side=tk.LEFT)
    
    # Start time for timer
    start_time = [None]
    
    def toggle_movement():
        global initial_robot_state
        new_state = not brain.is_active
        brain.update_parameters(is_active=new_state)
        
        if new_state:
            # Starting movement
            toggle_button.config(text="STOP", bg="red")
            start_time[0] = time.time()
            # Reset initial position when starting
            initial_robot_state = current_robot_state
        else:
            # Stopping movement
            toggle_button.config(text="START", bg="green")
            start_time[0] = None
    
    toggle_button = Button(
        toggle_frame, 
        text="START",  
        bg="green",    
        command=toggle_movement,
        width=10,
        height=2
    )
    toggle_button.pack(side=tk.LEFT, padx=10)
    
    # Distance and time display
    info_frame = Frame(root)
    info_frame.grid(row=2, column=0, pady=20, padx=10, sticky="ew")
    
    # Distance traveled
    distance_frame = Frame(info_frame)
    distance_frame.pack(fill=tk.X, pady=5)
    
    Label(distance_frame, text="Distance Traveled:", font=("Arial", 12)).pack(side=tk.LEFT)
    distance_label = Label(distance_frame, text="0.00 m", font=("Arial", 12, "bold"))
    distance_label.pack(side=tk.RIGHT)
    
    # Elapsed time
    time_frame = Frame(info_frame)
    time_frame.pack(fill=tk.X, pady=5)
    
    Label(time_frame, text="Elapsed Time:", font=("Arial", 12)).pack(side=tk.LEFT)
    time_label = Label(time_frame, text="0.00 s", font=("Arial", 12, "bold"))
    time_label.pack(side=tk.RIGHT)
    
    # Target distance display
    target_display_frame = Frame(info_frame)
    target_display_frame.pack(fill=tk.X, pady=5)
    
    Label(target_display_frame, text="Target Distance:", font=("Arial", 12)).pack(side=tk.LEFT)
    target_label = Label(target_display_frame, text="2.00 m", font=("Arial", 12, "bold"))
    target_label.pack(side=tk.RIGHT)
    
    # Progress display
    progress_frame = Frame(info_frame)
    progress_frame.pack(fill=tk.X, pady=10)
    
    Label(progress_frame, text="Progress:", font=("Arial", 12)).pack(side=tk.LEFT)
    progress_label = Label(progress_frame, text="0.0%", font=("Arial", 12, "bold"))
    progress_label.pack(side=tk.RIGHT)
    
    # Update function for the UI
    def update_ui():
        if root.winfo_exists():
            # Update distance traveled
            global distance_traveled
            distance_label.config(text=f"{distance_traveled:.2f} m")
            
            # Update target distance
            try:
                target_distance = float(target_var.get())
                target_label.config(text=f"{target_distance:.2f} m")
                
                # Update progress percentage
                if target_distance > 0:
                    progress = min(100.0, (distance_traveled / target_distance) * 100)
                    progress_label.config(text=f"{progress:.1f}%")
                else:
                    progress_label.config(text="0.0%")
            except ValueError:
                target_label.config(text="Invalid")
                progress_label.config(text="N/A")
            
            # Update elapsed time
            if start_time[0] is not None:
                elapsed = time.time() - start_time[0]
                time_label.config(text=f"{elapsed:.2f} s")
            
            # Schedule the next update
            root.after(100, update_ui)
    
    # Start the UI update loop
    update_ui()
    
    # Make sure the window appears in front
    root.lift()
    root.attributes('-topmost', True)
    root.after_idle(root.attributes, '-topmost', False)
    
    # Update the window to ensure proper sizing before mainloop
    root.update_idletasks()
    
    root.mainloop()


def main() -> None:
    """Run the distance travel test simulation."""
    # Setup
    setup_logging()
    body = gecko_v2()

    # Find all active hinges in the body
    active_hinges = body.find_modules_of_type(ActiveHinge)
    
    # Print information about the active hinges for reference
    print(f"Found {len(active_hinges)} active hinges in the gecko body")
    for i, hinge in enumerate(active_hinges):
        print(f"Hinge {i}: {hinge}")

    # Create the brain with default parameters
    brain = DistanceTravelBrain(
        active_hinges=active_hinges,
        frequency=1.0,
        amplitude=0.7,
        phase_offset=0.5,
        is_active=False,  # Start with movement disabled
    )
    
    # Create the modular robot
    global robot_reference
    robot = ModularRobot(body, brain)
    robot_reference = robot

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
    batch_parameters.simulation_time = 120  # Longer simulation time for testing

    # Start GUI in a separate thread
    threading.Thread(target=lambda: create_control_gui(brain), daemon=True).start()

    # Run the simulation with a callback to update distance
    scene_states = simulate_scenes(
        simulator=simulator,
        batch_parameters=batch_parameters,
        scenes=scene,
    )
    
    # This code won't be reached during normal execution since simulate_scenes blocks
    # until simulation is complete, but we include it for completeness
    global initial_robot_state, current_robot_state, distance_traveled
    
    # Get the state at the beginning of the simulation
    initial_state = scene_states[0]
    initial_robot_state = initial_state.get_modular_robot_simulation_state(robot)
    
    # Calculate distance for each state update
    for state in scene_states[1:]:
        current_robot_state = state.get_modular_robot_simulation_state(robot)
        if initial_robot_state is not None:
            distance_traveled = fitness_functions.xy_displacement(
                initial_robot_state, current_robot_state
            )


if __name__ == "__main__":
    main()
