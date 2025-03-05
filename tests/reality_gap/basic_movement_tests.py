"""Test script to assess robot movement patterns and gaits."""

import time
import math
import numpy as np
import tkinter as tk
from tkinter import Frame, Label, Button, Scale, OptionMenu, StringVar
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


class MovementBrainInstance(BrainInstance):
    """Brain instance that implements different movement patterns."""

    active_hinges: list[ActiveHinge]
    movement_type: str
    frequency: float
    amplitude: float
    phase_offset: float
    time_passed: float
    is_active: bool  
    
    def __init__(
        self, 
        active_hinges: list[ActiveHinge],
        movement_type: str = "sine_wave",
        frequency: float = 1.0,
        amplitude: float = 0.8,
        phase_offset: float = 0.5,
        is_active: bool = False,  
    ) -> None:
        """
        Initialize the brain instance with active hinges.

        Args:
            active_hinges: List of active hinges to control.
            movement_type: Type of movement pattern to use.
            frequency: Oscillation frequency in Hz.
            amplitude: Maximum angle amplitude (0.0-1.0).
            phase_offset: Phase offset between adjacent hinges.
            is_active: Whether movement is active.
        """
        self.active_hinges = active_hinges
        self.movement_type = movement_type
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
        Control the modular robot using the selected movement pattern.

        Args:
            dt: Elapsed seconds since last call to this function.
            sensor_state: Interface for reading the current sensor state.
            control_interface: Interface for controlling the robot.
        """
        self.time_passed += dt
        
        # Only apply movement patterns if active
        if self.is_active:
            if self.movement_type == "sine_wave":
                self._apply_sine_wave(control_interface)
            elif self.movement_type == "wave_gait":
                self._apply_wave_gait(control_interface)
            elif self.movement_type == "alternating":
                self._apply_alternating(control_interface)
            elif self.movement_type == "gecko_walk":
                self._apply_gecko_walk(control_interface)
            else:
                self._apply_sine_wave(control_interface)
        else:
            # When inactive, set all hinges to neutral position (0)
            for hinge in self.active_hinges:
                control_interface.set_active_hinge_target(hinge, 0.0)
    
    def _apply_sine_wave(self, control_interface: ModularRobotControlInterface) -> None:
        """Apply a sine wave pattern to all hinges with phase offset."""
        for i, hinge in enumerate(self.active_hinges):
            phase = i * self.phase_offset
            angle = self.amplitude * math.sin(2 * math.pi * self.frequency * self.time_passed + phase)
            # Scale to the actual range of the hinge
            control_interface.set_active_hinge_target(hinge, angle * 1.048)
    
    def _apply_wave_gait(self, control_interface: ModularRobotControlInterface) -> None:
        """Apply a wave gait pattern (good for many-legged robots)."""
        num_hinges = len(self.active_hinges)
        for i, hinge in enumerate(self.active_hinges):
            # Create a continuous wave through the body
            phase = (i / num_hinges) * 2 * math.pi
            angle = self.amplitude * math.sin(2 * math.pi * self.frequency * self.time_passed + phase)
            control_interface.set_active_hinge_target(hinge, angle * 1.048)
    
    def _apply_alternating(self, control_interface: ModularRobotControlInterface) -> None:
        """Apply an alternating pattern where adjacent hinges move in opposite directions."""
        for i, hinge in enumerate(self.active_hinges):
            # Alternate direction based on even/odd index
            direction = 1 if i % 2 == 0 else -1
            angle = direction * self.amplitude * math.sin(2 * math.pi * self.frequency * self.time_passed)
            control_interface.set_active_hinge_target(hinge, angle * 1.048)
    
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
            self._apply_wave_gait(control_interface)


class MovementBrain(Brain):
    """Brain that implements different movement patterns."""

    active_hinges: list[ActiveHinge]
    movement_type: str
    frequency: float
    amplitude: float
    phase_offset: float
    is_active: bool  
    _instance: MovementBrainInstance | None

    def __init__(
        self, 
        active_hinges: list[ActiveHinge],
        movement_type: str = "sine_wave",
        frequency: float = 1.0,
        amplitude: float = 0.8,
        phase_offset: float = 0.5,
        is_active: bool = False,  
    ) -> None:
        """
        Initialize the brain with active hinges.

        Args:
            active_hinges: List of active hinges to control.
            movement_type: Type of movement pattern to use.
            frequency: Oscillation frequency in Hz.
            amplitude: Maximum angle amplitude (0.0-1.0).
            phase_offset: Phase offset between adjacent hinges.
            is_active: Whether movement is active.
        """
        self.active_hinges = active_hinges
        self.movement_type = movement_type
        self.frequency = frequency
        self.amplitude = amplitude
        self.phase_offset = phase_offset
        self.is_active = is_active
        self._instance = None

    def make_instance(self) -> BrainInstance:
        """
        Create an instance of this brain.

        Returns:
            The created instance.
        """
        self._instance = MovementBrainInstance(
            self.active_hinges,
            self.movement_type,
            self.frequency,
            self.amplitude,
            self.phase_offset,
            self.is_active,
        )
        return self._instance
    
    def update_parameters(
        self, 
        movement_type: str = None,
        frequency: float = None,
        amplitude: float = None,
        phase_offset: float = None,
        is_active: bool = None,
    ) -> None:
        """
        Update the movement parameters.
        
        Args:
            movement_type: Type of movement pattern to use.
            frequency: Oscillation frequency in Hz.
            amplitude: Maximum angle amplitude (0.0-1.0).
            phase_offset: Phase offset between adjacent hinges.
            is_active: Whether movement is active.
        """
        if movement_type is not None:
            self.movement_type = movement_type
            if self._instance is not None:
                self._instance.movement_type = movement_type
        
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


def create_control_gui(brain: MovementBrain, robot_type: str):
    """Create a GUI for controlling the robot's movement patterns."""
    root = tk.Tk()
    root.title(f"{robot_type.capitalize()} Movement Control")
    root.geometry("600x400")
    
    # Configure grid
    root.grid_columnconfigure(0, weight=1)
    root.grid_rowconfigure(3, weight=1)
    
    # Movement pattern selection
    pattern_frame = Frame(root)
    pattern_frame.grid(row=0, column=0, pady=10, padx=10, sticky="ew")
    
    Label(pattern_frame, text="Movement Pattern:").pack(side=tk.LEFT)
    
    movement_patterns = [
        "sine_wave", 
        "wave_gait", 
        "alternating", 
        "gecko_walk"
    ]
    
    pattern_var = StringVar(value=brain.movement_type)
    
    def on_pattern_change(*args):
        brain.update_parameters(movement_type=pattern_var.get())
    
    pattern_menu = OptionMenu(pattern_frame, pattern_var, *movement_patterns)
    pattern_menu.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
    pattern_var.trace_add("write", on_pattern_change)
    
    # On/Off toggle button
    toggle_frame = Frame(root)
    toggle_frame.grid(row=1, column=0, pady=5, padx=10, sticky="ew")
    
    Label(toggle_frame, text="Movement:").pack(side=tk.LEFT)
    
    # Variable to track button state
    is_active_var = tk.BooleanVar(value=brain.is_active)
    
    def toggle_movement():
        new_state = not brain.is_active
        brain.update_parameters(is_active=new_state)
        is_active_var.set(new_state)
        toggle_button.config(text="ON" if new_state else "OFF", 
                            bg="green" if new_state else "red")
    
    toggle_button = Button(
        toggle_frame, 
        text="OFF",  
        bg="red",    
        command=toggle_movement,
        width=10,
        height=2
    )
    toggle_button.pack(side=tk.LEFT, padx=10)
    
    # Frequency control
    freq_frame = Frame(root)
    freq_frame.grid(row=2, column=0, pady=5, padx=10, sticky="ew")
    
    Label(freq_frame, text="Frequency (Hz):").pack(side=tk.LEFT)
    
    freq_value = Label(freq_frame, text=f"{brain.frequency:.2f}")
    freq_value.pack(side=tk.RIGHT, padx=10)
    
    def update_frequency(value):
        freq = float(value)
        brain.update_parameters(frequency=freq)
        freq_value.config(text=f"{freq:.2f}")
    
    freq_slider = Scale(
        freq_frame, 
        from_=0.1, 
        to=3.0, 
        resolution=0.1, 
        orient=tk.HORIZONTAL,
        command=update_frequency
    )
    freq_slider.set(brain.frequency)
    freq_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # Amplitude control
    amp_frame = Frame(root)
    amp_frame.grid(row=3, column=0, pady=5, padx=10, sticky="ew")
    
    Label(amp_frame, text="Amplitude:").pack(side=tk.LEFT)
    
    amp_value = Label(amp_frame, text=f"{brain.amplitude:.2f}")
    amp_value.pack(side=tk.RIGHT, padx=10)
    
    def update_amplitude(value):
        amp = float(value)
        brain.update_parameters(amplitude=amp)
        amp_value.config(text=f"{amp:.2f}")
    
    amp_slider = Scale(
        amp_frame, 
        from_=0.1, 
        to=1.0, 
        resolution=0.05, 
        orient=tk.HORIZONTAL,
        command=update_amplitude
    )
    amp_slider.set(brain.amplitude)
    amp_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # Phase offset control
    phase_frame = Frame(root)
    phase_frame.grid(row=4, column=0, pady=5, padx=10, sticky="ew")
    
    Label(phase_frame, text="Phase Offset:").pack(side=tk.LEFT)
    
    phase_value = Label(phase_frame, text=f"{brain.phase_offset:.2f}")
    phase_value.pack(side=tk.RIGHT, padx=10)
    
    def update_phase(value):
        phase = float(value)
        brain.update_parameters(phase_offset=phase)
        phase_value.config(text=f"{phase:.2f}")
    
    phase_slider = Scale(
        phase_frame, 
        from_=0.0, 
        to=2.0, 
        resolution=0.1, 
        orient=tk.HORIZONTAL,
        command=update_phase
    )
    phase_slider.set(brain.phase_offset)
    phase_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # Preset buttons
    preset_frame = Frame(root)
    preset_frame.grid(row=5, column=0, pady=10, padx=10, sticky="ew")
    
    Label(preset_frame, text="Presets:").pack(side=tk.LEFT)
    
    # Preset: Forward movement - optimized for gecko_v2
    def preset_forward():
        pattern_var.set("gecko_walk")  
        freq_slider.set(1.0)
        amp_slider.set(0.7)
        phase_slider.set(0.5)
        brain.update_parameters(
            movement_type="gecko_walk",
            frequency=1.0,
            amplitude=0.7,
            phase_offset=0.5
        )
    
    # Preset: Turn movement
    def preset_turn():
        pattern_var.set("sine_wave")
        freq_slider.set(1.0)
        amp_slider.set(0.8)
        phase_slider.set(0.5)
        brain.update_parameters(
            movement_type="sine_wave",
            frequency=1.0,
            amplitude=0.8,
            phase_offset=0.5
        )
    
    Button(preset_frame, text="Forward", command=preset_forward).pack(side=tk.LEFT, padx=5)
    Button(preset_frame, text="Turn", command=preset_turn).pack(side=tk.LEFT, padx=5)
    
    # Make sure the window appears in front
    root.lift()
    root.attributes('-topmost', True)
    root.after_idle(root.attributes, '-topmost', False)
    
    # Update the window to ensure proper sizing before mainloop
    root.update_idletasks()
    
    root.mainloop()


def main() -> None:
    """Run the simulation with movement patterns."""
    # Setup
    setup_logging()
    robot_type = "gecko"     # For this test, we'll always use the gecko
    body = gecko_v2()

    # Print information about the active hinges for reference
    active_hinges = body.find_modules_of_type(ActiveHinge)
    print(f"Found {len(active_hinges)} active hinges in the {robot_type} body")
    for i, hinge in enumerate(active_hinges):
        print(f"Hinge {i}: {hinge}")

    # Create the movement brain with default parameters
    brain = MovementBrain(
        active_hinges=active_hinges,
        movement_type="gecko_walk",  
        frequency=1.0,
        amplitude=0.7,
        phase_offset=0.5,
        is_active=False,  
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
        manual_control=False,  
    )

    # Configure simulation parameters
    batch_parameters = make_standard_batch_parameters()
    batch_parameters.simulation_time = 60  # Longer simulation time for testing

    # Start GUI in a separate thread
    threading.Thread(target=lambda: create_control_gui(brain, robot_type), daemon=True).start()

    # Run the simulation
    simulate_scenes(
        simulator=simulator,
        batch_parameters=batch_parameters,
        scenes=scene,
    )

if __name__ == "__main__":
    main()
