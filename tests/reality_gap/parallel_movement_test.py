"""
Script for simultaneously testing movement patterns on both simulated and physical gecko robots
to compare and analyze the sim-to-real gap.
"""

import socket
import time
import math
import logging
import threading
import tkinter as tk
from tkinter import Scale, Button, Frame, Label, Entry, StringVar, OptionMenu, LabelFrame
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


class ParallelMovementController:
    """Controller for both simulated and physical robots with movement patterns."""

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
        
        # Create brains for both robots with the same initial parameters
        self.sim_brain = MovementBrain(
            active_hinges=self.active_hinges,
            movement_type="gecko_walk",
            frequency=1.0,
            amplitude=0.7,
            phase_offset=0.5,
            is_active=False,
        )
        
        self.phys_brain = MovementBrain(
            active_hinges=self.active_hinges,
            movement_type="gecko_walk",
            frequency=1.0,
            amplitude=0.7,
            phase_offset=0.5,
            is_active=False,
        )
        
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
        self.root.title("Parallel Movement Control - Sim-to-Real Comparison")
        self.root.geometry("800x600")
        
        # Configure row and column weights for responsive resizing
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=0)  # Control panel
        self.root.grid_rowconfigure(1, weight=0)  # Movement pattern
        self.root.grid_rowconfigure(2, weight=0)  # Frequency
        self.root.grid_rowconfigure(3, weight=0)  # Amplitude
        self.root.grid_rowconfigure(4, weight=0)  # Phase offset
        self.root.grid_rowconfigure(5, weight=0)  # Activation
        self.root.grid_rowconfigure(6, weight=0)  # Presets
        
        # Control panel for starting/stopping robots
        control_panel = LabelFrame(self.root, text="Robot Control")
        control_panel.grid(row=0, column=0, pady=10, padx=10, sticky="ew")
        
        Button(control_panel, text="Start Simulation", 
               command=self.start_simulation).grid(row=0, column=0, padx=10, pady=5)
        
        Button(control_panel, text="Start Physical Robot", 
               command=self.start_physical_robot).grid(row=0, column=1, padx=10, pady=5)
        
        # Movement pattern selection
        pattern_frame = Frame(self.root)
        pattern_frame.grid(row=1, column=0, pady=5, padx=10, sticky="ew")
        
        Label(pattern_frame, text="Movement Pattern:").pack(side=tk.LEFT)
        
        self.pattern_var = StringVar(value=self.sim_brain.movement_type)
        patterns = ["sine_wave", "alternating", "sequential", "gecko_walk"]
        
        def update_pattern(*args):
            pattern = self.pattern_var.get()
            self.sim_brain.update_parameters(movement_type=pattern)
            self.phys_brain.update_parameters(movement_type=pattern)
        
        pattern_menu = OptionMenu(pattern_frame, self.pattern_var, *patterns)
        pattern_menu.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        self.pattern_var.trace_add("write", update_pattern)
        
        # Frequency control
        freq_frame = Frame(self.root)
        freq_frame.grid(row=2, column=0, pady=5, padx=10, sticky="ew")
        
        Label(freq_frame, text="Frequency (Hz):").pack(side=tk.LEFT)
        
        self.freq_value = Label(freq_frame, text=f"{self.sim_brain.frequency:.2f}")
        self.freq_value.pack(side=tk.RIGHT, padx=10)
        
        def update_frequency(value):
            freq = float(value)
            self.sim_brain.update_parameters(frequency=freq)
            self.phys_brain.update_parameters(frequency=freq)
            self.freq_value.config(text=f"{freq:.2f}")
        
        self.freq_slider = Scale(
            freq_frame, 
            from_=0.1, 
            to=2.0, 
            resolution=0.1, 
            orient=tk.HORIZONTAL,
            command=update_frequency
        )
        self.freq_slider.set(self.sim_brain.frequency)
        self.freq_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # Amplitude control
        amp_frame = Frame(self.root)
        amp_frame.grid(row=3, column=0, pady=5, padx=10, sticky="ew")
        
        Label(amp_frame, text="Amplitude:").pack(side=tk.LEFT)
        
        self.amp_value = Label(amp_frame, text=f"{self.sim_brain.amplitude:.2f}")
        self.amp_value.pack(side=tk.RIGHT, padx=10)
        
        def update_amplitude(value):
            amp = float(value)
            self.sim_brain.update_parameters(amplitude=amp)
            self.phys_brain.update_parameters(amplitude=amp)
            self.amp_value.config(text=f"{amp:.2f}")
        
        self.amp_slider = Scale(
            amp_frame, 
            from_=0.1, 
            to=1.0, 
            resolution=0.05, 
            orient=tk.HORIZONTAL,
            command=update_amplitude
        )
        self.amp_slider.set(self.sim_brain.amplitude)
        self.amp_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # Phase offset control
        phase_frame = Frame(self.root)
        phase_frame.grid(row=4, column=0, pady=5, padx=10, sticky="ew")
        
        Label(phase_frame, text="Phase Offset:").pack(side=tk.LEFT)
        
        self.phase_value = Label(phase_frame, text=f"{self.sim_brain.phase_offset:.2f}")
        self.phase_value.pack(side=tk.RIGHT, padx=10)
        
        def update_phase(value):
            phase = float(value)
            self.sim_brain.update_parameters(phase_offset=phase)
            self.phys_brain.update_parameters(phase_offset=phase)
            self.phase_value.config(text=f"{phase:.2f}")
        
        self.phase_slider = Scale(
            phase_frame, 
            from_=0.0, 
            to=2.0, 
            resolution=0.1, 
            orient=tk.HORIZONTAL,
            command=update_phase
        )
        self.phase_slider.set(self.sim_brain.phase_offset)
        self.phase_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # Activation control
        active_frame = Frame(self.root)
        active_frame.grid(row=5, column=0, pady=10, padx=10, sticky="ew")
        
        def toggle_active():
            new_state = not self.sim_brain.is_active
            self.sim_brain.update_parameters(is_active=new_state)
            self.phys_brain.update_parameters(is_active=new_state)
            active_button.config(text=f"{'Stop' if new_state else 'Start'} Movement")
        
        active_button = Button(
            active_frame, 
            text="Start Movement", 
            command=toggle_active,
            height=2
        )
        active_button.pack(fill=tk.X)
        
        # Preset buttons
        preset_frame = Frame(self.root)
        preset_frame.grid(row=6, column=0, pady=10, padx=10, sticky="ew")
        
        Label(preset_frame, text="Presets:").pack(side=tk.LEFT)
        
        # Preset: Forward movement - optimized for gecko_v2
        def preset_forward():
            self.pattern_var.set("gecko_walk")
            self.freq_slider.set(1.0)
            self.amp_slider.set(0.7)
            self.phase_slider.set(0.5)
            self.sim_brain.update_parameters(
                movement_type="gecko_walk",
                frequency=1.0,
                amplitude=0.7,
                phase_offset=0.5
            )
            self.phys_brain.update_parameters(
                movement_type="gecko_walk",
                frequency=1.0,
                amplitude=0.7,
                phase_offset=0.5
            )
            self.freq_value.config(text="1.00")
            self.amp_value.config(text="0.70")
            self.phase_value.config(text="0.50")
        
        # Preset: Turn movement
        def preset_turn():
            self.pattern_var.set("alternating")
            self.freq_slider.set(1.0)
            self.amp_slider.set(0.8)
            self.phase_slider.set(0.5)
            self.sim_brain.update_parameters(
                movement_type="alternating",
                frequency=1.0,
                amplitude=0.8,
                phase_offset=0.5
            )
            self.phys_brain.update_parameters(
                movement_type="alternating",
                frequency=1.0,
                amplitude=0.8,
                phase_offset=0.5
            )
            self.freq_value.config(text="1.00")
            self.amp_value.config(text="0.80")
            self.phase_value.config(text="0.50")
        
        # Preset: Wave movement
        def preset_wave():
            self.pattern_var.set("sequential")
            self.freq_slider.set(0.5)
            self.amp_slider.set(0.9)
            self.phase_slider.set(1.0)
            self.sim_brain.update_parameters(
                movement_type="sequential",
                frequency=0.5,
                amplitude=0.9,
                phase_offset=1.0
            )
            self.phys_brain.update_parameters(
                movement_type="sequential",
                frequency=0.5,
                amplitude=0.9,
                phase_offset=1.0
            )
            self.freq_value.config(text="0.50")
            self.amp_value.config(text="0.90")
            self.phase_value.config(text="1.00")
        
        Button(preset_frame, text="Forward", command=preset_forward).pack(side=tk.LEFT, padx=5)
        Button(preset_frame, text="Turn", command=preset_turn).pack(side=tk.LEFT, padx=5)
        Button(preset_frame, text="Wave", command=preset_wave).pack(side=tk.LEFT, padx=5)
        
        # Make sure the window appears in front
        self.root.lift()
        self.root.attributes('-topmost', True)
        self.root.after_idle(self.root.attributes, '-topmost', False)
        
        # Update the window to ensure proper sizing before mainloop
        self.root.update_idletasks()
    
    def run(self):
        """Run the main application."""
        self.root.mainloop()


def main():
    """Run the parallel movement controller."""
    controller = ParallelMovementController()
    controller.run()


if __name__ == "__main__":
    main()
