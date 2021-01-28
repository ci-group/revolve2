from abc import ABC, abstractmethod

from simulation.simulator.simulator_state import SimulatorState


class Simulator(ABC):

    def __init__(self):
        self.state: SimulatorState = SimulatorState.WAITING

    @abstractmethod
    def _connect(self) -> bool:
        pass

    @abstractmethod
    def _disconnect(self) -> bool:
        pass

    @abstractmethod
    def _pause_simulation(self) -> bool:
        pass

    @abstractmethod
    def _start_simulation(self) -> bool:
        pass

    @abstractmethod
    def _stop_simulation(self) -> bool:
        pass

    @abstractmethod
    def get_simulation_time(self):
        pass

    def stop(self):
        if self.state == SimulatorState.RUNNING:
            success = self._stop_simulation()
            if success:
                self.state = SimulatorState.READY

    def play(self):
        if self.state == SimulatorState.WAITING:
            success = self._connect()
            if success:
                self.state = SimulatorState.READY

        if self.state == SimulatorState.READY:
            success = self._start_simulation()
            if success:
                self.state = SimulatorState.RUNNING

    def pause(self):
        if self.state == SimulatorState.RUNNING:
            success = self._pause_simulation()
            if success:
                self.state = SimulatorState.PAUSED

    def exit(self):
        if self.state == SimulatorState.READY or self.state == SimulatorState.WAITING:
            success = self._disconnect()
            if success:
                self.state = SimulatorState.PAUSED
