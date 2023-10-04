import concurrent.futures
import logging
import os

from revolve2.simulation.scene import SimulationState
from revolve2.simulation.simulator import Batch, Simulator

from ._simulate_scene import simulate_scene


class LocalSimulator(Simulator):
    """Runner for simulating using Mujoco."""

    _headless: bool
    _start_paused: bool
    _num_simulators: int

    def __init__(
        self,
        headless: bool = False,
        start_paused: bool = False,
        num_simulators: int = 1,
    ):
        """
        Initialize this object.

        :param headless: If True, the simulation will not be rendered. This drastically improves performance.
        :param start_paused: If True, start the simulation paused. Only possible when not in headless mode.
        :param num_simulators: The number of simulators to deploy in parallel. They will take one core each but will share space on the main python thread for calculating control.
        """
        assert (
            headless or num_simulators == 1
        ), "Cannot have parallel simulators when visualizing."

        assert not (
            headless and start_paused
        ), "Cannot start simulation paused in headless mode."

        self._headless = headless
        self._start_paused = start_paused
        self._num_simulators = num_simulators

    def simulate_batch(self, batch: Batch) -> list[list[SimulationState]]:
        """
        Simulate the provided batch by simulating each contained scene.

        :param batch: The batch to run.
        :returns: List of simulation states in ascending order of time.
        """
        logging.info("Starting simulation batch with MuJoCo.")

        control_step = 1.0 / batch.parameters.control_frequency
        sample_step = (
            None
            if batch.parameters.sampling_frequency is None
            else 1.0 / batch.parameters.sampling_frequency
        )

        if batch.record_settings is not None:
            os.makedirs(batch.record_settings.video_directory, exist_ok=False)

        with concurrent.futures.ProcessPoolExecutor(
            max_workers=self._num_simulators
        ) as executor:
            futures = [
                executor.submit(
                    simulate_scene,
                    scene_index,
                    scene,
                    self._headless,
                    batch.record_settings,
                    self._start_paused,
                    control_step,
                    sample_step,
                    batch.parameters.simulation_time,
                    batch.parameters.simulation_timestep,
                )
                for scene_index, scene in enumerate(batch.scenes)
            ]
            results = [future.result() for future in futures]

        logging.info("Finished batch.")

        return results
