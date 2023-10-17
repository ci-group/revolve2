from revolve2.modular_robot import ModularRobot
from revolve2.simulation.scene import MultiBodySystem, UUIDKey
from revolve2.simulation.simulator import Batch, BatchParameters, RecordSettings

from ._modular_robot_scene import ModularRobotScene


def to_batch(
    scenes: ModularRobotScene | list[ModularRobotScene],
    batch_parameters: BatchParameters,
    record_settings: RecordSettings | None = None,
) -> tuple[Batch, list[dict[UUIDKey[ModularRobot], MultiBodySystem]]]:
    """
    Convert one or more modular robot scenes to a batch of simulation scenes.

    :param scenes: The modular robot scene(s) to make the batch from.
    :param batch_parameters: Parameters for the batch that are not contained in the modular robot scenes.
    :param record_settings: Setting for recording the simulations.
    :returns: The created batch and a mapping from modular robots to multi-body systems for each scene.
    """
    if isinstance(scenes, ModularRobotScene):
        scenes = [scenes]

    converted = [
        modular_robot_scene.to_simulation_scene() for modular_robot_scene in scenes
    ]

    batch = Batch(parameters=batch_parameters, record_settings=record_settings)
    batch.scenes.extend(simulation_scene for simulation_scene, _ in converted)

    return batch, [mapping for _, mapping in converted]
