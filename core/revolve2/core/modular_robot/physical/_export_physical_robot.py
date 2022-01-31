import json
from revolve2.core.modular_robot import ModularRobot
import os
import json
import shutil
from pathlib import Path


def export_physical_robot(robot: ModularRobot, output_path: str) -> None:
    if os.path.exists(output_path):
        raise RuntimeError("Output path already exists")

    os.mkdir(output_path)
    actor, controller = robot.make_actor_and_controller()
    controller.export_standalone(os.path.join(output_path, "brain"))

    with open(os.path.join(output_path, "gpio.json"), "w") as gpio:
        gpio.write(
            json.dumps(
                [{"dof": i, "gpio": i} for i in range(len(actor.joints))],
                indent=4,
            )
        )

    shutil.copyfile(
        os.path.join(Path(__file__).parent, "_main.py"),
        os.path.join(output_path, "main.py"),
    )
