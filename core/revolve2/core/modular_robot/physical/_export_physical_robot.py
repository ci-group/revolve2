import json
from revolve2.core.modular_robot import ModularRobot
import os
import json
import shutil
from pathlib import Path


def export_physical_robot(
    robot: ModularRobot, output_path: str, control_frequency: float
) -> None:
    if os.path.exists(output_path):
        raise RuntimeError("Output path already exists")

    os.mkdir(output_path)
    actor, controller = robot.make_actor_and_controller()
    controller.export_standalone(os.path.join(output_path, "brain"))

    with open(os.path.join(output_path, "settings.json"), "w") as gpio:
        gpio.write(
            json.dumps(
                {
                    "gpio": [
                        {"dof": i, "gpio_pin": i, "invert": False}
                        for i in range(len(actor.joints))
                    ],
                    "control_frequency": control_frequency,
                },
                indent=4,
            ),
        )

    shutil.copyfile(
        os.path.join(Path(__file__).parent, "_main.py"),
        os.path.join(output_path, "main.py"),
    )

    shutil.copyfile(
        os.path.join(Path(__file__).parent, "_settings_schema.json"),
        os.path.join(output_path, "settings_schema.json"),
    )

    os.makedirs(os.path.join(output_path, "revolve2/core/physics/control"))
    shutil.copyfile(
        os.path.join(
            Path(__file__).parent.parent.parent,
            "physics/control/_actor_controller.py",
        ),
        os.path.join(output_path, "revolve2/core/physics/control/_actor_controller.py"),
    )
    with open(
        os.path.join(output_path, "revolve2/core/physics/control/__init__.py"), "w"
    ) as init:
        init.write(
            """from ._actor_controller import ActorController\n\n__all__ = ["ActorController"]"""
        )
