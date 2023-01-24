"""Script that uses rpi controller remote to control a rpi robot as an example."""

import logging

from revolve2.core.rpi_controller_remote import connect
from revolve2.serialization import StaticData

config: StaticData = {
    "controller_module": "revolve2.actor_controllers.cpg",
    "controller_type": "Cpg",
    "control_frequency": 10,
    "gpio": [{"dof": 0, "gpio_pin": 17, "invert": False}],
    "serialized_controller": {
        "state": [0.707, 0.707],
        "num_output_neurons": 1,
        "weight_matrix": [[0.0, 0.5], [-0.5, 0.0]],
    },
}


async def main() -> None:
    """Run the script."""
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    async with connect("192.168.178.46", "pi", "raspberry") as conn:
        start_time1, log1 = await conn.run_controller(config, 5)
        print(start_time1, log1)
        start_time2, log2 = await conn.run_controller(config, 10)
        print(start_time2, log2)
        start_time3, log3 = await conn.run_controller(config, 15)
        print(start_time3, log3)


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
