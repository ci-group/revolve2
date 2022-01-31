import asyncio
from brain.brain import Brain  # type: ignore # Will be added when brain is exported
import json
from pathlib import Path
import os
import json
import time


async def async_main() -> None:
    with open(
        os.path.join(Path(__file__).parent, "settings.json"), "r"
    ) as settings_file:
        settings = json.loads(settings_file.read())

    period = 1.0 / settings["control_frequency"]

    brain = Brain()
    print("Hello world!")

    last_update_time = time.time()

    while True:
        await asyncio.sleep(period)
        current_time = time.time()
        elapsed_time = current_time - last_update_time
        last_update_time = current_time
        brain.step(elapsed_time)
        targets = brain.get_dof_targets()


def main() -> None:
    import asyncio

    asyncio.run(async_main())


if __name__ == "__main__":
    main()
