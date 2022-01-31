from brain.brain import Brain  # type: ignore # Will be added when brain is exported
import json
from pathlib import Path
import os
import json


async def async_main() -> None:
    with open(
        os.path.join(Path(__file__).parent, "settings.json"), "r"
    ) as settings_file:
        settings = json.loads(settings_file.read())

    brain = Brain()
    print("Hello world!")


def main() -> None:
    import asyncio

    asyncio.run(async_main())


if __name__ == "__main__":
    main()
