from .interface import Interface
import asyncio
import sys
import traceback
from .controller import Controller
import json


class Cli(Interface):
    _config_file: str

    def __init__(self, controller: Controller, config_file: str) -> None:
        super().__init__(controller)
        self._config_file = config_file

    @staticmethod
    async def _ainput() -> str:
        return (
            await asyncio.get_event_loop().run_in_executor(None, sys.stdin.readline)
        )[:-1]

    async def run(self) -> None:
        if not await self._load_controller():
            return

        print("Press enter to start controller..")
        _ = await self._ainput()
        await self._controller.start_controller()

        print("Press enter to stop controller..")
        _ = await self._ainput()
        await self._controller.stop_controller()

    async def _load_controller(self) -> bool:
        try:
            with open(self._config_file, "r") as config_file:
                config = json.loads(config_file.read())
            await self._controller.load_controller(config)
            return True
        except FileNotFoundError:
            print("Config file not found.")
            return False
        except json.JSONDecodeError:
            print("Config not valid JSON.")
            return False
        except (Controller.SystemError, Controller.UserError):
            print(f"Could not load controller: {traceback.format_exc()}")
            return False
