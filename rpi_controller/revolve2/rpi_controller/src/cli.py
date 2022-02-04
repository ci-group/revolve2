from .interface import Interface
import asyncio
import sys
from revolve2.object_controller import ObjectController
from .controller import Controller


class Cli(Interface):
    def __init__(self, controller: Controller) -> None:
        super().__init__(controller)

    @staticmethod
    async def _ainput() -> str:
        return (
            await asyncio.get_event_loop().run_in_executor(None, sys.stdin.readline)
        )[:-1]

    async def run(self) -> None:
        while True:
            controller = await self._get_controller()
            await self._controller.start_controller(controller)

    async def _get_controller(self) -> ObjectController:
        while True:
            print("Enter controller config file:")
            config_file = await self._ainput()
            if not await self._controller.start_controller(config_file):
                break

            print("Press enter to stop controller..")
            _ = await self._ainput()
            await self._controller.stop_controller()
