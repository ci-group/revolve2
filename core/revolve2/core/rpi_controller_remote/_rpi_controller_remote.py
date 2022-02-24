from __future__ import annotations

import asyncssh
from contextlib import asynccontextmanager
import asyncio
import logging


class RpiControllerError(Exception):
    pass


@asynccontextmanager
async def connect(rpi_ip: str, username: str, password: str) -> RpiControllerRemote:
    async with asyncssh.connect(
        host=rpi_ip, username=username, password=password
    ) as conn:
        yield RpiControllerRemote(conn)


class RpiControllerRemote:
    def __init__(self, conn: asyncssh.SSHClientConnection) -> None:
        self._conn = conn

    async def run_controller(self, config: str, run_time: int) -> None:
        """
        :raises RpiControllerError: if something fails.
        """
        # TODO return timings and log
        logging.info("Initializing controller..")
        # use bash so everything works as if we are an interactive shell instead of remote command.
        # else things like $PATH differ which is very annoying for the user.
        async with self._conn.create_process(
            "revolve2_rpi_controller $HOME/config.json"
        ) as process:
            if process.is_closing():
                self._conn.wait_closed()
            try:
                read = await process.stdout.readline()
                if read == "":
                    raise RpiControllerError()
                logging.info("Controller initialized. Running..")
                process.stdin.write("\n")
                await asyncio.sleep(run_time)
                logging.info("Stopping running..")
                process.stdin.write("\n")
                await process.wait_closed()
                logging.info("Controller exited.")
                if process.returncode != 0:
                    raise RpiControllerError()
            except RpiControllerError as err:
                await process.wait_closed()
                if not process.stderr.at_eof():
                    msg = await process.stderr.read()
                    raise RpiControllerError(
                        f'Error when running controller program: "{msg}"'
                    ) from err
                else:
                    raise RpiControllerError(
                        f"Error when running controller program: <no stderr>"
                    ) from err
