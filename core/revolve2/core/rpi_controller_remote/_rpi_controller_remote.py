from __future__ import annotations

import asyncio
import datetime
import json
import logging
from contextlib import asynccontextmanager
from typing import AsyncIterator, Tuple

import asyncssh.connection
from revolve2.serialization import StaticData


class RpiControllerError(Exception):
    """Error or when something fails in the RpiControllerRemote."""


@asynccontextmanager
async def connect(
    rpi_ip: str, username: str, password: str
) -> AsyncIterator[RpiControllerRemote]:
    """
    Connect to the controller machine using ssh and encapsulate that in an instance of `RpiControllerRemote`.

    :param rpi_ip: Ip of the machine.
    :param username: Username for ssh.
    :param password: Password for ssh.
    :yields: A remote controller.
    """
    async with asyncssh.connection.connect(
        host=rpi_ip, username=username, password=password
    ) as conn:
        yield RpiControllerRemote(conn)  # TODO yield or return?


class RpiControllerRemote:
    """A wrapper around an ssh connection that allows running of the rpi controller on the controller machine."""

    def __init__(self, conn: asyncssh.connection.SSHClientConnection) -> None:
        """
        Initialize this object.

        :param conn: Ssh connection to the pi.
        """
        self._conn = conn

    async def run_controller(
        self, config: StaticData, run_time: int
    ) -> Tuple[datetime.datetime, StaticData]:
        """
        Run the rpi controller on the controller machine.

        :param config: config to use for rpi controller.
        :param run_time: run controller for this many seconds.
        :returns: Tuple of controller start time and controller log.
        :raises RpiControllerError: if something fails.
        """
        start_time: datetime.datetime

        logging.info("Copying config..")
        async with self._conn.start_sftp_client() as sftp:
            async with sftp.open(
                "revolve2_rpi_controller_config.json", "w"
            ) as config_file:
                await config_file.write(json.dumps(config))
        logging.info("Copying done.")

        logging.info("Initializing controller..")
        async with self._conn.create_process(
            "revolve2_rpi_controller revolve2_rpi_controller_config.json --log revolve2_rpi_controller_log.json"
        ) as process:
            if process.is_closing():
                await self._conn.wait_closed()
            try:
                read = await process.stdout.readline()
                if read == "":
                    raise RpiControllerError()
                logging.info("Controller initialized. Running..")
                start_time = datetime.datetime.now()
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
                        "Error when running controller program: <no stderr>"
                    ) from err
        logging.info("Successfully ran controller.")

        logging.info("Retrieving log file..")
        async with self._conn.start_sftp_client() as sftp:
            async with sftp.open("revolve2_rpi_controller_log.json", "r") as log_file:
                log = await log_file.read()
        logging.info("Retrieving done.")
        try:
            return start_time, json.loads(log)
        except json.JSONDecodeError as err:
            raise RpiControllerError("Could not parse log.") from err
