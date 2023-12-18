"""
This is an automatically generated stub for `robot_daemon_protocol.capnp`.

This file was manually edited to contain the RoboServer interface.
"""
import os

import capnp  # type: ignore

capnp.remove_import_hook()
here = os.path.dirname(os.path.abspath(__file__))
module_file = os.path.abspath(os.path.join(here, "robot_daemon_protocol.capnp"))
Setupargs = capnp.load(module_file).Setupargs
SetupargsBuilder = Setupargs
SetupargsReader = Setupargs
SetupResponse = capnp.load(module_file).SetupResponse
SetupResponseBuilder = SetupResponse
SetupResponseReader = SetupResponse
PinControl = capnp.load(module_file).PinControl
PinControlBuilder = PinControl
PinControlReader = PinControl
ControlCommands = capnp.load(module_file).ControlCommands
ControlCommandsBuilder = ControlCommands
ControlCommandsReader = ControlCommands
SensorReadings = capnp.load(module_file).SensorReadings
SensorReadingsBuilder = SensorReadings
SensorReadingsReader = SensorReadings
RoboServer = capnp.load(module_file).RoboServer
