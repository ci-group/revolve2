"""
This is an automatically generated stub for `robot_daemon_protocol.capnp`.

This file was manually edited to contain the RoboServer interface.
"""
import os

import capnp  # type: ignore

capnp.remove_import_hook()
here = os.path.dirname(os.path.abspath(__file__))
module_file = os.path.abspath(os.path.join(here, "robot_daemon_protocol.capnp"))
SetupArgs = capnp.load(module_file).SetupArgs
SetupArgsBuilder = SetupArgs
SetupArgsReader = SetupArgs
SetupResponse = capnp.load(module_file).SetupResponse
SetupResponseBuilder = SetupResponse
SetupResponseReader = SetupResponse
PinControl = capnp.load(module_file).PinControl
PinControlBuilder = PinControl
PinControlReader = PinControl
ControlArgs = capnp.load(module_file).ControlArgs
ControlArgsBuilder = ControlArgs
ControlArgsReader = ControlArgs
ReadSensorsArgs = capnp.load(module_file).ReadSensorsArgs
ReadSensorsArgsBuilder = ReadSensorsArgs
ReadSensorsArgsReader = ReadSensorsArgs
ControlAndReadSensorsArgs = capnp.load(module_file).ControlAndReadSensorsArgs
ControlAndReadSensorsArgsBuilder = ControlAndReadSensorsArgs
ControlAndReadSensorsArgsReader = ControlAndReadSensorsArgs
Vector3 = capnp.load(module_file).Vector3
Vector3Builder = Vector3
Vector3Reader = Vector3
Image = capnp.load(module_file).Image
ImageBuilder = Image
ImageReader = Image
SensorReadings = capnp.load(module_file).SensorReadings
SensorReadingsBuilder = SensorReadings
SensorReadingsReader = SensorReadings
RoboServer = capnp.load(module_file).RoboServer
