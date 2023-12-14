=================================
An introduction to modular robots
=================================
This introduction applies to V2 robots. V1 robots are only used internally in CI Group.

-------
Modules
-------
Modular robots are robots built from a set of module types.
The modular robots system in Revolve2 is based on the RoboGen system (*RoboGen: Robot Generation through Artificial Evolution, ser. Artificial Life Conference Proceedings, vol. ALIFE 14: The Fourteenth International Conference on the Synthesis and Simulation of Living Systems, 07 2014*).
At the center of the robot lies the core module.
<<<real photo, sim photo>>>
Other modules are attached to the core in its sides.
Brick are passive blocks to which again modules can be attached.
<<<real photo, sim photo>>>
Active hinges are revolute joints that are controlled by the robot's brain.
<<<real photo, sim photo>>>

-------
Sensors
-------
Currently the brain can read only the current position of each servo. This works both in simulation and real hardware.
