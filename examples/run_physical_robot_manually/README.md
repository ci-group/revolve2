In this example, you will manually create and run a physical robot using SSH. You are most likely looking for the `physical_robot_remote` example instead, which applies most of the steps explained here automatically.

You learn:
- How to create a configuration describing how a physical modular robot should be controlled.
- How to upload this configuration onto the robot.
- How to run the physical robot using this configuration.

Requirements:
- A physical modular robot. Not whether you are using V1 or V2 hardware; If you do not know, it is probably V2. TODO in the next version of Revolve2 a guide will be added on how to build a physical robot. For now, ask the CI Group lab.
- The modular robot body and brain defined in a Revolve2 ModularRobot object. It is recommended that you test it in simulation first. The code in this example shows you how to create a ModularRobot as well.
- Understanding of simulation and other examples related to simulation will greatly help with your understanding of this example.

Steps to take:
- Create a config pickle file as shown in `make_config_file.py`.
- Obtain the IP address of the core. If you are on the same network, we recommend using the `nmap` tool, if available on your system: `sudo nmap -sP <your ip>`. For CI Group people: the core should be connected to ThymioNet.
- Copy the config file onto the core using `scp <config file path> <user>@<rpi-ip>:~/config.pickle`. The default user and password are `pi` and `raspberry`.
- SSH to the core using `ssh <user>@<rpi-ip>`.
- Run the robot using the following command: `run_brain --config ~/config.pickle --hardware <hardware type>`, where `hardware type` is either `v1` or `v2`.
- To get all possible arguments for the previous command, see `run_brain --help`

Congratulations! The robot should be moving now.
