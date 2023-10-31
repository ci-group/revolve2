In this example, you will manually create and run a physical robot using SSH.

You learn:
- How to make a configuration for a physical robot.
- How to use the configuration on the robot.

Requirements:
- Decide whether you are targeting v1 or v2 hardware. If you do not know, it should probably be v1.
- A previously set up modular robot core of the correct hardware type. If these are not provided by colleagues or supervisors, look at the documentation on how to create one yourself.
- The rest of the modular robot body. TODO in the next version we will add a guide to the documentation on how to create a modular robot body. For now, ask the CI Group lab.
- Additionally, understanding the simulation itself is helpful.

Steps to take:
- Create a config pickle file as shown in `make_config_file.py`.
- Obtain the IP address of the core. If you are on the same network  this can be done through `sudo nmap -sP <your ip>`. For CI Group people: the core should be connected to ThymioNet.
- Copy the config file onto the core using `scp <config file path> <user>@<rpi-ip>:~/config.pickle`. The default user and password are `pi` and `raspberry`.
- SSH to the core using `ssh <user>@r<rpi-ip>`.
- Run the robot using the following command: `run_brain --config ~/config.pickle --hardware <hardware type>`, where `hardware type` is either `v1` or `v2`.
- To get all possible arguments for the previous command, see `run_brain --help`

Congratulations! The robot should be moving now.
