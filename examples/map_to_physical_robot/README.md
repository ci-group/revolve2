### Running the Robot without the RemoteController
Keep in mind that for this you have to have a properly setup raspberry pi controller on the robot. If you don`t -> check the documentation for a tutorial.

* To manually run the physical robot you first have to create a config pickle file as shown in `make_config_file.py`.
* Copy the config file onto the RPi using: `scp <config file path> <user>@<rpi-ip>:<desired path>`.
* Run the robot using the following command: `run_brain --config <config file path on rpi> --hardware <hardware type>`
* To get all possible arguments for the previous command, see `run_brain --help`

Congratulations! The robot should be moving now.
