### Running the Robot without the RemoteController
* To manually run the physical robot you first have to create a config pickle file as shown in `make_config_file`.
* Copy the config file onto the physical robot using: `scp <config file path> <user>@<rpi-ip>:<desired path>`. (Keep in mind that for this you have to have a properly setup raspberry pi controller on the robot. If you dont -> check the documentation for a tutorial.)
* Run the robot using the following command: `run_brain --config <config file path on rpi> --hardware <hardware type>`
* To get all possible arguments for the previous command just check `run_brain --help`

Congratulations! The robot should be moving now.

### Running the Robot with the RemoteController
**WIP**