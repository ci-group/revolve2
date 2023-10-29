===============================
Setting up a new Physical Robot
===============================
This tutorial is aimed towards developers, who aim to set up a new robot.
If you simply want to know how to run a physical robot, check the examples.

-----------
Preparation
-----------
Firstly make sure to have all materials necessary to build the physical robot.
For **CI-Group:**
Models to print the proprietory Modules can be found on the following git repo: `<https://github.com/ci-group/revolve-models>`_.

--------------------
Install the Hardware
--------------------
For any type of hardware there are some required items:

* A controller such as a RPi.
* A HAT for the RPi to connect servos and other extensions to.
* Modules, Servos and extensions that have to be mounted.
* Cables and a Battery

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Hardware setup for a V1 Robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**This is only applicable to the CI-Group specific V1 design.**
Print the Modules and install the RPi in the Head compartment.
Assemble the robot and connect the Servos to the HAT.
More instructions can be found in the *how-to-build-a-modular-robot-guide*.

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Hardware setup for a V2 Robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**This is only applicable to the CI-Group specific V2 design.**
To build a V2 robot print all the necessary Modules and mount the RPi in the Head Compartment.
For instructions about the HAT and the Robot architecture itself refer to `<https://github.com/ci-group/robohat>`_.
Assemble the robot and connect the Servos to the HAT.

------------------
Setting up the RPi
------------------
This step is the same for all types of hardware.
To setup the RPI follow the steps below carefully.

#. Flash the SD-card with the RPi OS (previously Raspbian). Some Important notes:
    * Make sure to select a appropriate Keyboard Layout (preferably English(US)).
    * When setting up the WIFI connection, select your Country. (CI-Group WIFI: *Thymionet*)

#. Get CLI-access to the RPi
    * If you want to SSH and dont know the IP of the RPi use: :code:`sudo nmap -sP <your ip>` to find all clients on your network.

---------------------------
Install Revolve2 on the RPi
---------------------------
Setting up Revolve2 on the robot requires different steps, depending on the hardware version. Some general steps for all versions:
    * Setup a global pyenv. (This is necessary because RPis dont allow global pip installs):
        #. Install pyenv: :code:`curl https://pyenv.run | bash` & :code:`git clone https://github.com/pyenv/pyenv-virtualenv.git $(pyenv root)/plugins/pyenv-virtualenv`
        #. Add :code:`pyenv` to bash:
            .. code-block:: bash

                echo '# Enable pyenv
                export PYENV_ROOT="$HOME/.pyenv"
                command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"
                eval "$(pyenv init -)"
                eval "$(pyenv virtualenv-init -)"
                ' >> ~/.bashrc
        #. Log in and out of the rpi.
        #. Get the right python version (this takes a long time): :code:`pyenv install 3.11`
        #. create a global virtualenv: :code:`pyenv virtualenv 3.11 global_env` & :code:`pyenv global global_env`
    * Clone the Revolve2 branch or repository for your installation.
    * Then install using :code:`pip install <dir>/revolve2/modular_robot_physical`.
        * If you use a CI-Group specific hardware such as V1/ V2 do the install as follows: :code:`pip install <dir>/revolve2/modular_robot_physical[<botv1 or botv2>]`

^^^^^^^^^^^^^^^^^^^
V1 Additional Steps
^^^^^^^^^^^^^^^^^^^
If you use V1 hardware setup requires additional steps:

* Enabling :code:`pigpiod` deamon so it enables with startup (used for the servos).
    #. Setting up a :code:`systemd` service: The modern way to manage startup services on many Linux distributions is via :code:`systemd`. You can set up a service for :code:`pigpiod`.
    #. Create a systemd service file: :code:`sudo nano /etc/systemd/system/pigpiod.service`
    #. Add the following content to the file:
        .. code-block:: bash

            ini
            [Unit]
            Description=Pigpio Daemon
            After=network-online.target

            [Service]
            Type=forking
            ExecStart=/usr/bin/pigpiod -l -n localhost
            ExecStop=/bin/kill -15 $MAINPID
            Nice=-10
            Restart=on-failure
            RestartSec=10

            [Install]
            WantedBy=multi-user.target
    #. Here, the :code:`Nice=-10` line sets a high priority for the daemon (lower values are higher priority, with -20 being the highest priority). The :code:`-l` option in the :code:`ExecStart` line tells :code:`pigpiod` to only listen on the localhost interface. The :code:`-n localhost` option ensures that pigpiod only runs if it can connect to localhost (preventing certain failure cases).
    #. Enable and start the service: :code:`sudo systemctl daemon-reload` & :code:`sudo systemctl enable pigpiod` & :code:`sudo systemctl start pigpiod`.
    #. Check if it is running properly using: :code:`sudo systemctl status pigpiod`

That's it! Now :code:`pigpiod` will run at startup with a high priority. If you need to adjust the priority later, you can edit the :code:`Nice` value in the service file and restart the service.
