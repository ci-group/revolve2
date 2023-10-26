===============================
Setting up a new Physical Robot
===============================
This tutorial is aimed towards developers, who aim to set up a new robot.
If you simply want to know how to run a physical robot, check the examples.

-----------
Preparation
-----------
Firstly make sure to have all materials necessary to build the physical robot.
Models to print the Modules can be found on the following git repo: `<https://github.com/ci-group/revolve-models>`_.
Other items needed:

* Rasberry Pi (rpi)
* SD-card for the rpi-OS
* suitable Battery
* HAT for the rpi
* cables

------------------
Setting up the RPI
------------------
To setup the RPI follow the steps below carefully.

#. Flash the SD-card with the RPI-OS. Some Important notes:
    * Make sure to select a appropriate Keyboard Layout (preferably English(US)).

    * When setting up the WIFI connection, select your Country.

#. Once the SD-card is flashed, put it into the rpi, mounted on the robot and connect to it. If you dont know the IP-address do the following:
    * :code:`ip a` to find your ip in the **same** network.

    * :code:`sudo nmap -sP <your ip>` to find all clients on your network.

    * Find your rpi and connect via ssh.

#. Setting up Revolve2 on the robot requires different steps, depending on the hardware version. Some general steps for all versions:
    * In order to install Revolve2 you either have to create a venv or use a global pyenv, since rpi doesnt allow to pip install packages globally.
        * Make and activate a venv on the rpi: :code:`python -m venv .venv` & :code:`source .venv/bin/activate`

        * For :code:`pyenv` follow the steps below:
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

    * Then install using :code:`pip install <dir>/revolve2/modular_robot_physical[bot<hardware version>]`, as example: :code:`[botv1]`.

#. If you use V1 hardware setup requires additional steps:
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

        #. That's it! Now :code:`pigpiod` will run at startup with a high priority. If you need to adjust the priority later, you can edit the :code:`Nice` value in the service file and restart the service.


