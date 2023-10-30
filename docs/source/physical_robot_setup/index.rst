===================================
Setting up a new modular robot core
===================================
This tutorial is for maintainers from the CI Group who want to set up a new modular robot core.
The following tutorial does not touch the matter of building a complete robot, it just covers the setup for the **core**.
If you simply want to know how to use a pre-installed physical robot, check the examples. This is most likely the case if you are a student.

For anyone outside of the CI Group, please look at the v2 modular robot.

-----------
Preparation
-----------
Firstly make sure to have all the materials necessary to build the physical robot.
For **CI-Group:**
Models to print the core can be found on the following git repo: `<https://github.com/ci-group/revolve-models>`_.

^^^^^^^^^^^^^^^^^^^^
Install the Hardware
^^^^^^^^^^^^^^^^^^^^
For any type of hardware, there are some required items:

* A controller such as an RPi (Raspberry Pi).
* A hat (hardware attached on top) for the RPi to connect servos and other extensions (such as sensors or other actuators). Note that for v1 and v2 hats already exist.
* Cables and a Battery

"""""""""""""""""""""""""""""
Hardware setup for a V1 Robot
"""""""""""""""""""""""""""""
**This is only applicable to the CI-Group-specific V1 design.**


Install the RPi in the Head compartment.
Instructions about the assembly of robots can be found in the *how-to-build-a-modular-robot-guide*.

"""""""""""""""""""""""""""""
Hardware setup for a V2 Robot
"""""""""""""""""""""""""""""
**This is only applicable to the CI-Group-specific V2 design.**


Install the RPi in the Head compartment.
Instructions about the assembly of robots can be found in the *how-to-build-a-modular-robot-guide*.
For instructions about the hat and the Robot architecture itself refer to `<https://github.com/ci-group/robohat>`_.

------------------
Setting up the RPi
------------------
This step is the same for all types of hardware.
To set up the RPi follow the steps below carefully.

#. Flash the SD card with the RPi OS (previously Raspbian). Some Important notes:

    * Reminder for CI Group people: if you select a keyboard layout, pick `English(US)`. It can always be changed using the `raspi-config` tool on the RPi.
    * When setting up the Wi-Fi connection, select your Country. (CI-Group Wi-Fi: *ThymioNet*)

#. Boot the RPi and get command line access. Here are some general Tips:

    * You can connect a keyboard and screen.
    * Better is to SSH. This works only if you and the RPi are on the same network. For CI Group, connect it to *ThymioNet* Wi-Fi.
    * If you want to SSH and don't know the IP of the RPi, you can use: :code:`sudo nmap -sP <your ip>` to find all clients on your network.

---------------------------
Install Revolve2 on the RPi
---------------------------
Setting up Revolve2 on the robot requires different steps, depending on the hardware version. Some general steps for all versions:

#. Check if pyenv is installed by default on your RPi. If not follow the next points:
#. Set up a global pyenv. This is to prevent changes to the system's Python installation.:

    #. Install pyenv: :code:`curl https://pyenv.run | bash` & :code:`git clone https://github.com/pyenv/pyenv-virtualenv.git $(pyenv root)/plugins/pyenv-virtualenv`
    #. Add :code:`pyenv` to bash:

        .. code-block:: bash

            echo '# Enable pyenv
            export PYENV_ROOT="$HOME/.pyenv"
            command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"
            eval "$(pyenv init -)"
            eval "$(pyenv virtualenv-init -)"
            ' >> ~/.bashrc

    #. Log in and out of the RPi.
    #. Install required packages using :code:`sudo apt install -y`. The packages are:

        .. code-block:: bash

            libssl-dev
            libbz2-dev
            libncurses5-dev
            libncursesw5-dev
            libreadline-dev
            libsqlite3-dev
            libffi-dev
            liblzma-dev

    #. Get the right Python version (this takes a long time): :code:`pyenv install 3.11`
    #. create a global virtualenv: :code:`pyenv virtualenv 3.11 global_env` & :code:`pyenv global global_env`

#. Then install Revolve2 using :code:`pip install "git+https://github.com/ci-group/revolve2.git@development#egg=revolve2-modular_robot_physical[botv1]&subdirectory=modular_robot_physical"`.

    * If you use a CI-Group specific hardware such as V1/ V2 do the install as follows: :code:`pip install <dir>/revolve2/modular_robot_physical[<botv1 or botv2>]`

^^^^^^^^^^^^^^^^^^^
V1 Additional Steps
^^^^^^^^^^^^^^^^^^^
If you use V1 hardware setup requires additional steps:

* Install :code:`pigpiod` with :code:`pip install pigpiod`. This library allows the control of the servos attached to the RPis AHT.
* Enabling :code:`pigpiod` daemon so it enables at startup (used for the servos).

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
    #. Enable and start the service: :code:`sudo systemctl daemon-reload & :code:`sudo systemctl enable pigpiod` & :code:`sudo systemctl start pigpiod`.
    #. Check if it is running properly using: :code:`sudo systemctl status pigpiod`

    * That's it! Now :code:`pigpiod` will run at startup with a high priority. If you need to adjust the priority later, you can edit the :code:`Nice` value in the service file and restart the service.
