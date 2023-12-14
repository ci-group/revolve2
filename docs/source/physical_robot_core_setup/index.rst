===================================
Setting up a new modular robot core
===================================
This tutorial is for researchers who want to set up a new modular robot core.
First check if someone else has already arranged this for you, for example a research lab manager. If so, you should probably look at :ref:`how to create a physical modular robot <creating_a_physical_robot/index:Creating a physical robot>`.
If you simply want to know how to use a pre-installed physical robot, check the example :ref:`physical_robot_remote <getting_started/index:Step 2: Play with the examples>`.

Version 1 (V1) and version 2 (V2) of the hardware exists. For anyone outside of the CI Group, V1 is not available; please look at the V2 modular robot.

----------------------------------
Preparation + Install the Hardware
----------------------------------
Instructions about the assembly of robots can be found :ref:`here <creating_a_physical_robot/index:Creating a physical robot>`.
Firstly make sure to have all the materials necessary to build the physical robot.

For any type of hardware, there are some required items:

* A housing that will contain the rest of the core hardware. Models to print the core housing can be found on the following git repo: `<https://github.com/ci-group/revolve-models>`_.
* A Raspberry Pi (RPi). For V2, this must be a RPi 4.
* A HAT (hardware attached on top), as described in the following hardware specific sections.

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Hardware setup for a V1 Robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**V1 is only available to CI Group.**


For the V1 Robot some specific Hardware is required:

* It is required to use a V1 HAT, available in the CI Group lab.
* A V1 specific battery, available in the CI Group lab.

Attach the HAT onto the RPi and put them into the housing.

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Hardware setup for a V2 Robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
For instructions on  how to set up the hardware of a V2 core, refer to `<https://github.com/ci-group/robohat>`_.

------------------
Setting up the RPi
------------------
This step is the same for all types of hardware.
To set up the RPi follow the steps below carefully.

#. Flash the SD card with the RPi OS (previously Raspbian). Some Important notes:

    * If you are required to pick a username and password, it is recommended to pick :code:`pi` and :code:`raspberry` for easier access. These are the defaults for most RPi installations.
    * Reminder for CI Group people: if you select a keyboard layout, pick `English(US)`. It can always be changed using the `raspi-config` tool on the RPi.
    * When setting up the Wi-Fi connection, select your Country. (CI-Group Wi-Fi: *ThymioNet*)

#. Boot the RPi and get command line access. Here are some general Tips:

    * You can connect a keyboard and screen.
    * Better is to SSH. This works only if you and the RPi are on the same network. For CI Group, connect it to *ThymioNet* Wi-Fi.

        * Hint: SSH is not enabled by default. The simplest way to enable it is using the :code:`raspi-config`.

    * If you want to SSH and don't know the IP of the RPi, you can use: :code:`sudo nmap -sP <your ip>` to find all clients on your network.

---------------------------
Install Revolve2 on the RPi
---------------------------
Setting up Revolve2 on the robot requires different steps, depending on the hardware version. Some general steps for all versions:

#. Set up a global pyenv. This is to prevent changes to the system's Python installation.:

    #. Install required packages using:

        .. code-block:: bash

            sudo apt install -y git libssl-dev libbz2-dev libncurses5-dev libncursesw5-dev libreadline-dev libsqlite3-dev libffi-dev liblzma-dev

        If an `404 error` appears in this step, you most likely are on an old RPi OS.
        To check your os version you can run :code:`cat /etc/os-release`. It should be version :code:`12 Bookworm` or newer.

    #. Install pyenv: :code:`curl https://pyenv.run | bash`
    #. Add :code:`pyenv` to bash:

        .. code-block:: bash

            echo '# Enable pyenv
            export PYENV_ROOT="$HOME/.pyenv"
            command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"
            eval "$(pyenv init -)"
            eval "$(pyenv virtualenv-init -)"
            ' >> ~/.bashrc

    #. Log in and out of the RPi.
    #. Get the right Python version (this takes a long time): :code:`pyenv install 3.11`
    #. Create a global virtualenv: :code:`pyenv virtualenv 3.11 global_env` & :code:`pyenv global global_env`

#. Then install Revolve2 using:

    * V1: :code:`pip install "revolve2-modular_robot_physical[botv1] @ git+https://github.com/ci-group/revolve2.git@<revolve_version>#subdirectory=modular_robot_physical"`.
    * V2: :code:`pip install "revolve2-modular_robot_physical[botv2] @ git+https://github.com/ci-group/revolve2.git@<revolve_version>#subdirectory=modular_robot_physical"`.

#. Test if Revolve2 is properly installed: :code:`run_brain --help`

^^^^^^^^^^^^^^^^^^^
V1 Additional Steps
^^^^^^^^^^^^^^^^^^^
If you use V1 hardware setup requires additional steps:

* V1 used :code:`pigpiod`, which is installed automatically with the :code:`modular_robot_physical[botv1]` package. This library allows the control of the servos attached to the RPi's HAT.
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
    #. Enable and start the service: :code:`sudo systemctl daemon-reload` & :code:`sudo systemctl enable pigpiod` & :code:`sudo systemctl start pigpiod`.
    #. Check if it is running properly using: :code:`sudo systemctl status pigpiod`

    * That's it! Now :code:`pigpiod` will run at startup with a high priority. If you need to adjust the priority later, you can edit the :code:`Nice` value in the service file and restart the service.
