============
Installation
============
Revolve2 consists of multiple smaller Python packages.
These packages are not all required, depending on the usecase. In the following section you will find out what packages you need for your reserch.

-------------
Prerequisites
-------------
* Python 3.10 or higher.
* Pip. If you do not have pip for python 3.10, take a look at the get-pip script: `<https://pip.pypa.io/en/stable/installation/>`_.
* Virtualenv::

    python3.10 -m pip install virtualenv

---------------------
Create an environment
---------------------
Keeping your workspace tidy is important, virtual environments help with that.
Create a directory for your project, then create a virtual environment::

    python3.10 -m virtualenv .venv

Activate the virtual environment::

    source .venv/bin/activate

Leave the ``.venv`` directory as is and do not manually edit it, unless you know what you are doing.

-------------------
Download the source
-------------------
Download your preferred version from `<https://github.com/ci-group/revolve2/releases>`_.
If you need to edit Revolve2 itself to add new features, it is recommended to instead create a fork and clone using git.

--------------------------------------------
Install packages
--------------------------------------------
Revolve2 contains multiple packages that provide specific functionality.
Packages with a **!** next to them are always required for a working revolve2.

One shortcut to manually installing the packages is using: ::

    sh dev_requirements.sh

This script installs all packages in editable mode automatically.
If you choose manual installation, install the packages in order of the table to avoid potential conflicts or missing dependencies.
Each package can be installed using: ::

    pip install <package_name>

If you need to edit revolve2 itself to add new features, it is recommended that you use :ref:`installation/index:Editable Mode`

.. list-table:: revolve2 packages
   :widths: 25 50 5
   :header-rows: 1

   * - Package Name
     - Functionality
     -
   * - ci_group
     - This package provides revolve2 with some auxiliary functions and standard resources that can be easily reused.
     - **!**
   * - simulators/mujoco
     - This package provides revolve2 with the ability to simulate robots in a mujoco environment.
     - **!**
   * - experimentation
     - This package provides revolve2 with essential functionality for experiments, such as optimization techniques, genotype operations and database usage.
     - **!**
   * - rpi_controller_remote
     - This package allows to map a revolve2 ``ActorController`` to physical servos on a Raspberry Pi.
     -
   * - rpi_controller_remote_remote
     - This package allows to remotely connect to, and run a physical robot using SSH.
     -
   * - modular_robot
     - This package provides revolve2 with all functionality around the robots and their modules.
     - **!**
   * - simulation
     - This package provides revolve2 with some functionality for the simulations, such as Actor properties and Abstracted Classes.
     - **!**
   * - actor_controller
     - This package provides revolve2 with premade controllers for the modular robots, and the possibility to add new controllers.
     - **!**
   * - serialization
     - This package does what the name says.
     - **!**

-------------
Editable Mode
-------------
If you want to edit revolve2's code while having it installed, consider using pip's ``editable mode``::

    pip install -e <package>

Refer to pip's documentation for what this does exactly.