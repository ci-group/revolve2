============
Installation
============
Revolve2 consists of multiple smaller Python packages.
Not all packages are required; pick what you need based on your use case. In the following section you will find out what packages you need for your research.

-------------
Prerequisites
-------------
* Python 3.10 or higher.
* Pip. If you do not have pip for python 3.10, take a look at the get-pip script: `<https://pip.pypa.io/en/stable/installation/>`_.
* *For Mac users:* Installing Mujoco sometimes does not work properly. Try `pip install mujoco` to see weather you encounter any issues. If it does not install properly it might be necessary to run the command: `export SYSTEM_VERSION_COMPAT=0` before doing the installation of the modules. `See here. <https://github.com/conda-forge/python-feedstock/issues/445#issuecomment-773835866>`_
* Keep in mind Conda-environments are not officially supported and can cause issues with installation. (Especially in combination with Mac systems)
* Virtualenv::

    python3.10 -m pip install virtualenv

---------------------
Create an environment
---------------------
Keeping your workspace tidy is important, `virtual environments <https://docs.python.org/3/library/venv.html>`_ help with that.
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

----------------
Install packages
----------------
**For Students of the CI group:**
Packages with a **!** next to them are most likely required.
You can also use a shortcut to install all required packages: ::

    sh student_install.sh

This script installs all required packages in editable mode.

Revolve2 contains multiple packages that provide specific functionality. These packages contain functionality for simulations, modular robot description, hardware control and optimization / EA.
If you need to edit components of Revolve2, or you want to add new features, it is recommended that you use :ref:`installation/index:Editable Mode`.
Manual installation with editable mode requires you to install the packages in order of dependency, so that all packages are installed in editable mode.
For the correct order refer to the tables *requires* column or look at the `dev_install.sh` script.

Installing them without edible mode does not require the right order since it will automatically install dependencies.
Each package can be installed using: ::

    pip install <package_name>

.. list-table:: Revolve2 packages
   :widths: 25 50 25 5
   :header-rows: 1

   * - Package Name
     - Functionality
     - requires
     -
   * - actor_controller
     - This package provides a controller interface as well as already implemented controllers for the modular robots.
     - :code:`serialization`
     - **!**
   * - ci_group
     - This package provides Revolve2 with CI group specific revolve configuration and helper tools.
     - :code:`simulation` & :code:`modular_robot`
     - **!**
   * - experimentation
     - This package provides functionality for experiments, such as optimization techniques, genotype operations and database usage.
     - :code:`modular_robot`
     - **!**
   * - modular_robot
     - This package provides Revolve2 with all functionality around the robots and their modules.
     - :code:`simulation` & :code:`actor_controller`
     - **!**
   * - rpi_controller
     - This package allows to run a Revolve2 ``ActorController`` on a physical robot, with the same behavior as in the simulations.
     - :code:`actor_controller`
     -
   * - rpi_controller_remote
     - This package allows to remotely connect to, and run a physical robot using SSH.
     - :code:`serialization`
     -
   * - serialization (Deprecated)
     - This package does what the name says. (Will be removed soon)
     -
     - **!**
   * - simulation
     - This package provides an abstraction layer for physics simulators. Other packages provide a simulator-specific implementation, such as for MuJoCo.
     - :code:`actor_controller`
     - **!**
   * - simulators/mujoco
     - This package provides simulation using the MuJoCo simulator.
     - :code:`simulation` & :code:`modular_robot`
     - **!**



-------------
Editable Mode
-------------
When developing element in python packages it is crucial to test changes iteratively. To avoid having to constantly :code:`pip uninstall` and :code:`pip install`, you can simply use pip`s built in "developer" mode.
If you want to edit Revolve2's code while having it installed, use pip's ``editable mode``::

    pip install -e <package>

Refer to pip's `documentation <https://setuptools.pypa.io/en/latest/userguide/development_mode.html>`_ if you want to dig deeper into the editable mode.