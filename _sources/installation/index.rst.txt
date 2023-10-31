============
Installation
============
Revolve2 consists of multiple Python packages.
Not all packages are required; pick what you need based on your use case. In the following section you will find out what packages you need for your research.

-------------
Prerequisites
-------------
* Python 3.10 or higher.
* Pip. If you do not have pip for python 3.10, take a look at the get-pip script: `<https://pip.pypa.io/en/stable/installation/>`_.
* Virtualenv::

    python3.10 -m pip install virtualenv

----------------------------
Create a virtual environment
----------------------------
Keeping your workspace tidy is important, and `virtual environments <https://docs.python.org/3/library/venv.html>`_ help with that.
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

-------------
Common issues
-------------
* *For Mac users:* Installing Mujoco sometimes does not work properly. Try `pip install mujoco` to see weather you encounter any issues. If it does not install properly it might be necessary to run the command: `export SYSTEM_VERSION_COMPAT=0` before doing the installation of the modules. `See here. <https://github.com/conda-forge/python-feedstock/issues/445#issuecomment-773835866>`_
* Conda-environments are not officially supported and can cause issues with installation. (Especially in combination with Mac systems)

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
Manual installation with editable mode requires you to install the packages in reverse order of dependency, so that all packages are installed in editable mode.
For the correct order refer to the tables *requires* column or look at the `dev_install.sh` script.

If you do not use editable mode, the order of installation is irrelevant.
Each package can be installed using: ::

    pip install <path_to_revolve>/<package_name>

.. list-table:: Revolve2 packages (in alphabetical order)
   :widths: 25 50 25 5
   :header-rows: 1

   * - Package Name
     - Description
     - requires
     -
   * - ci_group
     - Computational Intelligence Group experimentation tools and standards.
     - :code:`modular_robot_simulation`
     - **!**
   * - experimentation
     - Tools for experimentation.
     - \-
     - **!**
   * - modular_robot
     - Everything for defining modular robots.
     - \-
     - **!**
   * - modular_robot_simulation
     - Functionality to define scenes with modular robots in a terrain and simulate them.
     - :code:`modular_robot`, :code:`simulation`
     - **!**
   * - simulation
     - Physics simulation abstraction layer.
     - \-
     - **!**
   * - simulators/mujoco_simulator
     - MuJoCo simulator.
     - :code:`simulation`
     - **!**



-------------
Editable Mode
-------------
During the development of python packages it is crucial to test changes iteratively. To avoid having to constantly :code:`pip uninstall` and :code:`pip install` your package, you can simply use pip`s built in "developer" mode.
If you want to edit Revolve2's code while having it installed, use pip's ``editable mode``::

    pip install -e <package>

Refer to pip's `documentation <https://setuptools.pypa.io/en/latest/userguide/development_mode.html>`_ if you want to dig deeper into the editable mode.