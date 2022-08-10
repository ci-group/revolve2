============
Installation
============
Revolve2 consists of multiple smaller Python packages.
The ``core`` package and it's automatically installed dependencies contain everything required for optimization as well as other features that are trivial to install.
Additionally there are optional supplementary packages that contain varying functionality that is not always required and may be more difficult to install.

-------------
Prerequisites
-------------
* Python 3.8 or higher. If you are using the Isaac Gym environment supplementary package, it requires exactly Python 3.8.
* Pip. If you do not have pip for python 3.8, take a look at the get-pip script: `<https://pip.pypa.io/en/stable/installation/>`_.
* Virtualenv::

    python3.8 -m pip install virtualenv

---------------------
Create an environment
---------------------
Create a directory for your project, then create a virtual environment::

    python3.8 -m virtualenv .venv

Activate the virtual environment::

    source .venv/bin/activate

Leave the ``.venv`` directory as is and do not manually edit it, unless you know what you are doing.

-------------------
Download the source
-------------------
Download your preferred version from `<https://github.com/ci-group/revolve2/releases>`_.
If you need to edit Revolve2 itself to add new features, it is recommended to instead create a fork and clone using git.

------------
Install core
------------
``Core`` installs all code and revolve2 packages required for optimization.
It has only PyPI dependencies and is a pure python package::

    pip install <revolve2_path>/core

If you need to edit Revolve2 itself to add new features, it is recommended that you use :ref:`installation/index:Editable Mode`.

--------------------------------------------
Install supplementary packages (Optional)
--------------------------------------------
Revolve2 contains additional packages that provide extra functionality. These are fully optional.

.. toctree::
   :maxdepth: 1

   Isaac Gym physics runner <runners/isaacgym>
   Mujoco physics runner <runners/mujoco>
   CPPNWIN genotype <genotypes/cppnwin>
   Raspberry Pi actor controller <rpi_controller>

-------------
Editable Mode
-------------
If you want to edit revolve2's code while having it installed, consider using pip's ``editable mode``::

    pip install <package> -e

Refer to pip's documentation for what this does exactly.