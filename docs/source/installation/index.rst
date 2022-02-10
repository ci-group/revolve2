============
Installation
============
Revolve2 consists of multiple smaller Python packages.
Installing the ``core`` package automatically installs all packages required for optimization.
Additionally there are optional supplementary packages that contain varying functionality that is not always required.

If you need to edit revolve2's code, take a look at :ref:`installation/index:Editable Mode`.

-------------
Prerequisites
-------------
* Python 3.8 or higher. If you are using the Isaac Gym environment supplementary package, it requires exactly Python 3.8.
* virtualenv::

    pip3.8 install virtualenv

---------------------
Create an environment
---------------------
Create a directory for your project, then create a virtual environment::

    python3.8 -m virtualenv .venv

Activate the virtual environment::

    source .venv/bin/activate

-------------------
Download the source
-------------------
Download your preferred version from `<https://github.com/ci-group/revolve2/releases>`_.

------------
Install core
------------

``Core`` installs all code and revolve2 packages required for optimization.
It has only PyPI dependencies and is a pure python package::

    pip install <revolve2_path>/core

--------------------------------------------
Install supplementary packages (Optional)
--------------------------------------------
Revolve2 contains additional packages that provide extra functionality. These are fully optional.

.. toctree::
   :maxdepth: 1

   Isaac Gym environment <runners/isaacgym>
   CPPNWIN genotype <genotypes/cppnwin>
   Raspberry Pi controller <rpi_controller>


-------------
Editable Mode
-------------
If you want to edit revolve2's code while having it installed, consider using pip's ``editable mode``::

    pip install <package> -e

Refer to pip's documentation for what this does exactly.