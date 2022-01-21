============
Installation
============
Revolve2 consists of a ``core`` library and supplementary libraries that depend on software not available on PyPI. All supplementary libraries require ``core``.

All libraries are installed as python libraries. If you need to edit revolve2 code, take a look at :ref:`installation/index:Editable Mode`.

--------
Download
--------
Download your preferred version from `<https://github.com/ci-group/revolve2/releases>`_.

------------
Install core
------------

``Core`` has no dependencies and is a pure python library::

    pip install <revolve_path>/core

-------------------------------
Install supplementary libraries
-------------------------------
.. toctree::
   :maxdepth: 1

   Isaac Gym environment <envs/isaacgym>
   Cppn+Neat genotype <genotypes/cppnneat>


-------------
Editable Mode
-------------
If you want to edit revolve's code while having it installed, consider using pip's ``editable mode``::

    pip install <library> -e

Refer to pip's documentation for what this does exactly.