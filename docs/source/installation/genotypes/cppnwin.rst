=====================================
CPPNWIN genotype supplemental package
=====================================
This package provides a modular robot genotype using CPPNWIN.

-------------
Prerequisites
-------------
This package depends on a patched version of the open source MultiNEAT library, available at `<https://github.com/ci-group/MultiNEAT>`_.
MultiNEAT is automatically installed by the CPPNWIN package, but you need to manually install one of its dependencies, the `cereal <https://uscilab.github.io/cereal/>`_ C++ library.
On Ubuntu this can be done by running::

    sudo apt install libcereal-dev

On other systems you can check yourself if a package is available. If it not, you can install cereal from source. It is a header-only library, so there is no need to build cereal.

-------
Install
-------
Install the package::

    pip install <revolve2_path>/genotypes/cppnwin