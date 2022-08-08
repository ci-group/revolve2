================
CPPNWIN genotype
================
This package provides a modular robot genotype based on CPPNWIN.

-------------
Prerequisites
-------------
This package depends on a patched version of the open source MultiNEAT library, available at `<https://github.com/ci-group/MultiNEAT>`_.
MultiNEAT is automatically installed by the CPPNWIN package, but you need to manually install one of its dependencies: the `cereal <https://uscilab.github.io/cereal/>`_ C++ library.

~~~~~~
Ubuntu
~~~~~~

    sudo apt install libcereal-dev

~~~~~~~~~~~~~~~~~~~~
macOS using Homebrew
~~~~~~~~~~~~~~~~~~~~

    brew install cereal

~~~~~~~~~~~~~
Other systems
~~~~~~~~~~~~~
If cereal is not available as a package on your system you can install it from source.
It is a header-only library, so there is no need to build cereal.

-------
Install
-------
Install the package::

    pip install <revolve2_path>/genotypes/cppnwin

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
macOS using Homebrew on M-series processors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
It could be that Homebrew installed cereal in ``/opt`` instead of the usual location.
See `<https://apple.stackexchange.com/questions/414622/installing-a-c-c-library-with-homebrew-on-m1-macs>`_.
If this is the case, you can tell CPPNWIN where to look for cereal manually::
    
    export CPATH=/opt/homebrew/include
