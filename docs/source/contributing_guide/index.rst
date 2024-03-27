==================
Contributing guide
==================
If you intend to contribute you may find this guide helpful. Contributions are highly appreciated.

----------------------------
Publishing your contribution
----------------------------
If you added something to Revolve2 that you would like to share with other people, you can do so by creating a `pull request <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests>`_ (PR) on `GitHub <https://github.com/ci-group/revolve2/pulls>`_.
The first time you make a contribution to a Revolve2 package your PR should add your name to the ``authors`` list in that package's ``pyproject.toml``.

Note that a general heuristic is, if your addition adds a dependency of another revolve package to the existing dependencies, you might not want to structure it that way.
For a guideline what can depend on what, look at the package diagram on the main page.

----------------------
Developer installation
----------------------
The normal installation guide applies. You should definitely use :ref:`editable mode<installation/index:Editable Mode>`.
Using the ``requirements_dev.txt`` allows you to quickly install all packages in editable mode, by executing: ``pip install -r requirements_dev.txt``.
If you want to uninstall all Revolve2 packages, you can use ``./uninstall.sh``, which uninstall all packages like ``revolve2*``.

----------------------
Continuous integration
----------------------
Github Actions is used for continuous integration(CI). You can find plenty of resources about the CI online. It is located in the revolve directory at ``.github/workflows``.
You cannot directly run the CI configuration locally, but scripts are available to run the tools.

----------
Code tools
----------
Revolve2 code quality is checked by a variety of tools. See the ``codetools`` directory.
The CI runs these tools automatically.
A shorthand for running the tools and applying as many automatic fixes as possible is ``./codetools/fix_all.sh``.
Not all problems can be fixed automatically, but the errors should be self explanatory.
Make sure to regularly run the tools during development (at least mypy), as they can help you detect many mistakes early.

.. list-table:: Revolve2 code tools
   :widths: 1 4
   :header-rows: 1

   * - Tool
     - Description
   * - Black
     - Python code formatting.
   * - Darglint
     - Check if python docstrings match what they are documenting.
   * - Isort
     - Sorts python imports.
   * - Mypy
     - Static type checker for Python.
   * - Pydocstyle
     - Makes docstrings conform to a single style. Mostly used to check for missing docstrings.
   * - Pyflakes
     - Finds simple errors in Python code. Mostly used to check for unused imports.
   * - Sort-all
     - Sorts the ``__all__`` in ``__init__.py`` files.

-------------
Documentation
-------------
This documentation is automatically built by the CI and uploaded to github pages.
Code is analyzed and an API reference is generated programmatically.
You can compile the documentation using the Makefile available at ``./docs`` using ``make -C docs html``.

---------------
Version control
---------------
The codebase is managed through Git. We strictly enforce a `linear history <https://www.bitsnbites.eu/a-tidy-linear-git-history/>`_.
Releases are according to `Semantic Versioning <https://semver.org/>`_.
