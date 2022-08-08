=================
Development Guide
=================
It is recommended that you use pip's :ref:`editable mode<installation/index:Editable Mode>` when developing.

----------------------
Continuous integration
----------------------
Github Actions is used for CI.

----------
Code tools
----------
Revolve two code quality is checked by a variety of tools.
See the ``codetools`` directory.
The CI runs these tools automatically.

-------------
Documentation
-------------
This documentation is automatically built by the CI and uploaded to github pages.

----------------------------
Programming style and typing
----------------------------
Revolve2 uses the Black formatter and its opinionated style.
This as well as typing and docstrings are enforced by the CI.