============
Optimization
============
The core of Revolve2's optimization framework are processes.
A process is part of program which state can be recovered in case it is stopped midway.
Each process is uniquely identified by an integer that is used to store and load it using a database.
One example process is Revolve2's generic evolutionary algorithm optimizer which is used in the :ref:`Optimize locomotion tutorial<tutorials/optimize_locomotion:Optimizing a modular robot for locomotion using CPPNWIN and Isaac Gym>`.
Another example is the evaluate function called by this optimizer.

A parent class is available for processes that are contained in a class, called ``Process``.
Classes inheriting this class can be instantiated using ``new`` or ``from_database``,
creating a completely new instance of the class or attempting to load the class state from a database, respectively.
Both functions are ``async``.
