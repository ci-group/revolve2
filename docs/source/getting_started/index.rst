===============
Getting Started
===============

Revolve2 is research software designed for the optimization of modular robot designs, both in terms of their physical structures (bodies) and their behaviour (brains). It primarily employs evolutionary algorithms for optimization, although other techniques can be used as well. Furthermore, Revolve2 offers capabilities to test these robots in simulation as well as in the real-world.

The software comes with several packages. Depending on your research needs, you may choose to work with a specific subset of these packages.
This tutorial will guide you through the key steps to initiate your studies and research using Revolve2 efficiently.

--------------------
Step 1: Installation
--------------------
The first step is to install Revolve2. For this follow the detailed tutorial from :doc:`Installation Guide </installation/index>`.
Keep in mind that installation can vary depending on the release you choose. This documentation matches the latest release of Revolve2.

------------------------------
Step 2: Play with the examples
------------------------------
Based on your research topic, certain functionalities in Revolve2 may be of interest to you.
A good starting point to delve deeper into Revolve2's inner workings is the example directory found in the repository.
Some examples come in two versions: one with and one without a database.
It is recommended to explore the examples without the database first to better grasp the significance of this addition.

.. list-table:: Revolve2 examples
   :widths: 25 50 25
   :header-rows: 1

   * - Example
     - Contents
     - Prequisite examples
   * - **Simulation**
     - 
     - 
   * - simulate single robot
     - Simulate and visualize a single modular robot.
     - \-
   * - evaluate single robot
     - Evaluate a robot by simulating it and calculating its displacement.
     - :code:`simulate single robot`
   * - evaluate multiple isolated robots
     - Evaluate multiple non-interacting robots concurrently.
     - :code:`evaluate single robot`
   * - evaluate multiple interacting robots
     - Evaluate multiple interacting robots.
     - :code:`evaluate single robot`
   * - evaluate multiple isolated robots
     - Evaluate multiple non-interacting robots concurrently.
     - :code:`evaluate single robot`
   * - custom brain
     - Design your own brain and test it in simulation.
     - :code:`simulate single robot`
   * - custom terrain
     - Design your own terrain and test it in simulation
     - :code:`simulate single robot`
   * - **Experiments**
     - 
     - 
   * - experiment setup
     - Set up your first experiment.
     - \-
   * - simple ea xor
     - Use an evolutionary algorithm to train a neural network to perform XOR.
     - :code:`experiment setup`
   * - simple ea xor database
     - Similar to :code:`simple ea xor`, but now using a database to store experiment results.
       Learn how to read the database for e.g. plotting or recovering crashed experiments.
     - :code:`simple ea xor`
   * - robot bodybrain ea
     - Optimize the bodies and brains of modular robots using an evolutionary algorithm.
     - :code:`evaluate multiple isolated robots`, :code:`simple ea xor`
   * - robot bodybrain ea database
     - Similar to :code:`robot bodybrain ea`, but now using a database to store experiment results.
     - :code:`robot bodybrain ea`, :code:`simple ea xor database`
   * - robot brain cmaes
     - Optimize the brains of modular robots using CMA-ES.
     - :code:`evaluate multiple isolated robots`, :code:`simple ea xor`
   * - robot brain cmaes database
     - Similar to :code:`robot brain cmaes`, but now using a database to store experiment results.
     - :code:`robot brain cmaes`, :code:`simple ea xor database`
   * - map to physical robot
     - Learn how to use your evolved robots in the physical world, with and without :code:`RemoteController`
     - :code:`simulate single robot`
