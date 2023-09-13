============
Getting Started
============

Revolve2 is a library that allows you to use evolutionary computing for modular robot evolution and simulation of those robots.
Additionally these modular robots can be built physically and subsequently be controlled by the brain of your simulated robots.
In this tutorial we will go over all steps for you to start researching ASAP.

-------------
Step 1: Installation
-------------
The first step is to install revolve2. For this follow the detailed tutorial from the link below.
Keep in mind that installation can vary depending on the release you choose.For releases < **0.4.0**
use the Installation Guide Old.
.. toctree::
   :maxdepth: 1

   Installation Guide <_include>
   Installation Guide Old <_include_old>

---------------------
Step 2: Play with examples
---------------------
Depending on your topic of research some different functionality in revolve2 might be interesting to you.
One good starting point to get a better understanding of the inner workings of revolve2, are the examples found in the repository.

* To understand the mechanics behind the simulations better look at: `simulate_single_robot`

* If you need a refresh on evolutionary algorithms try to understand the following examples:

    * `simple_ea_xor`and `simple_ea_xor_database`

* To get into the evolution of modular robots brains/ bodies or both there are multiple examples to look at:

    * `robot_bodybrain_ea` for evolving the brain and body of a robot using an evolutionary algorithm

    * `robot_bodybrain_ea_database` for doing the same as above, however with a database to collect simulation data.

    * `robot_brain_cmaes` is a good starting point when you want to evolve only a brain using covariance matrix adaptation evolution strategy (CMA-ES)

    * `robot_brain_cmaes_database` for the same with a database again.

* Evaluating your experiments is crucial. Examples on how you could do that can be found in:

    * `evaluate_multiple_isolated_robots` and `evaluate_single_robot`

* To see how you can structure a experiment look at: `experiment_setup`

* Finally if you want to use your freshly evolved ``ActorController`` on a physical robot look at the following examples:

    * `rpi_controller` to see an example of a mapped ``ActorController``.

    * `rpi_controller_remote` to see how you can connect to your physical robot.

