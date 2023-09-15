===============
Getting Started
===============

Revolve2 is a library that allows you to use evolutionary computing for modular robot evolution and simulation of those robots.
Additionally these modular robots can be built physically and subsequently be controlled by the brain of your simulated robots.
Revolve2 is modular in nature so depending on your use case you might need only a specific subset of packages.
In this tutorial we will go over all steps for you to start researching ASAP.

--------------------
Step 1: Installation
--------------------
The first step is to install Revolve2. For this follow the detailed tutorial from Installation :doc:`Guide <installation/index>`.
Keep in mind that installation can vary depending on the release you choose. This documentation matches the latest release of Revolve2.

------------------------------
Step 2: Play with the examples
------------------------------
Depending on your topic of research some different functionality in Revolve2 might be interesting to you.
One good starting point to get a better understanding of the inner workings of Revolve2,
are the examples found in the repositories :code:`example` directory.
Some examples have two versions: with and without database. Try the examples with the databases last to understand the role of this addition.


* To understand the mechanics behind the simulations, look at: :code:`simulate_single_robot`

* If you need a refresh on evolutionary algorithms, try to understand the following examples:

    * :code:`simple_ea_xor` and :code:`simple_ea_xor_database`

* To get into the evolution of modular robots brains/ bodies or both there are multiple examples to look at:

    * :code:`robot_bodybrain_ea` for evolving the brain and body of a robot using an evolutionary algorithm.

    * :code:`robot_bodybrain_ea_database` for doing the same as above, but with a database to collect simulation data.

    * :code:`robot_brain_cmaes` is a good starting point when you want to evolve only a brain using the covariance matrix adaptation evolution strategy (CMA-ES)
    * :code:`robot_brain_cmaes_database` for the previous example, but again with an added database.

* Evaluating your robots is crucial for your experiments. In :code:`simulate_single_robot` we just simulated a single robot but did nothing with results. Examples on how you can evaluate fitness or results in general, can be found in:

    * :code:`evaluate_single_robot` shows how to evaluate a single robots fitness (defined by displacement from the origin).
    * :code:`evaluate_multiple_isolated_robots` shows how to evaluate the fitness of multiple robots, in parrallel.

* To see how you can structure an experiment, look at: :code:`experiment_setup`

* Finally, if you want to use your freshly evolved ``ActorController`` on a physical robot, look at the following examples:

    * :code:`rpi_controller` to see an example of a serialized ``ActorController``.

    * :code:`rpi_controller_remote` to see how you can connect to your physical robot.

