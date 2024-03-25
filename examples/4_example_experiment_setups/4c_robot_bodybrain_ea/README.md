In this example you see an experiment that optimizes the bodies and brains of a robots using a simple evolutionary algorithm.
The genotypes for both body and brain are CPPNWIN.

Before starting this tutorial, it is useful to look at the 
`3b_evaluate_multiple_isolated_robots`, and `4a_simple_ea_xor` examples.
It is also nice to understand the concept of a cpg brain and CPPN, although not really needed.

You learn:
- How to optimize the body and brain of a robot using an EA.
- How to use the ModularRobotEvolution object for easy evolutionary control.

To change the parameters of the experiment use `config.py`.<br/>
The evaluation of individual robots is done in `evaluator.py`.<br/>
`genotype.py` houses all representations and functionality behind a robots genotypes.<br/>
In `individual.py` you find a class that stores individual robots.

To visualize the evolved robots, use `rerun.py` with the pickled genotype you got from evolution.
