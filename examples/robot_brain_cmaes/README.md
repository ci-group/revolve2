Here you will set up an experiment that optimizes the brain of a given robot body using CMA-ES.
As the body is static, the genotype of the brain will be a fixed length real-valued vector.

Before starting this tutorial, it is useful to look at the `experiment_setup` and `evaluate_multiple_isolated_robots` examples.
It is also nice to understand the concept of a cpg brain, although not really needed.

You learn:
- How to optimize the brain of a robot using CMA-ES.

To change the parameters of the experiment use `config.py`.<br/>
The evaluation of individual robots is done in `evaluator.py`.<br/>

To visualize the evolved robots, use `rerun.py` with the *PARAMS* you got from evolution.
