import os

from third_party.popi4sb.algorithms.differential_evolution import RevDE, DE, RevDEknn
from third_party.popi4sb.algorithms.evolutionary_model import EvolutionaryExperiment
from third_party.popi4sb.algorithms.evolutionary_strategies import EvolutionaryStrategies
from third_party.popi4sb.algorithms.model_building_evolution import EstimationDistributionAlgorithmKNN, \
    EstimationDistributionAlgorithm
from third_party.popi4sb.simulators.ode_simulator import PySCESSimulatorODE

from third_party.popi4sb.utils.config import Config


def run(dir_method, json_method,
        dir_model, json_model,
        dir_results,
        dir_solver, json_solver,
        dir_data=None, file_data=None,
        exp_sign='_exp_'):

    config_method = Config(os.path.join(dir_method, json_method))
    config_model = Config(os.path.join(dir_model, json_model))

    config_solver = Config(os.path.join(dir_solver, json_solver))

    # Init method
    # -get all classes in the file
    classes = [EvolutionaryStrategies, DE, RevDE, RevDEknn, EstimationDistributionAlgorithm, EstimationDistributionAlgorithmKNN]
    # -check whether the provided name is available
    assert config_method.config['method_name'] in classes, 'Wrong name of the method! Please pick one of the following methods: {}'.format(classes)

    experiment_path = dir_results + "/" + exp_sign + config_method.config['method_name'] + '_'

    simulator = PySCESSimulatorODE(config_model, config_method, config_solver, dir_data, file_data, dir_model, experiment_path)

    # -initialize the appropriate class
    module = __import__("algorithms.population_optimization_algorithms", fromlist=[config_method.config['method_name']])
    my_class = getattr(module, config_method.config['method_name'])
    algorithm = my_class(config_method.config, config_model.config, simulator.calculate_fitness)

    experiment = EvolutionaryExperiment(algorithm, config_method, experiment_path)
    experiment.run()
