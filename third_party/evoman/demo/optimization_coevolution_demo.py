###############################################################################
# EvoMan FrameWork - V1.0 2016  			                      			  #
# DEMO : Neuroevolution - Genetic Algorithm with neural network.              #
# Author: Karine Miras        			                             		  #
# karine.smiras@gmail.com     				                     			  #
###############################################################################

# imports framework
import sys
sys.path.insert(0, '..')
from environment import Environment
from demo_controller import player_controller, enemy_controller

# imports other libs
import time
import numpy as np
import os

import rootpath

os.chdir(os.path.join(rootpath.detect(), 'third_party', '..'))


class TrainingEnvironment(Environment):

	# implements fitness function
	def fitness_single(self):
		return 0.9 * (100 - self.get_enemylife()) + 0.1 * self.get_playerlife() - np.log(self.get_time())



# runs simulation
def simulation(env, x):
	f, _, _, _ = env.play(pcont=x)
	return f


# evaluation
def evaluate(x):
	return np.array(list(map(lambda y: simulation(env, y), x)))


# normalizes
def norm(x,pfit_pop):

	if (max(pfit_pop) - min(pfit_pop)) > 0:
		x_norm = (x - min(pfit_pop))/(max(pfit_pop) - min(pfit_pop))
	else:
		x_norm = 0

	if x_norm <= 0:
		x_norm = 0.0000000001
	return x_norm


# tournament
def tournament(pop, fit_pop):
	c1 = np.random.randint(0, pop.shape[0], 1)
	c2 = np.random.randint(0, pop.shape[0], 1)
	if fit_pop[c1] > fit_pop[c2]:
		return pop[c1][0]
	else:
		return pop[c2][0]


# limits
def limits(x):
	if x > dom_u:
		return dom_u
	elif x < dom_l:
		return dom_l
	else:
		return x


# crossover
def crossover(pop, fit_pop):

	total_offspring = np.zeros((0, n_vars))

	for p in range(0, pop.shape[0], 2):
		p1 = tournament(pop, fit_pop)
		p2 = tournament(pop, fit_pop)

		n_offspring = np.random.randint(1, 3+1, 1)[0]
		offspring = np.zeros((n_offspring, n_vars))

		for f in range(0, n_offspring):

			cross_prop = np.random.uniform(0, 1)
			offspring[f] = p1*cross_prop+p2 * (1 - cross_prop)

			# mutation
			for i in range(0,len(offspring[f])):
				if np.random.uniform(0, 1) <= mutation:
					offspring[f][i] = offspring[f][i]+np.random.normal(0, 1)

			offspring[f] = np.array(list(map(lambda y: limits(y), offspring[f])))

			total_offspring = np.vstack((total_offspring, offspring[f]))

	return total_offspring


def evolution(pop, fit_pop):

	offspring = crossover(pop, fit_pop)  # crossover
	fit_offspring = evaluate(offspring)   # evaluation
	pop = np.vstack((pop, offspring))
	fit_pop = np.append(fit_pop, fit_offspring)

	# selection
	fit_pop_cp = fit_pop
	fit_pop_norm = np.array(list(map(lambda y: norm(y, fit_pop_cp), fit_pop))) # avoiding negative probabilities, as fitness is ranges from negative numbers
	probs = (fit_pop_norm)/(fit_pop_norm).sum()
	chosen = np.random.choice(pop.shape[0], npop, p=probs, replace=False)
	chosen = np.append(chosen[1:], np.argmax(fit_pop))

	pop = pop[chosen]
	fit_pop = fit_pop[chosen]

	return pop, fit_pop


experiment_name = 'co_demo'
if not os.path.exists(experiment_name):
	os.makedirs(experiment_name)

n_hidden_neurons = 10

# initializes simulation for coevolution evolution mode.
env = TrainingEnvironment(experiment_name=experiment_name, enemies=[2], playermode="ai", level=2, speed="fastest",
						  player_controller=player_controller(n_hidden_neurons), enemy_controller="static")

env.state_to_log()  # checks environment state

####   Optimization for controller solution (best genotype/weights for perceptron phenotype network): Ganetic Algorihm    ###
ini = time.time()  # sets time marker

# number of weights for multilayer with 10 hidden neurons
n_vars = (env.get_num_sensors()+1)*n_hidden_neurons + (n_hidden_neurons+1)*5

# genetic algorithm params
dom_u = 1
dom_l = -1
npop = 3
gens = 3
mutation = 0.2
last_best = 0

print('\nNEW EVOLUTION\n')

pop_p = np.random.uniform(dom_l, dom_u, (npop, n_vars))
fit_pop_p = evaluate(pop_p)
solutions = [pop_p, fit_pop_p]
env.update_solutions(solutions)

# evolution
for i in range(1, gens):

	pop_p, fit_pop_p = evolution(pop_p, fit_pop_p)
	print('\n GEN ', i, ' evolving ', env.contacthurt, ' - player mean ', np.mean(fit_pop_p))

	# saves simulation state
	solutions = [pop_p, fit_pop_p]
	env.update_solutions(solutions)
	env.save_state()


fim = time.time()  # prints total execution time for experiment
print('\nExecution time: '+str(round((fim-ini)/60))+' minutes \n')

file = open(experiment_name+'/neuroended', 'w')  # saves control (simulation has ended) file for bash loop file
file.close()

env.state_to_log()  # checks environment state
