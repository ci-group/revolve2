import gzip
import pickle
import sys


# writes all variables related to game state into log
def state_to_log(environment):
    environment.print_logs("########## Simulation state - INI ###########")
    if environment.solutions is None:
        environment.print_logs("# solutions # : EMPTY ")
    else:
        environment.print_logs("# solutions # : LOADED ")

    environment.print_logs("# sensors # : " + str(environment.get_num_sensors()))
    environment.print_logs(" ------  parameters ------  ")
    environment.print_logs("# contact hurt (training agent) # : " + environment.contact_hurt)

    environment.print_logs("multiple mode: " + str(environment.multiple_mode))

    en = ''
    for e in environment.enemies:
        en += ' ' + str(e)
    environment.print_logs("enemies list:" + en)

    environment.print_logs("current enemy: " + str(environment.enemy_number))
    environment.print_logs("player mode: " + environment.player_mode)
    environment.print_logs("enemy mode: " + environment.enemy_mode)
    environment.print_logs("level: " + str(environment.level))
    environment.print_logs("clock precision: " + environment.clock_prec)
    environment.print_logs("inputs coded: " + str(environment.inputs_coded))
    environment.print_logs("random initialization: " + str(environment.random_initialization))
    environment.print_logs("expiration time: " + str(environment.time_expire))
    environment.print_logs("speed: " + environment.speed)
    environment.print_logs("load player: " + str(environment.load_player))
    environment.print_logs("load enemy: " + str(environment.load_enemy))
    environment.print_logs("sound: " + str(environment.sound))
    environment.print_logs("overture time: " + str(environment.overture_time))
    environment.print_logs("logs: " + str(environment.logs))
    environment.print_logs("save logs: " + str(environment.save_logs))
    environment.print_logs("########## Simulation state - END ###########")


# exports current environment state to files
def save_state(environment):
    # saves configuration file for simulation parameters
    file_aux = open(environment.experiment_name + '/evoman_paramstate.txt', 'w')
    enemies = ''
    for e in environment.enemies:
        enemies += ' ' + str(e)
    file_aux.write("\nenemies " + enemies)
    file_aux.write("\ntime_expire " + str(environment.time_expire))
    file_aux.write("\nlevel " + str(environment.level))
    file_aux.write("\nenemy_number " + str(environment.enemy_number))
    file_aux.write("\noverture_time " + str(environment.overture_time))
    file_aux.write("\nplayer_mode " + str(environment.player_mode))
    file_aux.write("\nenemy_mode " + environment.enemy_mode)
    file_aux.write("\ncontact_hurt " + environment.contact_hurt)
    file_aux.write("\nclock_prec " + environment.clock_prec)
    file_aux.write("\ninputs_coded " + str(environment.inputs_coded))
    file_aux.write("\nrandom_initialization " + str(environment.random_initialization))
    file_aux.write("\nmultiple_mode " + str(environment.multiple_mode))
    file_aux.write("\nspeed " + environment.speed)
    file_aux.write("\nload_player " + str(environment.load_player))
    file_aux.write("\nload_enemy " + str(environment.load_enemy))
    file_aux.write("\nsound " + str(environment.sound))
    file_aux.write("\nlogs " + str(environment.logs))
    file_aux.write("\nsave_logs " + str(environment.save_logs))
    file_aux.close()

    # saves state of solutions in the simulation
    file = gzip.open(environment.experiment_name + '/evoman_solstate', 'w', compresslevel=5)
    pickle.dump(environment.solutions, file, protocol=2)
    file.close()

    environment.print_logs("MESSAGE: state has been saved to files.")


# loads a state for environment from files
def load_state(environment):
    try:
        # loads parameters
        state = open(environment.experiment_name + '/evoman_paramstate.txt', 'r')
        state = state.readlines()
        for idp, p in enumerate(state):
            pv = p.split(' ')

            if idp > 0:  # ignore first line
                if idp == 1:  # enemy list
                    en = []
                    for i in range(1, len(pv)):
                        en.append(int(pv[i].rstrip('\n')))
                    environment.update_parameter(pv[0], en)
                elif idp < 6:  # numeric params
                    environment.update_parameter(pv[0], int(pv[1].rstrip('\n')))
                else:  # string params
                    environment.update_parameter(pv[0], pv[1].rstrip('\n'))

        # loads solutions
        file = gzip.open(environment.experiment_name + '/evoman_solstate')
        environment.solutions = pickle.load(file, encoding='latin1')
        environment.print_logs("MESSAGE: state has been loaded.")

    except IOError:
        environment.print_logs("ERROR: could not load state.")


# method for updating simulation parameters
def update_parameter(environment, name, value):

    if type(value) is str:
        exec('environment.' + name + "= '" + value+"'")
    else:
        exec('environment.' + name + "= " + str(value))

    environment.print_logs("PARAMETER CHANGE: " + name + " = " + str(value))
