import sys
from typing import List

from third_party.evoman.path import experiment_path


class EvomanConfiguration:

    def __init__(self, experiment_name: str = 'test',
                 enemies: List = None,                   # array with 1 to 8 items, values from 1 to 8
                 load_player: bool = True,              # True or False
                 load_enemy: bool = True,               # True or False
                 level: int = 2,                        # integer
                 speed="fastest",                       # normal or fastest
                 inputs_coded: bool = False,            # True or False
                 random_initialization: bool = False,   # True or False
                 sound: bool = False,                   # True or False
                 contact_hurt: str = "player",          # player or enemy
                 logs: bool = True,                     # True or False
                 save_logs: bool = True,                # True or False
                 clock_precision: str = "low",          # low or med
                 time_expire: int = 3000,               # integer
                 solutions: object = None,              # any
                 fullscreen: bool = False,
                 headless: bool = False):
        # initializes parameters
        self.experiment_name = experiment_name
        if enemies is None:
            enemies = [1]
        self.enemies = enemies
        self.enemy_number = enemies[0]  # initial current enemy
        self.load_player = load_player
        self.load_enemy = load_enemy
        self.level = level
        self.speed = speed
        self.inputs_coded = inputs_coded
        self.random_initialization = random_initialization
        self.sound = sound
        self.contact_hurt = contact_hurt
        self.logs = logs
        self.fullscreen = fullscreen
        self.save_logs = save_logs
        self.clock_precision = clock_precision
        self.time_expire = time_expire
        self.solutions = solutions
        self.headless = headless

    def checks_params(self):

        if self.speed not in ('normal', 'fastest'):
            self.print_logs("ERROR: 'speed' value must be 'normal' or 'fastest'.")
            sys.exit(0)

        if self.clock_precision not in ('low', 'medium'):
            self.print_logs("ERROR: 'clockprec' value must be 'low' or 'medium'.")
            sys.exit(0)

        if self.contact_hurt not in ('player', 'enemy'):
            self.print_logs("ERROR: 'contacthurt' value must be 'player' or 'enemy'.")
            sys.exit(0)

        if type(self.time_expire) is not int:
            self.print_logs("ERROR: 'timeexpire' must be integer.")
            sys.exit(0)

        if type(self.level) is not int:
            self.print_logs("ERROR: 'level' must be integer.")
            sys.exit(0)

        if self.level < 1 or self.level > 3:
            self.print_logs("MESSAGE: 'level' chosen is out of recommended (tested).")

    def print_logs(self, msg):
        if self.logs:
            print('\n'+msg)  # prints log messages to screen

            if self.save_logs:  # prints log messages to file
                file_aux = open(os.path.join(experiment_path, self.experiment_name, 'evoman_logs.txt'), 'a')
                file_aux.write('\n\n'+msg)
                file_aux.close()
