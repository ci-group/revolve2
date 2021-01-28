################################
# EvoMan FrameWork - V2.0 2021 #
# Author: Karine Miras, DZ     #
# karine.smiras@gmail.com      #
################################
import os

import numpy
import pygame
from pygame.locals import *
import third_party.evoman.game_helper as game_helper
import gym
from third_party.evoman.player import Player

# main class
from third_party.evoman.enemy.enemy import Enemy
from third_party.evoman.evoman_helper import EvomanConfiguration
from third_party.evoman.projectile.enemy_bullet import EnemyBullet
from third_party.evoman.sensors import Sensors


class EvomanEnvironment(gym.Env):

    max_difficulty = 20
    width = 736
    height = 512

    # simulation parameters
    def __init__(self, configuration: EvomanConfiguration):
        self.configuration = configuration
        self.observation_space = None
        self.action_space = None
        self.state = None
        self.measures = None

        # State information
        self.time = 0
        self.ends = 0
        self.done = False

        if self.configuration.headless:
            os.environ["SDL_VIDEODRIVER"] = "dummy"
        else:
            if "SDL_VIDEODRIVER" in os.environ:
                del os.environ["SDL_VIDEODRIVER"]

        # initializes pygame library
        pygame.init()
        self.configuration.print_logs("MESSAGE: Pygame initialized for simulation.")

        self.sensors = Sensors()

        # initializes log file
        if self.configuration.logs and self.configuration.save_logs:
            if not os.path.exists(self.configuration.experiment_name):
                os.mkdir(self.configuration.experiment_name)
            file_aux = open(self.configuration.experiment_name + '/evoman_logs.txt', 'w')
            file_aux.close()

        # initializes sound library for playing mode
        if self.configuration.sound:
            pygame.mixer.init()
            self.configuration.print_logs("MESSAGE: sound has been turned on.")

        self.clock = pygame.time.Clock()  # initializes game clock resource

        self.set_game_flags()

        self.player: Player = None
        self.enemy: Enemy = None
        self.tilemap = None

        self.load_sprites()
        self.start = False

        self.configuration.checks_params()

    def set_game_flags(self):
        # generates screen
        flags = DOUBLEBUF

        if self.configuration.fullscreen:
            flags = flags | FULLSCREEN

        self.screen = pygame.display.set_mode((self.width, self.height), flags)

        self.screen.set_alpha(None)  # disables uneeded alpha
        pygame.event.set_allowed([QUIT, KEYDOWN, KEYUP])  # enables only needed events

    def load_sprites(self):
        self.tilemap = game_helper.load(Enemy.tilemap, self.screen.get_size())  # map

        sprite_enemy = game_helper.SpriteLayer()
        start_cell = self.tilemap.layers['triggers'].find('enemy')[0]
        self.enemy = Enemy((start_cell.px, start_cell.py), EnemyBullet, "static", sprite_enemy)

        # loads player
        sprite_player = game_helper.SpriteLayer()
        start_cell = self.tilemap.layers['triggers'].find('player')[0]
        self.player = Player((start_cell.px, start_cell.py), self.configuration.enemy_number, sprite_player)

        # loads enemy and map
        self.tilemap.layers.append(sprite_enemy)  # enemy
        self.tilemap.layers.append(sprite_player)

    def reset(self):
        self.configuration.print_logs("Reset Evoman")
        self.time = 0
        self.ends = 0

        self.done = False
        self.start = True

        self.update_clock()
        self.load_sprites()

        return self.done, 0.0, self.get_observation()

    def get_observation(self):
        return self.sensors.get(self.player, self.enemy, self.configuration.inputs_coded)

    def update_clock(self):
        # adjusts frames rate for defining game speed
        if self.configuration.clock_precision == "medium":  # medium clock precision
            if self.configuration.speed == 'normal':
                self.clock.tick_busy_loop(30)
            elif self.configuration.speed == 'fastest':
                self.clock.tick_busy_loop()

        else:  # low clock precision
            if self.configuration.speed == 'normal':
                self.clock.tick(30)
            elif self.configuration.speed == 'fastest':
                self.clock.tick()

    def step(self, actions):
        # game timer
        self.time += 1
        # gets fitness for training agents
        fitness = self.fitness_single()
        self.player.set_actions(actions)

        # add to measures
        # checks player life status
        if self.player.life == 0:
            self.ends -= 1

            self.player.kill()  # removes player sprite
            self.enemy.kill()  # removes enemy sprite

            self.done = True

        # checks enemy life status
        if self.enemy.life == 0:
            self.enemy.kill()  # removes enemy sprite
            self.player.kill()  # removes player sprite
            self.done = True

        if not self.configuration.load_player:  # removes player sprite from game
            self.player.kill()

        if not self.configuration.load_enemy:  # removes enemy sprite from game
            self.enemy.kill()

        # game runtime limit
        if self.time >= self.enemy.time_expire:
            self.done = True

        return self.done, fitness, self.get_observation()

    def render(self, mode='human'):

        # Exit condition
        # checks screen closing button
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE or event.type == pygame.QUIT:
                self.done = True

        # updates objects and draws its itens on screen
        self.screen.fill((250, 250, 250))
        self.tilemap.update(10 / 1000., self)
        self.tilemap.draw(self.screen)

        # player life bar
        vbar = int(100 * (1 - (self.player.life / float(self.player.max_life))))
        pygame.draw.line(self.screen, (0, 0, 0), [40, 40], [140, 40], 2)
        pygame.draw.line(self.screen, (0, 0, 0), [40, 45], [140, 45], 5)
        pygame.draw.line(self.screen, (150, 24, 25), [40, 45], [140 - vbar, 45], 5)
        pygame.draw.line(self.screen, (0, 0, 0), [40, 49], [140, 49], 2)

        # enemy life bar
        vbar = int(100 * (1 - (self.enemy.life / float(self.enemy.max_life))))
        pygame.draw.line(self.screen, (0, 0, 0), [590, 40], [695, 40], 2)
        pygame.draw.line(self.screen, (0, 0, 0), [590, 45], [695, 45], 5)
        pygame.draw.line(self.screen, (194, 118, 55), [590, 45], [695 - vbar, 45], 5)
        pygame.draw.line(self.screen, (0, 0, 0), [590, 49], [695, 49], 2)

        # checks enemy life status
        if self.enemy.life == 0:
            self.screen.fill((250, 250, 250))
            self.tilemap.draw(self.screen)

        # updates screen
        pygame.display.flip()

    def close(self) -> bool:
        pygame.quit()
        return True

    # updates environment with backup of current solutions in simulation
    def get_solutions(self):
        return self.solutions

        # method for updating solutions bkp in simulation
    def update_solutions(self, solutions):
        self.solutions = solutions

    # default fitness function for single solutions
    def fitness_single(self):
        return 0.9 * (100 - self.get_enemylife()) + 0.1 * self.get_playerlife() - numpy.log(self.get_time())

    # default fitness function for consolidating solutions among multiple games
    def cons_multi(self, values):
        return values.mean() - values.std()

    # measures the energy of the player
    def get_playerlife(self):
        return self.player.life

    # measures the energy of the enemy
    def get_enemylife(self):
        return self.enemy.life

    # gets run time
    def get_time(self):
        return self.time

    def life_penalty(self):
        return self.max_difficulty / self.configuration.level

    def get_boundaries(self):
        # [left, right, bottom, top]
        return [1, self.width-1, 1, self.height-1]
