################################
# EvoMan FrameWork - V1.0 2016 #
# Author: Karine Miras         #
# karine.smiras@gmail.com      #
################################

import numpy as np
from gym.envs import game

from third_party.evoman.base.SpriteDefinition import *
from third_party.evoman.character_helper import wall_collisions
from third_party.evoman.playable_character import PlayableCharacter

from third_party.evoman.path import map_path, sound_path
from third_party.evoman.projectile.projectile import Projectile


class Enemy(PlayableCharacter):

    tilemap = map_path + '/map2.tmx'
    time_expire = 1000  # game run limit

    def __init__(self, location, projectile_type: type(Projectile), mode: str, *groups):
        super(Enemy, self).__init__(location, -1, projectile_type, "EnemySprites", mode, *groups)

        self.attack_timer = 0

        #self.controller: SimpleNeuralNetwork = SimpleNeuralNetwork()
        #self.controller_parameters = np.random.random(265)

    def initialize(self, game):
        # puts enemy in random initial position
        if game.configuration.random_initialization:
            self.rect.x = np.random.randint(game.width * 0.15) + game.width * 0.75

    def update(self, dt, game):
        # if the 'start game' marker is 1
        if not game.start:
            return

        if self.attack_timer == 1:
            self.initialize(game)

        attack1, attack2, attack3, attack4 = self.get_attack()

        # increments enemy timer
        self.attack_timer += 1

        self.movement_check()

        # copies last position state of the enemy
        last_position = self.rect.copy()

        # movements of the enemy on the axis x. Happens 2 to each side.
        if attack1 and self.resting:
            self.rect.x += self.direction * self.movement_speed * dt

            # jumps
            if attack2 == 1:
                self.dy = -900
                self.resting = 0

            self.move(dt)

            #  changes the image when enemy jumps
            self.jumping_animation()
        else:
            self.standing_animation()

        # restart enemy timer and turns the enemy around
        if attack3:
            self.attack_timer = 0
            self.set_direction(self.direction * -1)

        """# checks collision of the player with the enemy
        if self.rect.colliderect(game.player.rect):

            # choses what sprite penalise according to config
            if game.contact_hurt == "player":
                game.player.damage((game.level * 1))
            if game.contact_hurt == "enemy":
                game.enemy.damage(game.level * 1)
        """

        # gravity
        self.gravity(dt)

        # controls screen walls and platforms limits agaist enemy.
        new_position = self.rect
        self.resting = False

        wall_collisions(game, new_position, last_position)

        # enemy shoots
        if attack4 and not self.gun_cooldown:
            self.shooting_cooldown = 3
            self.gun_cooldown = 3

            # bullets sound effect
            if game.configuration.sound and game.playermode == "human":
                sound = pygame.mixer.Sound(sound_path + '/scifi011.wav')
                c = pygame.mixer.Channel(3)
                c.set_volume(10)
                c.play(sound)

            # shoots 6 bullets placed in a fixed range
            for bullet_index in range(3):
                bullet_position = list(self.rect.center)
                bullet_position[0] += np.random.normal(25, 25)
                bullet_position[1] -= np.random.normal(25, 25)
                self.fire_projectile(tuple(bullet_position))

        self.update_projectiles(dt, game)

        self.hurt_animation()
        self.shooting_animation()
        self.animation_cooldowns(dt)

    def get_attack(self):
        # defines game mode for player action.
        if self.mode == 'static':  # enemy controlled by static movements

            attack1 = (210 <= self.attack_timer <= 250) or (260 <= self.attack_timer <= 300)

            attack2 = self.attack_timer == 210 or self.attack_timer == 260

            attack3 = self.attack_timer > 300

            attack4 = (self.attack_timer == 40) or (self.attack_timer == 110) or (self.attack_timer == 180)

        else:  # game.enemymode == 'ai':  # player controlled by AI algorithm

            # calls the controller providing game sensors
            pass

            #attack1, attack2, attack3, attack4, _ = self.controller.activate(self.sensors.get(game.enemy, game.player, game.inputs_coded), self.controller_parameters)

        return attack1, attack2, attack3, attack4