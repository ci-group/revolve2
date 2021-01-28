################################
# EvoMan FrameWork - V1.0 2016 #
# Author: Karine Miras         #
# karine.smiras@gmail.com      #
################################
import os

import pygame
import rootpath
from gym.envs import game

from third_party.evoman.character_helper import wall_collisions
from third_party.evoman.playable_character import PlayableCharacter
from third_party.evoman.projectile.player_bullet import PlayerBullet

root_path = os.path.join(rootpath.detect(), 'third_party', 'evoman')
image_path = os.path.join(root_path, 'resources/images')
sounds_path = os.path.join(root_path, 'resources/sounds')


# player sprite
class Player(PlayableCharacter):

    def __init__(self, location, mode, *groups):
        super(Player, self).__init__(location, 1, PlayerBullet, 'EvoManSprites', mode, *groups)
        self.actions = None

    def set_actions(self, actions):
        self.actions = actions

    def update(self, dt, game):
        # if the 'start game' marker is 1
        if not game.start:
            return

        if self.actions is None:
            return

        left, right, jump, shoot, release = self.actions
        # if the enemies are not atacking with the freezing atack
        # (prevents player from making any movements or atacking) and also the 'start game' marker is 1.
        if self.frozen:
            game.tilemap.set_focus(self.rect.x, self.rect.y)
            return

        self.movement_check()

        # if the button is released before the jumping maximum height, them player stops going up.
        if release and not self.resting:
            self.dy = 0

        # copies last position state of the player
        old_position = self.rect.copy()

        self.movement(dt, left, right, jump)
        self.gravity(dt)
        self.jumping_animation()

        new_position = self.rect  # copies new (after movement) position state of the player

        # focuses screen center on player
        game.tilemap.set_focus(new_position.x, new_position.y)

        # controls screen walls and platforms limits agaist player
        self.resting = 0
        resting = wall_collisions(game, new_position, old_position)
        if resting:
            self.resting = True  # player touches the floor
            self.dy = 0

        # shoots, limiting time between bullets.
        if shoot and not self.gun_cooldown:
            self.shoot()

        else:
            self.attacked = 0

        # hurt player animation
        self.hurt_animation()

        # shooting animation
        self.shooting_animation()
        self.animation_cooldowns(dt)

        self.update_projectiles(dt, game)

        # kills player in case he touches killers stuff, like spikes.
        for cell in game.tilemap.layers['triggers'].collide(self.rect, 'killers'):
            game.player.life = 0

    def shoot(self):
        self.shooting_cooldown = self.shooting_delay
        self.attacked = True  # marks if the player has atacked enemy

        # creates bullets objects according to the direction.
        self.fire_projectile()

        self.gun_cooldown = 1  # marks time to the bullet for allowing next bullets
