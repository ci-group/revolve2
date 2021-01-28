from abc import abstractmethod
from typing import Dict

import pygame
from gym.envs import game
from pygame.sprite import AbstractGroup

from third_party.evoman.base import SpriteConstants
from third_party.evoman.base.SpriteDefinition import SpriteDefinition
from third_party.evoman.character_helper import get_direction_sprite
from third_party.evoman.path import image_path
from third_party.evoman.projectile.projectile import Projectile
from third_party.evoman.sensors import Sensors


class PlayableCharacter(pygame.sprite.Sprite):

    max_speed = 400
    movement_speed = 200
    jumping_speed = 100

    shooting_delay = 5
    hurt_delay = 5

    width = 43
    height = 59

    # direction = SpriteConstants.RIGHT
    def __init__(self, location, direction, projectile_type: type(Projectile), sprite_name: str, mode: str, *groups: AbstractGroup):
        super().__init__(*groups)
        self.sprite_layer = groups
        self.sprite_definition = SpriteDefinition(image_path + '/' + sprite_name + '.png', 0, 0, self.width, self.height)

        self.direction = None
        self.animation_direction = None
        self.set_direction(direction)

        self.update_sprite(SpriteConstants.STANDING, self.animation_direction)
        self.rect = pygame.rect.Rect(location, self.image.get_size())

        self.projectile_type: type(Projectile) = projectile_type
        self.projectiles: Dict[int, Projectile] = {}

        self.mode: str = mode  # ai or human

        # States
        self.frozen: bool = False
        self.resting: bool = False
        self.attacked: bool = False
        self.in_water: bool = False

        # Cooldowns
        self.gun_cooldown: int = 0
        self.hurt_cooldown: int = 0
        self.shooting_cooldown: int = 0

        # Life
        self.max_life = 100
        self.life = self.max_life

        # Animation rendering
        self.direction: int = direction
        self.animation_iteration: int = 1

        # Animation position / speed
        self.vx = 0
        self.vy = 0
        self.hy = 0
        self.dy = 0

        self.sensors: Sensors = Sensors()

    @abstractmethod
    def get_num_sensors(self):
        pass

    def update_sprite(self, state, direction):
        self.image = self.sprite_definition.getImage(state, direction)

    @abstractmethod
    def update(self, dt, game):
        pass

    def movement_check(self):
        # checks water environment flag to regulate movements speed
        if self.in_water:
            self.vx = 0.5
            self.vy = 0.5
            self.hy = -2000
        else:
            self.vx = 1.0
            self.vy = 1.0
            self.hy = -900

    def movement(self, dt, left, right, jump):
        # movements on the axis x (left)
        if left:
            self.set_direction(-1)
            self.move(dt)
        # movements on the axis x (right)
        elif right:
            self.set_direction(1)
            self.move(dt)
        else:
            self.standing_animation()

        # if player is touching the floor, he is allowed to jump
        if self.resting == 1 and jump == 1:
            self.dy = self.hy

    def gravity(self, dt):
        self.dy = min(self.max_speed, self.dy + self.jumping_speed)
        self.rect.y += self.dy * dt * self.vy

    def move(self, dt):
        self.rect.x += self.direction * self.movement_speed * dt * self.vx

        # animation, running player images alternation
        if self.animation_iteration == 1:
            self.update_sprite(SpriteConstants.START_RUNNING, self.animation_direction)
        if self.animation_iteration == 4 or self.animation_iteration == 10:
            self.update_sprite(SpriteConstants.RUNNING_STEP1, self.animation_direction)
        if self.animation_iteration == 7:
            self.update_sprite(SpriteConstants.RUNNING_STEP2, self.animation_direction)

        self.animation_iteration += 1
        if self.animation_iteration > 12:
            self.animation_iteration = 1

    @abstractmethod
    def shoot(self):
        pass

    def shooting_animation(self):
        if self.shooting_cooldown > 0:
            if not self.resting:
                self.update_sprite(SpriteConstants.SHOOTING_JUMPING, self.animation_direction)
            else:
                self.update_sprite(SpriteConstants.SHOOTING, self.animation_direction)

    def animation_cooldowns(self, dt):
        self.hurt_cooldown = max(0, self.hurt_cooldown - dt)
        self.gun_cooldown = max(0, self.gun_cooldown - dt)
        self.shooting_cooldown = max(0, self.shooting_cooldown - dt)

    def damage(self, amount: float):
        self.life = max(0, self.life - amount)
        # Sets flag to change the player image when he is hurt.
        self.hurt_cooldown = self.hurt_delay

    def hurt_animation(self):
        if self.hurt_cooldown > 0:
            self.update_sprite(SpriteConstants.HURTING, self.animation_direction)

    def jumping_animation(self):
        if not self.resting:
            self.update_sprite(SpriteConstants.JUMPING, self.animation_direction)

    def standing_animation(self):
        self.update_sprite(SpriteConstants.STANDING, self.animation_direction)

    def fire_projectile(self, position=None, bullet_index: int = 0):
        if position is None:
            position = self.rect.midright
        projectile = self.projectile_type(position, self.direction, bullet_index, self.sprite_layer)

        self.projectiles[projectile.id] = projectile

    def set_direction(self, direction):
        self.direction = direction
        self.animation_direction = get_direction_sprite(self.direction)

    def update_projectiles(self, dt, game):
        killed_projectile = []

        for projectile in self.projectiles.values():
            killed = projectile.update(dt, game)
            if killed:
                killed_projectile.append(projectile.id)

        for id in killed_projectile:
            del self.projectiles[id]

    def shot(self, damage):
        if self.hurt_cooldown == 0.0:
            # enemy loses life points, according to the difficult level of the game (the more difficult, the less it loses)
            self.life = max(0, self.life - damage)
            self.hurt_cooldown = 1
