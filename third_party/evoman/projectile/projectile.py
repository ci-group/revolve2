from abc import ABC, abstractmethod

import pygame

from nca.core.abstract.sequential_identifier import SequentialIdentifier
from third_party.evoman.path import projectile_image_path


class Projectile(pygame.sprite.Sprite, ABC):

    identifier = SequentialIdentifier()

    def __init__(self, location, direction: int, bullet_type: str, bullet_speed: int, *groups):
        super(Projectile, self).__init__(*groups)

        self.id = self.identifier.id()

        self.direction = direction
        self.bullet_speed = bullet_speed

        # fits image according to the side the player is turned to
        self.bullet_type: str = bullet_type
        if self.direction == 1:
            self.path = projectile_image_path + '/%s_right.png' % bullet_type
        else:
            self.path = projectile_image_path + '/%s_left.png' % bullet_type
        self.image = pygame.image.load(self.path)

        self.rect = pygame.rect.Rect(location, self.image.get_size())

    def update(self, dt, game):
        out_of_frame = self.out_of_bounds(game)
        if out_of_frame:
            return True

        self.move(dt)

        killed = self.target(game)
        if killed:
            return True

        return False

    @abstractmethod
    def target(self, game):
        pass

    def out_of_bounds(self, game):
        boundaries = game.get_boundaries()
        # removes bullets objetcs when they transpass the screen limits
        if self.rect.right < boundaries[0] or self.rect.left > boundaries[1] or \
                self.rect.top < boundaries[2] or self.rect.bottom > boundaries[3]:
            self.kill()
            return True

        return False

    def move(self, dt):
        self.rect.x += self.direction * self.bullet_speed * dt
