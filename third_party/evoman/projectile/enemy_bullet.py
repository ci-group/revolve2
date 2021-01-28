# Enemy's bullets.
from abc import ABC

import pygame

from third_party.evoman.path import projectile_image_path
from third_party.evoman.projectile.projectile import Projectile


class EnemyProjectile(Projectile, ABC):

    def __init__(self, location, direction, bullet_index, bullet_type: str, bullet_speed: int, *groups):
        super(EnemyProjectile, self).__init__(location, direction, bullet_type, bullet_speed, *groups)
        self.bullet_index = bullet_index

    def target(self, game):
        # checks collision of player's bullet with the enemy
        if self.rect.colliderect(game.player.rect):
            # if enemy is not imune
            game.player.shot(game.life_penalty())

            # removes the bullet off the screen after collision.
            self.kill()
            return True
        return False


class EnemyBullet(EnemyProjectile):
    bullet_speed = 300

    def __init__(self, location, direction, bullet_index, *groups):
        super(EnemyBullet, self).__init__(location, direction, bullet_index, 'bullet_enemy', self.bullet_speed, *groups)


# enemy's bullet
class EnemyDynamicProjectile(EnemyProjectile):

    def update(self, dt, game):
        if game.attack_timer % 2 == 0:
            self.image = pygame.image.load(projectile_image_path + '/' + self.bullet_type + '_right.png')
        else:
            self.image = pygame.image.load(projectile_image_path + '/' + self.bullet_type + '_left.png')


class EnemyTornado(EnemyDynamicProjectile):
    def __init__(self, location, direction, bullet_index, *groups):
        super(EnemyTornado, self).__init__(location, direction, bullet_index, 'tornado', *groups)
        self.lifespan = 55

    def update(self, dt, game):
        super().update(dt, game)

        # removes bullets objetcs when they transpass the screen limits
        if self.rect.right < 1 or self.rect.left > 736 or self.rect.bottom < 1 or self.rect.top > 512:
            self.kill()
            projectile_index = game.enemy.projectiles.index()
            del game.enemy.projectiles[self.id]
            return

        # enemy atack: blows the player forward with the bullets
        if self.lifespan > 43:
            ax = [100, 380, 440, 270, 220, 300]
            ay = [30, 70, 120, -40, 80, 130]

            if self.direction == -1:
                if self.rect.x >= game.enemy.rect.x - ax[self.bullet_index]:
                    self.rect.x -= 1400 * dt
            if self.direction == 1:
                if self.rect.x <= game.enemy.rect.x + ax[self.bullet_index]:
                    self.rect.x += 1400	 * dt

            if self.rect.y >= game.enemy.rect.y - ay[self.bullet_index]:
                self.rect.y -= 550 * dt

        elif self.lifespan <= 5:
            self.rect.x += self.direction * 650 * dt
            game.player.rect.x += self.direction * 150 * dt

            # limitates player in the screen.
            if game.player.rect.x < 60:
                game.player.rect.x = 60
            if game.player.rect.x > 620:
                game.player.rect.x = 620

        # decreases bullet's timer
        self.lifespan -= 1

        # checks collision of enemy's bullet with the player
        if self.rect.colliderect(game.player.rect):

            # player loses life points, according to the difficult level of the game (the more difficult, the more it loses).
            game.player.life = max(0, game.player.life-(game.level*1))

            # sets flag to change the player image when he is hurt
            game.player.hurt = 5

        # removes player's bullets when colliding with enemy's bullets
        aux = 0
        for t in game.player.twists:
            if t != None:
                if self.rect.colliderect(t.rect):
                    t.kill()
                    game.player.twists[aux] = None
            aux += 1
