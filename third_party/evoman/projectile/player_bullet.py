# player projectile
from third_party.evoman.projectile.projectile import Projectile


class PlayerBullet(Projectile):

    bullet_speed = 600

    def __init__(self, location, direction, twist_index, *groups):
        super(PlayerBullet, self).__init__(location, direction, 'bullet_player', self.bullet_speed, *groups)

    def target(self, game):
        # checks collision of player's bullet with the enemy
        if self.rect.colliderect(game.enemy.rect):
            # if enemy is not imune
            game.enemy.shot(game.life_penalty())

            # removes the bullet off the screen after collision.
            self.kill()
            return True
        return False