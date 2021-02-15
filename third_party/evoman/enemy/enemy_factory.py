from abstract.creational.factory import Factory
from third_party.evoman.enemy.enemy import Enemy
from third_party.evoman.projectile.enemy_bullet import EnemyBullet, EnemyProjectile


class EnemyFactory(Factory):

    def create(self, enemy_type, bullet_name) -> (type(EnemyProjectile), type(Enemy)):
        if bullet_name == "bullet":
            bullet_type = EnemyBullet

        if enemy_type == "enemy":
            enemy_type = Enemy

        return bullet_type, enemy_type
