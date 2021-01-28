################################
# EvoMan FrameWork - V1.0 2016 #
# Author: Karine Miras         #
# karine.smiras@gmail.com      #
################################

import sys
import numpy

from third_party.evoman.base.SpriteDefinition import *

from third_party.evoman.enemy.enemy import Enemy
from third_party.evoman.path import sound_path, evoman_path
from third_party.evoman.projectile.enemy_bullet import EnemyBullet

tilemap = evoman_path + 'map1.tmx'  # scenario
time_expire = 1000  # game run limit


# enemy 1 sprite, flashman
class Enemy2(Enemy):

    def __init__(self, location, *groups):
        super().__init__(location, *groups)
        self.gun_cooldown2 = 0

    def update(self, dt, game):

        if game.attack_timer == 1:
            # puts enemy in random initial position
            if game.random_initialization:
                self.rect.x = numpy.random.choice([640, 500, 400, 300])

        # increments enemy timer
        if game.start == 1:
            self.attack_timer += 1

        # defines game mode for player action
        if game.enemymode == 'static': #  controlled by static movements
            attack1 = 1

            if self.attack_timer >= 200 and self.attack_timer < 260:
                attack2 = 1
            else:
                attack2 = 0

            if self.attack_timer == 220:
                attack3 = 1
            else:
                attack3 = 0

            attack4 = 1

        elif game.enemymode == 'ai': # enemy controlled by AI algorithm

            # calls the controller providing game sensors
            actions = game.EnemyController.control(self.sensors.get(game), game.econt)
            if len(actions) < 4:
                game.print_logs("ERROR: Enemy 1 controller must return 4 decision variables.")
                sys.exit(0)

            attack1 = actions[0]
            attack2 = actions[1]
            attack3 = actions[2]
            attack4 = actions[3]

            # applies attack rules
            if attack2 == 1 and not self.gun_cooldown:
                attack2 = 1
            else:
                attack2 = 0

            if attack3 == 1 and not self.gun_cooldown2:
                attack3 = 1
            else:
                attack3 = 0

        # if the enemy is not atacking with the feezing atack (prevents player from making any movements) and also the 'start game' marker is 1.
        if game.freeze_e == 0 and game.start == 1:

            last = self.rect.copy()# copies last position state of the enemy

            if attack1 == 1:
                # moves the enemy on the axis x
                self.rect.x += self.direction * 100 * dt

                # chases player, switching direction as he moves.
                if attack4 == 1:

                    if game.enemymode == 'static':
                        if game.player.rect.right < self.rect.left:
                            self.direction = -1
                        elif game.player.rect.left > self.rect.right:
                             self.direction = 1
                    else:
                        self.direction = self.direction * -1

            # animation, running enemy images alternation.
            if self.direction > 0:
                direction = SpriteConstants.RIGHT
            else:
                direction = SpriteConstants.LEFT

            if self.alternate == 1:
                self.update_sprite(SpriteConstants.START_RUNNING, direction)
            if self.alternate == 4 or self.alternate == 10:
                self.update_sprite(SpriteConstants.RUNNING_STEP1, direction)
            if self.alternate == 7:
                self.update_sprite(SpriteConstants.RUNNING_STEP2, direction)

            self.alternate += 1
            if self.alternate > 12:
                self.alternate = 1

            # checks collision of the player with the enemy
            if self.rect.colliderect(game.player.rect):

                # sprite loses life points, according to the difficult level of the game (the more difficult, the more it loses).

                # choses what sprite penalise according to config
                if game.contacthurt == "player":
                    game.player.life = max(0, game.player.life-(game.level*1))
                if game.contacthurt == "enemy":
                    game.enemy.life = max(0, game.enemy.life-(game.level*1))

                # counts duration of the collision to jump from time to time during the collision
                self.time_colis += 1
                if self.time_colis > 15:
                    self.time_colis = 0
                    self.dy = -600

                # sets flag to change the player image when he is hurt
                game.player.hurt = 5


            # gravity
            self.dy = min(400, self.dy + 100)
            self.rect.y += self.dy * dt

            # controls screen walls and platforms limits towards enemy
            new = self.rect
            self.resting = 0
            for cell in game.tilemap.layers['triggers'].collide(new, 'blockers'):

                blockers = cell['blockers']

                if 't' in blockers and last.bottom <= cell.top and new.bottom > cell.top:
                    self.resting = 1
                    new.bottom = cell.top
                    self.dy = 0

                if 'b' in blockers and last.top >= cell.bottom and new.top < cell.bottom:
                    new.top = cell.bottom

                if 'l' in blockers and last.right <= cell.left and new.right > cell.left  and last.bottom>cell.top:
                    new.right = cell.left
                    # Jumps when finds a wall in the middle plataforms.
                    if new.left<600:
                        self.dy = -600

                if 'r' in blockers and last.left >= cell.right and new.left < cell.right and last.bottom>cell.top:
                    new.left = cell.right
                    # Jumps when finds a wall in the middle plataforms.
                    if new.left>29:
                        self.dy = -600

            #  Changes the image when enemy jumps.
            if self.resting == 0:
               if self.direction == -1:
                   self.update_sprite(SpriteConstants.JUMPING, SpriteConstants.LEFT)
               else:
                   self.update_sprite(SpriteConstants.JUMPING, SpriteConstants.RIGHT)

            # Hurt enemy animation.
            if self.hurt > 0:
                if self.direction == -1:
                   self.update_sprite(SpriteConstants.HURTING, SpriteConstants.LEFT)
                else:
                   self.update_sprite(SpriteConstants.HURTING, SpriteConstants.RIGHT)

            self.hurt -=1

        # Enemy atack: freezes the player (preeveting him from making any movements or atacking) and also himself from moving. Freenzing endures according to the timer.
        if attack2 == 1:

            self.gun_cooldown = 6

            game.freeze_p = 1
            game.freeze_e = 1

        # Enemy shooting after freezing.
        if attack3 == 1:

            self.shooting = 5

            self.gun_cooldown2 = 6

            # Shoots 8 bullets placed in a fixed range with a little random variation in their position (x and y).
            for i in range (0,8):
                rand = numpy.array([30,20,10,15,9,25,18,5])
                rand2 = numpy.array([1,2,3,4,5,2,4,3])

                rand = rand[i]
                rand2 = rand2[i]

                # Start position of the bullets vary according to the position of the enemy.
                if self.direction > 0:
                    self.twists.append(EnemyBullet((self.rect.x+(i*rand),self.rect.y+10+(i*rand2)), 1, len(self.twists), game.sprite_enemy))
                else:
                    self.twists.append(EnemyBullet((self.rect.x-(i*rand)+46,self.rect.y+10+(i*rand2)), -1, len(self.twists), game.sprite_enemy))

        # Decreases time for bullets and freezing limitation.
        self.gun_cooldown = max(0, self.gun_cooldown - dt)
        self.gun_cooldown2 = max(0, self.gun_cooldown2 - dt)

        # Changes bullets images according to the enemy direction.
        if self.shooting > 0:
            if self.direction == -1:
                self.update_sprite(SpriteConstants.SHOOTING, SpriteConstants.LEFT)
            else:
                self.update_sprite(SpriteConstants.SHOOTING, SpriteConstants.RIGHT)

        self.shooting -= 1
        self.shooting = max(0,self.shooting)

        # Releases movement.
        if self.gun_cooldown <= 5:
            game.freeze_p = 0
            game.freeze_e = 0

        # Reinicializes enemy atacking timer.
        if self.attack_timer == 260:
            self.attack_timer = 0
