################################
# EvoMan FrameWork - V1.0 2016 #
# Author: Karine Miras         #
# karine.smiras@gmail.com      #
################################

import sys
import numpy
import random

import third_party.evoman.base
from third_party.evoman.base.SpriteConstants import *
from third_party.evoman.base.SpriteDefinition import *
from third_party.evoman.sensors import Sensors

from third_party.evoman.path import evoman_path, image_path, sound_path

tilemap = evoman_path + '/map2.tmx'
timeexpire = 1000 # game run limit

# enemy 2 sprite, airman
class Enemy(pygame.sprite.Sprite):

    def __init__(self, location, *groups):

        super(Enemy, self).__init__(*groups)

        self.spriteDefinition = SpriteDefinition(image_path + '/EnemySprites.png', 0, 0, 43, 59)
        self.updateSprite(SpriteConstants.STANDING, SpriteConstants.LEFT)

        self.rect = pygame.rect.Rect(location, self.image.get_size())
        self.direction = -1
        self.max_life = 100
        self.life = self.max_life
        self.resting = 0
        self.dy = 0
        self.twists = []
        self.alternate = 1
        self.imune = 0
        self.timeenemy = 0
        self.hurt = 0
        self.shooting = 0
        self.gun_cooldown = 0

    def update(self, dt, game):

        if game.attack_timer==1:
            # puts enemy in random initial position
            if game.random_initialization == 'yes':
                self.rect.x = numpy.random.choice([630,610,560,530])

        # defines game mode for player action.
        if game.enemymode == 'static': # enemy controlled by static movements

            if (self.timeenemy >= 210  and self.timeenemy <= 250) or (self.timeenemy >= 260  and self.timeenemy <= 300):
                atack1 = 1
            else:
                atack1 = 0

            if self.timeenemy == 210 or self.timeenemy == 260:
                atack2 = 1
            else:
                atack2 = 0

            if self.timeenemy> 300:
                atack3 = 1
            else:
                atack3 = 0

            if (self.timeenemy == 40) or (self.timeenemy == 110) or (self.timeenemy == 180):
                atack4 = 1
            else:
                atack4 = 0

        elif game.enemymode == 'ai': # player controlled by AI algorithm

            # calls the controller providing game sensors
            actions = game.EnemyController.control(self.sensors.get(game), game.econt)
            if len(actions) < 4:
                game.print_logs("ERROR: Enemy 1 controller must return 4 decision variables.")
                sys.exit(0)

            atack1 = actions[0]
            atack2 = actions[1]
            atack3 = actions[2]
            atack4 = actions[3]


            if atack4 == 1 and not self.gun_cooldown:
                atack4 = 1
            else:
                atack4 = 0

            if atack1 == 1 and self.resting == 1:
                atack1 = 1
            else:
                atack1 = 0

        # if the 'start game' marker is 1
        if game.start == 1:

            # increments enemy timer
            self.timeenemy += 1

            # copies last position state of the enemy
            last = self.rect.copy()

            # movements of the enemy on the axis x. Happens 2 to each side.
            if atack1 == 1  :
                self.rect.x += self.direction * 200 * dt

                # jumps
                if atack2 == 1:
                    self.dy = -900
                    self.resting = 0

               # animation, running enemy images alternatetion.
                if self.direction > 0:
                    direction = SpriteConstants.RIGHT
                else:
                    direction = SpriteConstants.LEFT

                if self.alternate == 1:
                    self.updateSprite(SpriteConstants.START_RUNNING, direction)
                if self.alternate == 4 or self.alternate == 10:
                    self.updateSprite(SpriteConstants.RUNNING_STEP1, direction)
                if self.alternate == 7:
                    self.updateSprite(SpriteConstants.RUNNING_STEP2, direction)

                self.alternate += 1
                if self.alternate > 12:
                    self.alternate = 1

                #  changes the image when enemy jumps
                if self.resting == 0:
                   if self.direction == -1:
                       self.updateSprite(SpriteConstants.JUMPING, SpriteConstants.LEFT)
                   else:
                       self.updateSprite(SpriteConstants.JUMPING, SpriteConstants.RIGHT)

            else:
                # animation, standing up images
                if self.direction == -1:
                    self.updateSprite(SpriteConstants.STANDING, SpriteConstants.LEFT)
                else:
                    self.updateSprite(SpriteConstants.STANDING, SpriteConstants.RIGHT)

            # restart enemy timer and turns the enemy around
            if atack3 == 1:
                self.timeenemy = 0
                self.direction = self.direction * -1

            # checks collision of the player with the enemy
            if self.rect.colliderect(game.player.rect):

                # choses what sprite penalise according to config
                if game.contacthurt == "player":
                    game.player.life = max(0, game.player.life-(game.level*1))
                if game.contacthurt == "enemy":
                    game.enemy.life = max(0, game.enemy.life-(game.level*1))

                game.player.hurt = 5 # sets flag to change the player image when he is hurt.

            # gravity
            self.dy = min(400, self.dy + 100)
            self.rect.y += self.dy * dt

            # controls screen walls and platforms limits agaist enemy.
            new = self.rect
            self.resting = 0
            for cell in game.tilemap.layers['triggers'].collide(new, 'blockers'):

                blockers = cell['blockers']

                if 'l' in blockers and last.right <= cell.left and new.right > cell.left:
                    new.right = cell.left

                if 'r' in blockers and last.left >= cell.right and new.left < cell.right:
                    new.left = cell.right

                if 't' in blockers and last.bottom <= cell.top and new.bottom > cell.top:
                    self.resting = 1
                    new.bottom = cell.top
                    self.dy = 0

                if 'b' in blockers and last.top >= cell.bottom and new.top < cell.bottom:
                    new.top = cell.bottom

            # enemy shoots
            if atack4 == 1:

                self.shooting = 5

                self.gun_cooldown = 3

                # shoots 6 bullets placed in a fixed range
                for i in range (0,6):
                    self.twists.append(Bullet_e2((self.rect.x+10,self.rect.bottom), self.direction, i, len(self.twists), game.sprite_enemy))

            # decreases time for bullets limitation
            self.gun_cooldown = max(0, self.gun_cooldown - dt)

            # hurt enemy animation
            if self.hurt > 0:
                if self.direction == -1:
                   self.updateSprite(SpriteConstants.HURTING, SpriteConstants.LEFT)
                else:
                   self.updateSprite(SpriteConstants.HURTING, SpriteConstants.RIGHT)

            self.hurt -=1

            # changes bullets images according to the enemy direction
            if self.shooting > 0:
                if self.direction == -1:
                    self.updateSprite(SpriteConstants.SHOOTING, SpriteConstants.LEFT)
                else:
                    self.updateSprite(SpriteConstants.SHOOTING, SpriteConstants.RIGHT)

            self.shooting -= 1
            self.shooting = max(0,self.shooting)

    def shot(self):
        if game.enemy.immune == 0:
            # enemy loses life points, according to the difficult level of the game (the more difficult, the less it loses)
            game.enemy.life = max(0, game.enemy.life - (self.max_difficulty / game.level))

            if game.enemyn == 4:
                # makes enemy imune to player's shooting.
                game.enemy.immune = 1

    def updateSprite(self, state, direction):
        self.image = self.spriteDefinition.getImage(state, direction)
