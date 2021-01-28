

# enemy's bullet
import pygame

from third_party.evoman.path import projectile_image_path


class Bullet_e3(pygame.sprite.Sprite):

    def __init__(self, location, direction, btype, n_twist, *groups):
        super(Bullet_e3, self).__init__(*groups)
        self.rect = pygame.rect.Rect(location, self.image.get_size())
        self.direction = direction
        self.lifespan = 100
        self.btype = btype
        self.swingtime = 0
        self.n_twist = n_twist
        self.image = pygame.image.load(projectile_image_path + '/metal_right.png')

    def update(self, dt, game):
        if game.attack_timer % 2 == 0:
            self.image = pygame.image.load(projectile_image_path + '/metal_right.png')
        else:
            self.image = pygame.image.load(projectile_image_path + '/metal_left.png')

        # decreases bullet's timer
        self.lifespan -= 1

        # removes bullets objetcs when they transpass the screen limits
        if self.rect.right < 1 or self.rect.left>736 or self.rect.bottom < 1  or self.rect.top > 512:
            self.kill()
            game.enemy.twists[self.n_twist] = None
            return

        # moves the bullets
        if self.btype == 'h':  # bullets that come from the enemy
            if self.lifespan <= 50:
                self.rect.x += self.direction * 550 * dt
        else:
            if self.lifespan <= 60: # bullets that come from the top
                self.rect.y += 300 * dt

                # animation of the bullets swinging
                self.swingtime += 1

                if self.swingtime == 10:
                    self.rect.x += self.direction * 1000 * dt
                    self.direction = self.direction * -1
                    self.swingtime = 0

        # checks collision of enemy's bullet with the player
        if self.rect.colliderect(game.player.rect):

            # player loses life points, accoring to the difficult level of the game (the more difficult, the more it loses).
            game.player.life = max(0, game.player.life-(game.level*1))

            # pushes player when he collides with the enemy
            game.player.rect.x +=  self.direction *  100 * dt

            # limits the player to stand on the screen space even being pushed.
            if game.player.rect.x < 60:
                game.player.rect.x = 60
            if game.player.rect.x > 620:
                game.player.rect.x = 620

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


# enemy bullets
class Bullet_e4(pygame.sprite.Sprite):

    def __init__(self, location, direction, n, n_twist, *groups):
        super(Bullet_e4, self).__init__(*groups)
        self.rect = pygame.rect.Rect(location, self.image.get_size())
        self.direction = direction
        self.lifespan = 30
        self.n= n
        self.n_twist = n_twist
        self.image = pygame.image.load(projectile_image_path + '/bullet_left.png')

    def update(self, dt, game):
        # puts the bullets in positions relative to the player. They go from the enemy to where the player is.
        if self.n == 0:
            aux_x = 50
            aux_y = (abs(game.player.rect.x - game.enemy.rect.x)*0.55)
        elif self.n == 1:
            aux_x = 20
            aux_y = (abs(game.player.rect.x - game.enemy.rect.x)*0.60)
        elif self.n == 2:
            aux_x = -10
            aux_y = (abs(game.player.rect.x - game.enemy.rect.x)*0.65)

        # bullets axis x movement
        if self.direction == -1:
            if self.rect.x > game.player.rect.left + aux_x:
                self.rect.x += self.direction *  650  * dt
        else:
            if self.rect.x < game.player.rect.right - aux_x:
                self.rect.x += self.direction *  650  * dt

        # bullets axis y movements
        if self.direction == -1:
             if self.rect.x > game.player.rect.left + aux_y:
                 self.rect.y -=  500 * dt
             else:
                 self.rect.y +=  700 * dt
        else:
             if self.rect.x < game.player.rect.right - aux_y-10:
                 self.rect.y -=  500 * dt
             else:
                 self.rect.y +=  700 * dt

        # prevents bullets from passing through the floor
        self.rect.y = min(410,self.rect.y)

        # removes old bullets
        if self.rect.y == 410:
            self.lifespan -= 1

        if self.lifespan < 0:
            self.kill()
            game.enemy.twists[self.n_twist] = None
            return

        if self.rect.right<1 or self.rect.left>736 or  self.rect.top <1 or self.rect.bottom>512 :
            self.kill()
            game.enemy.twists[self.n_twist] = None
            return

        # checks collision of enemy's bullet with the player
        if self.rect.colliderect(game.player.rect):

            # player loses life points, accoring to the difficulty level of the game (the more difficult, the more it loses).
            game.player.life = max(0, game.player.life-(game.level*0.3))

            # pushes player when he collides with the enemy
            game.player.rect.x +=  self.direction *  100 * dt

           # limits the player to stand on the screen space even being pushed
            if game.player.rect.x < 60:
                game.player.rect.x = 60
            if game.player.rect.x > 620:
                game.player.rect.x = 620

            # sets flag to change the player image when he is hurt
            game.player.hurt = 5


# enemy bullets
class Bullet_e5(pygame.sprite.Sprite):



    def __init__(self, location, direction, pos_p, n_twist, *groups):
        super(Bullet_e5, self).__init__(*groups)
        self.rect = pygame.rect.Rect(location, self.image.get_size())
        self.direction = direction
        self.pos_p = pos_p
        self.n_twist = n_twist
        self.image = pygame.image.load(projectile_image_path + '/blade.png')

    def update(self, dt, game):

        # bullets go the player's  direction marked at the shooting time
        self.rect.x += self.direction *  550 * dt
        if self.rect.bottom < self.pos_p.bottom:
            self.rect.y +=  300  * dt

        # removes bullets objetcs when they transpass the screen limits
        if self.rect.right < 1 or self.rect.left>736 or self.rect.bottom < 1  or self.rect.top > 512:
            self.kill()
            game.enemy.twists[self.n_twist] = None
            return

        # checks collision of enemy's bullet with the player
        if self.rect.colliderect(game.player.rect):

            # player loses life points, according to the difficulty level of the game (the more difficult, the more it loses).
            game.player.life = max(0, game.player.life-(game.level*0.3))

            game.player.rect.x +=  self.direction *  100 * dt # pushes player when he collides with the enemy

           # limits the player to stand on the screen space even being pushed
            if game.player.rect.x < 60:
                game.player.rect.x = 60
            if game.player.rect.x > 620:
                game.player.rect.x = 620

            game.player.hurt = 5 # sets flag to change the player image when he is hurt



# enemy's bullet
class Bullet_e6(pygame.sprite.Sprite):

    def __init__(self, location, direction, n_twist, *groups):
        super(Bullet_e6, self).__init__(*groups)
        self.rect = pygame.rect.Rect(location, self.image.get_size())
        self.direction = direction
        self.lifespan = 70
        self.n_twist = n_twist
        self.image = pygame.image.load(projectile_image_path + '/mi2.png')

    def update(self, dt, game):

        self.rect.y += 500 * dt  # moves the bullets

        self.rect.y = min(410, self.rect.y) # prevents bullets from passing throught the floor

        self.lifespan -= 1 #  decreases bullet's timer

        # removes old bullets
        if self.lifespan < 0:
            self.kill()
            game.enemy.twists[self.n_twist] = None
            return

        # checks collision of enemy's bullet with the player
        if self.rect.colliderect(game.player.rect):

            # player loses life points, according to the difficulty level of the game (the more difficult, the more it loses).
            game.player.life = max(0, game.player.life-(game.level*0.3))

            game.player.rect.x +=  self.direction *  100 * dt # pushes player when he collides with the enemy

            # limits the player to stand on the screen space even being pushed
            if game.player.rect.x < 60:
                game.player.rect.x = 60
            if game.player.rect.x > 620:
                game.player.rect.x = 620


            game.player.hurt = 5 # sets flag to change the player image when he is hurt


# enemy's bullet
class Bullet_e7(pygame.sprite.Sprite):

    image = pygame.image.load(projectile_image_path + '/bullet_enemy_left.png')

    def __init__(self, location, direction, n_twist, *groups):
        super(Bullet_e7, self).__init__(*groups)
        self.rect = pygame.rect.Rect(location, self.image.get_size())
        self.direction = direction
        self.n_twist = n_twist

    def update(self, dt, game):

        self.rect.x +=  self.direction * 500 * dt  # moves the bullets on the axis x

        # removes bullets objetcs when they transpass the screen limits
        if self.rect.right < 1 or self.rect.left>736 or self.rect.bottom < 1  or self.rect.top > 512:
            self.kill()
            game.enemy.twists[self.n_twist] = None
            return

        # checks collision of enemy's bullet with the player
        if self.rect.colliderect(game.player.rect):

            # player loses life points, accoring to the difficult level of the game (the more difficult, the more it loses).
            game.player.life = max(0, game.player.life-(game.level*0.3))

            game.player.rect.x +=  self.direction *  100 * dt  # pushes player when he collides with the enemy

            # limits the player to stand on the screen space even being pushed
            if game.player.rect.x < 60:
                game.player.rect.x = 60
            if game.player.rect.x > 620:
                game.player.rect.x = 620

            # sets flag to change the player image when he is hurt
            game.player.hurt = 1
        else:
            game.player.hurt = 0


# enemy's bullet 2 (bubble)
class Bullet_e72(pygame.sprite.Sprite):

    image = pygame.image.load(projectile_image_path + '/bubble.png')

    def __init__(self, location, direction, n_twist, *groups):
        super(Bullet_e72, self).__init__(*groups)
        self.rect = pygame.rect.Rect(location, self.image.get_size())
        self.direction = direction
        self.direc = 1
        self.n_twist = n_twist

    def update(self, dt, game):

        self.rect.x +=  self.direction * 200 * dt * 0.5      # moves the bullets on the axis x

        # moves the bullets on the axis y. Go up and down according to the floor and imaginary top.
        self.rect.y += 200 * self.direc * dt * 0.4
        if self.rect.y >= 460 or self.rect.y <= 350:
            self.direc = self.direc * -1

        # removes bullets objetcs when they transpass the screen limits
        if self.rect.right < 1 or self.rect.left>736 or self.rect.bottom < 1  or self.rect.top > 512:
            self.kill()
            game.enemy.twists[self.n_twist] = None
            game.enemy.projectiles -=1
            return

        # checks collision of enemy's bullet with the player
        if self.rect.colliderect(game.player.rect):

            # player loses life points, according to the difficulty level of the game (the more difficult, the more it loses).
            game.player.life = max(0, game.player.life-(game.level*0.3))

            game.player.rect.x += self.direction * 100 * dt  # pushes player when he collides with the enemy

            # limits the player to stand on the screen space even being pushed
            if game.player.rect.x < 60:
                game.player.rect.x = 60
            if game.player.rect.x > 620:
                game.player.rect.x = 620

            game.player.hurt = 5  # sets flag to change the player image when he is hurt


# enemy's bullet
class Bullet_e8(pygame.sprite.Sprite):

    image = pygame.image.load(projectile_image_path + '/bullet_enemy_left.png')

    def __init__(self, location, direction, n, n_twist, *groups):
        super(Bullet_e8, self).__init__(*groups)
        self.rect = pygame.rect.Rect(location, self.image.get_size())
        self.direction = direction
        self.lifespan = 70
        self.n = n
        self.n_twist = n_twist

    def update(self, dt, game):

        self.lifespan -= 1  # decreases bullet's timer

        # moves the bullets up after sometime
        if self.lifespan < 40:
            self.rect.y -= 700 * dt
        else:
            self.rect.y += 500 * dt # moves the bullets down when it is shoot
            self.rect.y = min(410, self.rect.y) # preevens bullets from going away

        # moves the bullet on the axis x according to the player's direction
        if not (abs(self.rect.left-game.player.rect.left) <= 10 or abs(self.rect.right-game.player.rect.right) <= 10):
            if game.player.rect.left < self.rect.left:
               self.rect.x -= (400) * dt
            else:
               self.rect.x += (400) * dt

        # removes bullets objetcs when they transpass the screen limits
        if self.rect.right < 1 or self.rect.left>736 or self.rect.bottom < 1  or self.rect.top > 512:
            self.kill()
            game.enemy.twists[self.n_twist] = None
            return

        # checks collision of enemy's bullet with the player
        if self.rect.colliderect(game.player.rect):
            # player loses life points, according to the difficult level of the game (the more difficult, the more it loses).
            game.player.life = max(0, game.player.life - (game.level * 0.3))

            game.player.hurt = 5  # sets flag to change the player image when he is hurt
