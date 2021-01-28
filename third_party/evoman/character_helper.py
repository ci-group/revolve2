import pygame
from gym.envs import game

from third_party.evoman.base import SpriteConstants


def get_direction_sprite(direction: int):
    if direction == 1:
        return SpriteConstants.RIGHT

    return SpriteConstants.LEFT


def get_human_action():
    # if joystick is connected, initializes it.
    if game.joy > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

    # tests if the button/key was pressed or released.
    press = 0
    release = 0
    for event in game.event:
        press = event.type == pygame.JOYBUTTONDOWN or event.type == pygame.KEYDOWN
        release = event.type == pygame.JOYBUTTONUP or event.type == pygame.KEYUP

    # gets pressed key value
    key = pygame.key.get_pressed()

    # gets joystick value for axis x (left/right)
    left = 0
    if game.joy > 0:
        if round(joystick.get_axis(0)) == -1:
            left = 1
    if key[pygame.K_LEFT]:
        left = 1

    right = 0
    if game.joy > 0:
        if round(joystick.get_axis(0)) == 1:
            right = 1
    if key[pygame.K_RIGHT]:
        right = 1

    # gets joystick/key value for jumping
    jump = 0
    if game.joy > 0:
        if int(joystick.get_button(2)) == 1 and press == 1:
            jump = 1
    if key[pygame.K_SPACE] and press == 1:
        jump = 1

    # gets joystick/key value for shooting
    shoot = 0
    if game.joy > 0:
        if int(joystick.get_button(3)) == 1 and press == 1:
            shoot = 1
    if key[pygame.K_LSHIFT] and press == 1:
        shoot = 1

    return left, right, jump, shoot, release


def wall_collisions(environment, new_position, last_position):
    resting: bool = False
    for cell in environment.tilemap.layers['triggers'].collide(new_position, 'blockers'):

        blockers = cell['blockers']

        if 'l' in blockers and last_position.right <= cell.left and new_position.right > cell.left \
                and last_position.bottom > cell.top:
            new_position.right = cell.left

        if 'r' in blockers and last_position.left >= cell.right and new_position.left < cell.right \
                and last_position.bottom > cell.top:
            new_position.left = cell.right

        if 't' in blockers and last_position.bottom <= cell.top and new_position.bottom > cell.top:
            new_position.bottom = cell.top
            resting = True

        if 'b' in blockers and last_position.top >= cell.bottom and new_position.top < cell.bottom:
            new_position.top = cell.bottom

    return resting