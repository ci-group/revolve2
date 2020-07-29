from enum import auto

from pyrevolve.evolutionary.algorithm.genome.representations.l_system.alphabet import Alphabet


class RobogenAlphabet(Alphabet):

    MODULES = auto()
    MOUNTING = auto()
    MOVE = auto()
    BRAIN = auto()


class RobogenModuleAlphabet(RobogenAlphabet):
    MODULE_CORE_COMPONENT = 'C'
    MODULE_JOINT_HORIZONTAL = 'AJ1'
    MODULE_JOINT_VERTICAL = 'AJ2'
    MODULE_BLOCK = 'B'
    MODULE_SENSOR = 'ST'


class RobogenMountingAlphabet(RobogenAlphabet):
    MOUNTING_ADD_RIGHT = 'addr'
    MOUNTING_ADD_FRONT = 'addf'
    MOUNTING_ADD_LEFT = 'addl'


class RobogenMovingAlphabet(RobogenAlphabet):
    MOVE_RIGHT = 'mover'
    MOVE_FRONT = 'movef'
    MOVE_LEFT = 'movel'
    MOVE_BACK = 'moveb'


class RobogenBrainAlphabet(RobogenAlphabet):
    # ControllerChangingCommands
    BRAIN_ADD_EDGE = 'brainedge'
    BRAIN_MUTATE_EDGE = 'brainperturb'
    BRAIN_LOOP = 'brainloop'
    BRAIN_MUTATE_AMP = 'brainampperturb'
    BRAIN_MUTATE_PER = 'brainperperturb'
    BRAIN_MUTATE_OFF = 'brainoffperturb'

    # ControllerMovingCommands
    BRAIN_MOVE_S = 'brainmoveFTS'
    BRAIN_MOVE_O = 'brainmoveTTS'
