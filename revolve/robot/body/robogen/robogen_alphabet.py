from nca.core.genome.grammar import Alphabet


class RobogenModuleAlphabet(Alphabet):

    MODULE_CORE_COMPONENT = 'C'
    MODULE_JOINT_HORIZONTAL = 'AJ1'
    MODULE_JOINT_VERTICAL = 'AJ2'
    MODULE_BLOCK = 'B'
    MODULE_SENSOR = 'ST'

    @classmethod
    def probabilities(self):
        return [0.0, 0.25, 0.25, 0.5, 0.0]


class RobogenMountingAlphabet(Alphabet):

    MOUNTING_ADD_RIGHT = 'addr'
    MOUNTING_ADD_FRONT = 'addf'
    MOUNTING_ADD_LEFT = 'addl'


class RobogenMovingAlphabet(Alphabet):

    MOVE_RIGHT = 'mover'
    MOVE_FRONT = 'movef'
    MOVE_LEFT = 'movel'
    MOVE_BACK = 'moveb'


class RobogenAlphabet(Alphabet):

    MODULES = RobogenModuleAlphabet
    MOUNTING = RobogenMountingAlphabet
    MOVING = RobogenMovingAlphabet
