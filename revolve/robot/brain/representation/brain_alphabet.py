from nca.core.genome.grammar import Alphabet


class RobogenBrainAlphabet(Alphabet):

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
