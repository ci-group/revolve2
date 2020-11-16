from nca.core.genome.grammar.symbol import Symbol


class RobogenBrainSymbol(Symbol):

    # ControllerChangingCommands
    ADD_EDGE = 'brainedge'
    MUTATE_EDGE = 'brainperturb'
    LOOP = 'brainloop'
    MUTATE_AMP = 'brainampperturb'
    MUTATE_PER = 'brainperperturb'
    MUTATE_OFF = 'brainoffperturb'

    # ControllerMovingCommands
    MOVE_S = 'brainmoveFTS'
    MOVE_O = 'brainmoveTTS'
