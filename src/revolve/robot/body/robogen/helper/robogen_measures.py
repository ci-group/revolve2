from nca.core.genome.representations.symbolic_representation import SymbolicRepresentation


def symbol_counting(encoding: SymbolicRepresentation):
    symbol_count = {}
    for symbol in encoding:
        if symbol in symbol_count.keys():
            symbol_count[symbol] += 1
        else:
            symbol_count[symbol] = 1

    return symbol_count
