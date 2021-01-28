import numpy as np


class SemiThueSystem:

    def __init__(self, ):
        pass

    def apply_rules(self, rules: dict, symbols):
        new_symbols = []

        for index, element in enumerate(symbols):
            if element in rules.keys():
                # ability for deterministic and probabilistic rule application
                new_symbols.extend(rules[element]) # random.choice()
            else:
                new_symbols.append(element)

        return new_symbols


RewritingGrammar = SemiThueSystem


class StochasticSemiThueSystem:

    def __init__(self, ):
        pass

    def apply_rules(self, rules: dict, symbols, iterations=10):
        new_symbols = []

        rule = np.random.choice(list(rules.keys()))
        for index, element in enumerate(symbols):
            if element in rule:
                # ability for deterministic and probabilistic rule application
                new_symbols.extend(rules[element])  # random.choice()
            else:
                new_symbols.append(element)

        if iterations > 0:
            return self.apply_rules(rules, new_symbols, iterations-1)
        else:
            return new_symbols


StochasticRewritingGrammar = StochasticSemiThueSystem
