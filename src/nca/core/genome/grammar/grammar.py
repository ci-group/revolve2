import random



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
