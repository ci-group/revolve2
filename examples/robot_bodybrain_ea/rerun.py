"""Rerun a robot with given body and parameters."""

import logging
import pickle

from evaluator import Evaluator
from individual import Individual
from revolve2.ci_group.logging import setup_logging

# This is a pickled genotype we optimized.
# You can copy your own parameters from the optimization output log.
PICKLED_GENOTYPE = b'\x80\x04\x95\x18\x1c\x00\x00\x00\x00\x00\x00\x8c\nindividual\x94\x8c\nIndividual\x94\x93\x94)\x81\x94}\x94(\x8c\x08genotype\x94h\x05\x8c\x08Genotype\x94\x93\x94)\x81\x94}\x94(\x8c\x05brain\x94\x8cMrevolve2.experimentation.genotypes.cppnwin._multineat_genotype_pickle_wrapper\x94\x8c\x1eMultineatGenotypePickleWrapper\x94\x93\x94)\x81\x94X\xf4\x08\x00\x00{\n"value0":{\n"value0":0,\n"value1":[\n{\n"value0":{\n"value0":[]\n},\n"value1":1,\n"value2":1,\n"value3":0.0,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":1,\n"value10":0.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":2,\n"value2":1,\n"value3":0.0,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":1,\n"value10":0.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":3,\n"value2":1,\n"value3":0.0,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":1,\n"value10":0.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":4,\n"value2":1,\n"value3":0.0,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":1,\n"value10":0.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":5,\n"value2":1,\n"value3":0.0,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":1,\n"value10":0.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":6,\n"value2":1,\n"value3":0.0,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":1,\n"value10":0.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":7,\n"value2":2,\n"value3":0.0,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":1,\n"value10":0.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":8,\n"value2":4,\n"value3":3.025,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":9,\n"value10":1.0\n}\n],\n"value2":[\n{\n"value0":{\n"value0":[]\n},\n"value1":1,\n"value2":8,\n"value3":1,\n"value4":false,\n"value5":0.038737880811819367\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":2,\n"value2":8,\n"value3":2,\n"value4":false,\n"value5":-0.09907188448129026\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":3,\n"value2":8,\n"value3":3,\n"value4":false,\n"value5":-0.22909649995177287\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":4,\n"value2":8,\n"value3":4,\n"value4":false,\n"value5":-0.6387836898954495\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":5,\n"value2":8,\n"value3":5,\n"value4":false,\n"value5":0.2102014923432433\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":6,\n"value2":8,\n"value3":6,\n"value4":false,\n"value5":0.3529407170070935\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":7,\n"value2":8,\n"value3":7,\n"value4":false,\n"value5":-0.8963519233987225\n}\n],\n"value3":7,\n"value4":1,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0.0,\n"value9":false,\n"value10":16384,\n"value11":{\n"value0":[]\n},\n"value12":8,\n"value13":7\n}\n}\x94b\x8c\x04body\x94h\r)\x81\x94X2\x12\x00\x00{\n"value0":{\n"value0":0,\n"value1":[\n{\n"value0":{\n"value0":[]\n},\n"value1":1,\n"value2":1,\n"value3":0.0,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":1,\n"value10":0.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":2,\n"value2":1,\n"value3":0.0,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":1,\n"value10":0.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":3,\n"value2":1,\n"value3":0.0,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":1,\n"value10":0.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":4,\n"value2":1,\n"value3":0.0,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":1,\n"value10":0.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":5,\n"value2":2,\n"value3":0.0,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":1,\n"value10":0.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":6,\n"value2":4,\n"value3":3.025,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":2,\n"value10":1.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":7,\n"value2":4,\n"value3":3.025,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":2,\n"value10":1.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":8,\n"value2":4,\n"value3":3.025,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":2,\n"value10":1.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":9,\n"value2":4,\n"value3":3.025,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":2,\n"value10":1.0\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":10,\n"value2":4,\n"value3":3.025,\n"value4":0.0,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0,\n"value9":2,\n"value10":1.0\n}\n],\n"value2":[\n{\n"value0":{\n"value0":[]\n},\n"value1":1,\n"value2":6,\n"value3":1,\n"value4":false,\n"value5":-0.8382595918707214\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":2,\n"value2":6,\n"value3":2,\n"value4":false,\n"value5":0.103917826220254\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":3,\n"value2":6,\n"value3":3,\n"value4":false,\n"value5":-0.4827558546439893\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":4,\n"value2":6,\n"value3":4,\n"value4":false,\n"value5":-0.6022557354328545\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":5,\n"value2":6,\n"value3":5,\n"value4":false,\n"value5":1.1862340579829092\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":1,\n"value2":7,\n"value3":6,\n"value4":false,\n"value5":0.42011057179410796\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":2,\n"value2":7,\n"value3":7,\n"value4":false,\n"value5":0.18068634920248165\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":3,\n"value2":7,\n"value3":8,\n"value4":false,\n"value5":-0.5894769422771407\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":4,\n"value2":7,\n"value3":9,\n"value4":false,\n"value5":-0.17043299197166085\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":5,\n"value2":7,\n"value3":10,\n"value4":false,\n"value5":0.07580802594176672\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":1,\n"value2":8,\n"value3":11,\n"value4":false,\n"value5":-0.22013182407796634\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":2,\n"value2":8,\n"value3":12,\n"value4":false,\n"value5":-0.5911980258119383\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":3,\n"value2":8,\n"value3":13,\n"value4":false,\n"value5":-0.24193877154144195\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":4,\n"value2":8,\n"value3":14,\n"value4":false,\n"value5":0.43140249853859449\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":5,\n"value2":8,\n"value3":15,\n"value4":false,\n"value5":0.19220604833897957\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":1,\n"value2":9,\n"value3":16,\n"value4":false,\n"value5":0.4868388307145815\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":2,\n"value2":9,\n"value3":17,\n"value4":false,\n"value5":0.19013503562770343\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":3,\n"value2":9,\n"value3":18,\n"value4":false,\n"value5":0.12407342022224152\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":4,\n"value2":9,\n"value3":19,\n"value4":false,\n"value5":-0.10264634968532864\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":5,\n"value2":9,\n"value3":20,\n"value4":false,\n"value5":-0.5620415535186855\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":1,\n"value2":10,\n"value3":21,\n"value4":false,\n"value5":0.20237041070749937\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":2,\n"value2":10,\n"value3":22,\n"value4":false,\n"value5":0.07332207063104221\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":3,\n"value2":10,\n"value3":23,\n"value4":false,\n"value5":0.490101031619121\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":4,\n"value2":10,\n"value3":24,\n"value4":false,\n"value5":0.20966011324605716\n},\n{\n"value0":{\n"value0":[]\n},\n"value1":5,\n"value2":10,\n"value3":25,\n"value4":false,\n"value5":-0.165981716033895\n}\n],\n"value3":5,\n"value4":5,\n"value5":0.0,\n"value6":0.0,\n"value7":0,\n"value8":0.0,\n"value9":false,\n"value10":16384,\n"value11":{\n"value0":[]\n},\n"value12":10,\n"value13":25\n}\n}\x94bub\x8c\x07fitness\x94G?\xd27\xfe\x0f\x19v\xc5ub.'


def main() -> None:
    """Perform the rerun."""
    setup_logging()

    individual: Individual = pickle.loads(PICKLED_GENOTYPE)
    robot = individual.genotype.develop()

    logging.info(f"Fitness from pickle: {individual.fitness}")

    evaluator = Evaluator(
        headless=False,
        num_simulators=1,
    )
    fitness = evaluator.evaluate([robot])[0]
    logging.info(f"Rerun fitness: {fitness}")


if __name__ == "__main__":
    main()
