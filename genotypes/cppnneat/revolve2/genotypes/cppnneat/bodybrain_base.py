from dataclasses import dataclass

import multineat


@dataclass
class BodybrainBase:
    _genotype: multineat.Genome
