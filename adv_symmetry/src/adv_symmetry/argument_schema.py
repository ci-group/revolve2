from typing import Dict
from dataclasses import dataclass

import marshmallow_dataclass


@dataclass
class Arguments:
    num_generations: int
    num_individuals: int


PARASER = marshmallow_dataclass.class_schema(Arguments)()


def parse_json(json: Dict) -> Arguments:
    return PARASER.load(json)
