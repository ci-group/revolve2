from ._cpg import Cpg
from typing import Tuple
import numpy.typing as npt
import numpy as np
import json
from pathlib import Path
import os


class Brain(Cpg):
    def __init__(self) -> None:
        weight_matrix, num_output_neurons, initial_state = self.load_settings(
            os.path.join(Path(__file__).parent, "settings.json")
        )
        super().__init__(
            initial_state=initial_state,
            num_output_neurons=num_output_neurons,
            weight_matrix=weight_matrix,
        )

    @staticmethod
    def load_settings(
        file: str,
    ) -> Tuple[npt.NDArray[np.float_], int, npt.NDArray[np.float_]]:
        with open(file, "r") as settings_file:
            settings = json.loads(settings_file.read())
            return (
                np.array(settings["weight_matrix"]),
                settings["num_output_neurons"],
                np.array(settings["initial_state"]),
            )
