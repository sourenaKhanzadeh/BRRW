from __future__ import annotations
from XInstrumenter import XInstrumenter
from typing import List, Callable, Tuple
import os
import sys
import subprocess
import shutil
import tempfile
import multiprocessing


class Xperiment:
    """
    Experiment class for BRRW and EHC experiments.
    """


    def __init__(self: Xperiment) -> None:
        """
        Initialize the experiment.
        """
        self.instrumenter = XInstrumenter()

    def run(self: Xperiment, run: Callable[[str], List[str]], path: str) -> None:
        """
        Run the experiment at the given path.

        :param run: The function to run the experiment.
        :param path: The path to the experiment.
        """
        if not os.path.exists(path):
            print(f"Error: {path} does not exist.")
            sys.exit(1)

        names, output = run(path)

        os.makedirs("parser/OUTPUT", exist_ok=True)

        # parse the output and save the result in parser/OUTPUT
        for idx, parsed_output in enumerate(output):
            parsed_output = self.instrumenter.parse(parsed_output)
            self.instrumenter.to_file(parsed_output, "parser/OUTPUT/" + names[idx] + ".json")




class X:
    """
    Main class for running
    """

    def __init__(self: X) -> None:
        """
        Initialize the main class.
        """
        pass

    # to be called in Xperiment.run
    @staticmethod
    def run_experiment_set_1(path: str) -> Tuple[List[str], List[str]]:
        """
        Run the experiment at the given path.

        :param path: The path to the experiment.
        """
        result = []
        names = []
        # run the experiment
        with open(path, "r") as f:
            data = f.read()
            # the experiment is a text file with commands to run
            commands = data.split("\n")

            for command in commands:
                print(command)
                names.append(command)
                result.append(os.popen(command).read())


        return names, result


if __name__ == "__main__":
    __PATH__ = os.path.join(os.path.dirname(__file__), "experiments", "Xx1.txt")
    xperiment = Xperiment()
    xperiment.run(X.run_experiment_set_1, __PATH__)