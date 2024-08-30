from __future__ import annotations
from XHandler import XHandler
from typing import Dict, List
import csv
import tempfile


class XInstrumenter(XHandler):
    """
    Instrumenter class for parsing
    """

    def __init__(self: XInstrumenter) -> None:
        """
        Initialize the instrumenter.
        """
        pass

    def instrument(self: XInstrumenter, run: Dict[str, str]) -> Dict[str, str]:
        """
        Instrument the given experiment run.

        :param run: The experiment run to instrument.
        :return: The instrumented experiment run.
        """
        return run

    def deinstrument(self: XInstrumenter, run: Dict[str, str]) -> Dict[str, str]:
        """
        Deinstrument the given experiment run.

        :param run: The experiment run to deinstrument.
        :return: The deinstrumented experiment run.
        """
        return run

    def to_string(self: XInstrumenter, run: Dict[str, str]) -> str:
        """
        Convert the given experiment run to a string.

        :param run: The experiment run to convert.
        :return: The string representation of the experiment run.
        """
        return super().to_string(run)


    def to_csv(self: XInstrumenter, run: Dict[str, str]) -> str:
        """
        Convert the given experiment run to a CSV string.

        :param run: The experiment run to convert.
        :return: The CSV representation of the experiment run.
        """
        # save the dictionary to a csv file
        with open('experiment.csv', 'w') as f:
            for key in run.keys():
                f.write("%s,%s\n"%(key,run[key]))

        return "experiment.csv"

    def read_csv(self: XInstrumenter, path: str) -> Dict[str, str]:
        """
        Read an experiment run from the given CSV file.

        :param path: The path to read the experiment run from.
        :return: The experiment run.
        """
        # read the csv file and return the dictionary
        with open(path, mode='r') as infile:
            reader = csv.reader(infile)
            mydict = {rows[0]:rows[1] for rows in reader}

        return mydict

    def to_file(self: XInstrumenter, run: Dict[str, str], path: str) -> None:
        """
        Write the given experiment run to a file.

        :param run: The experiment run to write.
        :param path: The path to write the experiment run to.
        """
        super().to_file(run, path)


    def from_file(self: XInstrumenter, path: str) -> Dict[str, str]:
        """
        Read an experiment run from the given file.

        :param path: The path to read the experiment run from.
        :return: The experiment run.
        """
        return super().from_file(path)

    def to_files(self: XInstrumenter, runs: List[Dict[str, str]], path: str) -> None:
        """
        Write the given experiment runs to files.

        :param runs: The experiment runs to write.
        :param path: The path to write the experiment runs to.
        """
        super().to_files(runs, path)

    def from_files(self: XInstrumenter, path: str) -> List[Dict[str, str]]:
        """
        Read experiment runs from the given files.
        :param path:
        :return:
        """
        return super().from_files(path)

