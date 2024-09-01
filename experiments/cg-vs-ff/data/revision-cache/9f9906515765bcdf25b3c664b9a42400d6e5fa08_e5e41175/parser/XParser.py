from __future__ import annotations
from abc import ABC, abstractmethod
from typing import List, Dict

class Parser(ABC):
    """
    Abstract parser class for parsing experiment runs.
    """

    @abstractmethod
    def parse(self: Parser, path: str) -> Dict[str, str]:
        """
        Parse the experiment run at the given path.

        :param path: The path to the experiment run.
        :return: The parsed experiment run.
        """
        pass

    @abstractmethod
    def to_string(self: Parser, run: Dict[str, str]) -> str:
        """
        Convert the given experiment run to a string.

        :param run: The experiment run to convert.
        :return: The string representation of the experiment run.
        """
        pass

    @abstractmethod
    def from_string(self: Parser, string: str) -> Dict[str, str]:
        """
        Convert the given string to an experiment run.

        :param string: The string to convert.
        :return: The experiment run.
        """
        pass

    @abstractmethod
    def to_file(self: Parser, run: Dict[str, str], path: str) -> None:
        """
        Write the given experiment run to a file.

        :param run: The experiment run to write.
        :param path: The path to write the experiment run to.
        """
        pass

    @abstractmethod
    def from_file(self: Parser, path: str) -> Dict[str, str]:
        """
        Read an experiment run from the given file.

        :param path: The path to read the experiment run from.
        :return: The experiment run.
        """
        pass

    @abstractmethod
    def to_files(self: Parser, runs: List[Dict[str, str]], path: str) -> None:
        """
        Write the given experiment runs to files.

        :param runs: The experiment runs to write.
        :param path: The path to write the experiment runs to.
        """
        pass

    @abstractmethod
    def from_files(self: Parser, path: str) -> List[Dict[str, str]]:
        """
        Read experiment runs from the given files.

        :param path: The path to read the experiment runs from.
        :return: The experiment runs.
        """
        pass

