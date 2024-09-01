from __future__ import annotations
from XParser import Parser
import os
import sys
import json
from typing import Dict, List
import re


class XHandler(Parser):
    """
    Handler class for parsing
    """

    def parse(self: XHandler, data: str) -> Dict[str, str]:
        """
        Parse the experiment run at the given path.

        :param data: data given to parse.
        :return: The parsed experiment run.
        """

        # search for Search Found in the file
        # search for [t=<float>s, <int> KB] Plan length: [int] step(s).
        # search for [t=<float>s, <int> KB] Plan cost: [int]
        # search for [t=<float>s, <int> KB] Expanded [int] state(s).
        # search for [t=<float>s, <int> KB] Reopened [int] state(s).
        # search for [t=<float>s, <int> KB] Evaluated [int] state(s).
        # search for [t=<float>s, <int> KB] Generated [int] state(s).
        # search for [t=<float>s, <int> KB] Dead ends: [int] state(s).
        # search for [t=<float>s, <int> KB] Total time: [float]


        # search for Search Found in the file
        search_found = re.search(r"Solution found", data)
        if search_found:
            search_found = search_found.group()
            # search for [t=<float>s, <int> KB] Plan length: [int] step(s).
            plan_length = re.search(r"\[t=\d+\.\d+s, \d+ KB\] Plan length: \d+ step\(s\)", data)
            if plan_length:
                plan_length = plan_length.group()

            # search for [t=<float>s, <int> KB] Plan cost: [int]
            plan_cost = re.search(r"\[t=\d+\.\d+s, \d+ KB\] Plan cost: \d+", data)
            if plan_cost:
                plan_cost = plan_cost.group()

            # search for [t=<float>s, <int> KB] Expanded [int] state(s).
            expanded_states = re.search(r"\[t=\d+\.\d+s, \d+ KB\] Expanded \d+ state\(s\)", data)
            if expanded_states:
                expanded_states = expanded_states.group()

            # search for [t=<float>s, <int> KB] Reopened [int] state(s).
            reopened_states = re.search(r"\[t=\d+\.\d+s, \d+ KB\] Reopened \d+ state\(s\)", data)
            if reopened_states:
                reopened_states = reopened_states.group()

            # search for [t=<float>s, <int> KB] Evaluated [int] state(s).
            evaluated_states = re.search(r"\[t=\d+\.\d+s, \d+ KB\] Evaluated \d+ state\(s\)", data)
            if evaluated_states:
                evaluated_states = evaluated_states.group()

            # search for [t=<float>s, <int> KB] Generated [int] state(s).
            generated_states = re.search(r"\[t=\d+\.\d+s, \d+ KB\] Generated \d+ state\(s\)", data)
            if generated_states:
                generated_states = generated_states.group()

            # search for [t=<float>s, <int> KB] Dead ends: [int] state(s).
            dead_ends = re.search(r"\[t=\d+\.\d+s, \d+ KB\] Dead ends: \d+ state\(s\)", data)
            if dead_ends:
                dead_ends = dead_ends.group()

            # search for [t=<float>s, <int> KB] Total time: [float]
            total_time = re.search(r"\[t=\d+\.\d+s, \d+ KB\] Total time: \d+\.\d+", data)
            if total_time:
                total_time = total_time.group()

            return {
                "search_found": search_found,
                "plan_length": plan_length,
                "plan_cost": plan_cost,
                "expanded_states": expanded_states,
                "reopened_states": reopened_states,
                "evaluated_states": evaluated_states,
                "generated_states": generated_states,
                "dead_ends": dead_ends,
                "total_time": total_time
            }
        else:
            search_found = "None"
            # search for [t=<float>s, <int> KB] Expanded [int] state(s).
            expanded_states = re.search(r"\[t=\d+\.\d+s, \d+ KB\] Expanded \d+ state\(s\)", data)
            if expanded_states:
                expanded_states = expanded_states.group()

            # search for [t=<float>s, <int> KB] Reopened [int] state(s).
            reopened_states = re.search(r"\[t=\d+\.\d+s, \d+ KB\] Reopened \d+ state\(s\)", data)
            if reopened_states:
                reopened_states = reopened_states.group()

            # search for [t=<float>s, <int> KB] Evaluated [int] state(s).
            evaluated_states = re.search(r"\[t=\d+\.\d+s, \d+ KB\] Evaluated \d+ state\(s\)", data)
            if evaluated_states:
                evaluated_states = evaluated_states.group()

            # search for [t=<float>s, <int> KB] Generated [int] state(s).
            generated_states = re.search(r"\[t=\d+\.\d+s, \d+ KB\] Generated \d+ state\(s\)", data)
            if generated_states:
                generated_states = generated_states.group()

            # search for [t=<float>s, <int> KB] Dead ends: [int] state(s).
            dead_ends = re.search(r"\[t=\d+\.\d+s, \d+ KB\] Dead ends: \d+ state\(s\)", data)
            if dead_ends:
                dead_ends = dead_ends.group()

            # search for [t=<float>s, <int> KB] Total time: [float]
            total_time = re.search(r"\[t=\d+\.\d+s, \d+ KB\] Total time: \d+\.\d+", data)
            if total_time:
                total_time = total_time.group()

            return {
                "search_found": search_found,
                "expanded_states": expanded_states,
                "reopened_states": reopened_states,
                "evaluated_states": evaluated_states,
                "generated_states": generated_states,
                "dead_ends": dead_ends,
                "total_time": total_time
            }

    def to_string(self: XHandler, run: Dict[str, str]) -> str:
        """
        Convert the given experiment run to a string.

        :param run: The experiment run to convert.
        :return: The string representation of the experiment run.
        """
        return json.dumps(run)

    def from_string(self: XHandler, string: str) -> Dict[str, str]:
        """
        Convert the given string to an experiment run.

        :param string: The string to convert.
        :return: The experiment run.
        """
        return json.loads(string)

    def to_file(self: XHandler, run: Dict[str, str], path: str) -> None:
        """
        Write the given experiment run to a file.

        :param run: The experiment run to write.
        :param path: The path to write the experiment run to.
        """
        with open(path, "w") as f:
            json.dump(run, f)

    def from_file(self: XHandler, path: str) -> Dict[str, str]:
        """
        Read an experiment run from the given file.

        :param path: The path to read the experiment run from.
        :return: The experiment run.
        """
        if not os.path.exists(path):
            print(f"Error: {path} does not exist.")
            sys.exit(1)

        with open(path, "r") as f:
            data = json.load(f)
            return data