import os
import sys

folders = [
    "gripper",
    "miconic",
    "miconic-simpleadl",
    "philosophers",
    "satellite",
]

beams = [1]
max_depths = [3]

# Construct the base command
base_command = "./fast-downward.py misc/tests/benchmarks/{}/domain.pddl misc/tests/benchmarks/{}/{}"

all_commands = []

for folder in folders:
    # Get the file inside the folder that ends with .pddl and is not domain.pddl
    files = [file for file in os.listdir(f"misc/tests/benchmarks/{folder}") if file.endswith(".pddl") and file != "domain.pddl"]

    if files:
        command = base_command.format(folder, folder, files[0])

        for beam in beams:
            for max_depth in max_depths:
                search_command = f'--search "ehcbrrw(ipdb(), cost_type=normal, bound=infinity, max_time=infinity, beam_width={beam}, max_depth={max_depth})"'
                full_command = f"{command} {search_command}"
                all_commands.append(full_command)

__BUILD__ = True

# run command 0
if __BUILD__:
    os.system("./build.py")
    os.system(all_commands[1])
else:
    os.system(all_commands[1])

