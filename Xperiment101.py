import os
import argparse

folders = [
    "gripper",
    "miconic",
    "miconic-simpleadl",
    "philosophers",
    "satellite",
]

beams = [1]
max_depths = [10]

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
                search_command = f'--search "ehcbrrw(add(), cost_type=normal, bound=infinity, max_time=infinity, beam_width={beam}, max_depth={max_depth})"'
                full_command = f"{command} {search_command}"
                all_commands.append(full_command)

parser = argparse.ArgumentParser()
parser.add_argument("--build", action="store_true", help="Build the planner before running the experiments.")
parser.add_argument("--domain", type=int, help="The domain to run the experiments on.")
parser.add_argument("--lama", action="store_true", help="Run the experiments with LAMA instead of BRRW.")

# run command 0
if parser.parse_args().build:
    print("Building the planner.")
    os.system("./build.py")
    os.system(all_commands[parser.parse_args().domain])
elif parser.parse_args().lama:
    print(f"Running the experiments for domain {folders[parser.parse_args().domain]} with LAMA.")
    command = all_commands[parser.parse_args().domain]
    command = command.split("--search")
    command = command[0].replace("./fast-downward.py", "./fast-downward.py --alias seq-sat-lama-2011") 
    print(command)
    os.system(command)
else:
    print(f"Running the experiments for domain {folders[parser.parse_args().domain]}.")
    print(all_commands[parser.parse_args().domain])
    os.system(all_commands[parser.parse_args().domain])

