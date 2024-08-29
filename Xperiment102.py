import os
import argparse
import shutil
import re

folders = [
    "airport"  # Add other domain folder names here if needed
    , "tpp",
    "my"
]

beams = [2]
max_depths = [10000]

def mdprdif(folder, ds, ps):
    # Create directories and move domain and problems into corresponding folders
    for d in ds:
        domain_name = d.replace('.pddl', '')
        domain_dir = f"misc/tests/benchmarks/{folder}/{domain_name}"
        os.makedirs(domain_dir, exist_ok=True)
        shutil.copy(f"misc/tests/benchmarks/{folder}/{d}", f"{domain_dir}/domain.pddl")

    for p in ps:
        problem_name = p.replace('.pddl', '')
        for d in ds:
            domain_name = d.replace('.pddl', '')
            problem_dir = f"misc/tests/benchmarks/{folder}/{domain_name}"
            shutil.copy(f"misc/tests/benchmarks/{folder}/{p}", f"{problem_dir}/{p}")

    # Original files remain in place; no need to remove them
def generate_fast_downward_commands(base_path, domain_folder, beam_widths, max_depths, task, ehc=False):
    """
    Generates Fast Downward commands with domain.pddl listed first, followed by the task files.

    :param base_path: The base path where the domain and task files are located.
    :param domain_folder: The specific folder containing the domain.pddl and task files.
    :param beam_widths: List of beam widths to iterate over.
    :param max_depths: List of max depths to iterate over.
    :return: List of commands.
    """
    commands = []
    domain_path = os.path.join(base_path, domain_folder, "domain.pddl")

    # List all task files (excluding domain.pddl)
    task_files = [f for f in os.listdir(os.path.join(base_path, domain_folder))
                  if f.endswith(".pddl") and f != "domain.pddl"]

    t_paths = [os.path.join(base_path, domain_folder, task) for task in task_files]
    # Generate commands for each combination of task, beam width, and max depth
    for beam in beam_widths:
        for max_depth in max_depths:
            search_command = f'--search "{"ehc" if ehc else "ehcbrrw"}(add(), cost_type=normal, bound=infinity, max_time=10'
            if not ehc:
                search_command += f', beam_width={beam}, max_depth={max_depth})"'
            else:
                search_command += ')"'
            # search_command = f'--search "ehc(add(), cost_type=normal, bound=infinity, max_time=infinity)"'
            command = f"./fast-downward.py {domain_path} {t_paths[task]} {search_command}"
            commands.append(command)

    return commands


# Construct the base command
base_command = "./fast-downward.py misc/tests/benchmarks/{}/{}/{}"

all_commands = []

for folder in folders:
    domains = [file for file in os.listdir(f"misc/tests/benchmarks/{folder}") if file.startswith("domain") and file.endswith(".pddl")]
    problems = [file for file in os.listdir(f"misc/tests/benchmarks/{folder}") if not file.startswith("domain") and file.endswith(".pddl")]

    if domains and not os.path.isdir(f"misc/tests/benchmarks/{folder}/{domains[0].replace('.pddl', '')}"):
        mdprdif(folder, domains, problems)

    for domain in domains:
        domain_name = domain.replace('.pddl', '')
        command = base_command.format(folder, domain_name, "domain.pddl")
        for problem in problems:
            problem_name = problem.replace('.pddl', '')
            full_command = command + f" misc/tests/benchmarks/{folder}/{domain_name}/{problem_name}.pddl"
            for beam in beams:
                for max_depth in max_depths:
                    search_command = f'--search "ehcbrrw(add(), cost_type=normal, bound=infinity, max_time=infinity, beam_width={beam}, max_depth={max_depth})"'
                    final_command = f"{full_command} {search_command}"
                    all_commands.append(final_command)

parser = argparse.ArgumentParser()
parser.add_argument("--build", action="store_true", help="Build the planner before running the experiments.")
parser.add_argument("--domain", type=int, help="The domain index to run the experiments on.")
parser.add_argument("--problem", type=int, help="The problem index to run the experiments on.")
parser.add_argument("--task", type=int, help="The problem index to run the experiments on.")
parser.add_argument("--lama", action="store_true", help="Run the experiments with LAMA instead of BRRW.")
parser.add_argument("--ehc", action="store_true", help="Run the experiments with EHC instead of BRRW.")
parser.add_argument("--all", action="store_true", help="Run the experiments for all domains and problems.")
parser.add_argument("--beam", type=int, help="The beam width to use for the experiments.")
parser.add_argument("--depth", type=int, help="The max depth to use for the experiments.")
parser.add_argument("--luby", action="store_true", help="Run the experiments with Luby sequence.")

args = parser.parse_args()

print(f"Total number of commands: {len(all_commands)}")

if args.build:
    print("Building the planner.")
    os.system("./build.py")


if args.domain is not None and args.problem is not None and args.task is not None and not args.all:
    if folders[args.domain] == "airport":
        two_digit_problem = f"{args.problem:02d}"
        command = generate_fast_downward_commands("misc/tests/benchmarks", folders[args.domain] + f"/domain{two_digit_problem}", beams, max_depths, args.task, args.ehc)[0]

        if args.lama:
            print(f"Running the experiments for domain {folders[args.domain]} with LAMA.")
            command = command.replace("./fast-downward.py", "./fast-downward.py --alias seq-sat-lama-2011").split("--search")[0]
            print(command)
        else:
            print(f"Running the experiments for domain {folders[args.domain]}.")
            print(command)

        # print(command[0])
        bash = """
        #!/bin/bash
        for i in {0..30}
        do
        """ + f""" 
            python3 Xperiment102.py --domain {args.domain} --problem {two_digit_problem} --task $i
        done
        """
        if not os.path.isdir(f"bash/"):
            os.makedirs(f"bash/", exist_ok=True)
            with open(f"bash/run1.sh", "w") as f:
                f.write(bash)
        # check in sta output for solution found
        # if found, write to file
        result = os.popen(command).read()
        print(result)

        # if "Solution found!" in result:
        #     with open(f"bash/run1.sh", "a+") as f:
        #         f.write(f"python3 Xperiment102.py --domain {args.domain} --problem {two_digit_problem} --task {args.task}\n")
        # else:
        #     print("No solution found")

        print(f"Task completed: {args.task}")
        print(f"Problem completed: {two_digit_problem}")
    else:
        two_digit_problem = f"{args.problem:02d}"

        if args.ehc and not args.luby:
            command = f"./fast-downward.py misc/tests/benchmarks/{folders[args.domain]}/domain.pddl misc/tests/benchmarks/{folders[args.domain]}/task{two_digit_problem}.pddl --search 'ehc(add(), cost_type=normal, bound=infinity, max_time=10)'"
        elif args.luby:
            command = f"./fast-downward.py misc/tests/benchmarks/{folders[args.domain]}/domain.pddl misc/tests/benchmarks/{folders[args.domain]}/task{two_digit_problem}.pddl --search 'ehcbrrw(add(), cost_type=normal, bound=infinity, max_time=60, beam_width={args.beam}, max_depth={args.depth}, restart_strategy=\"luby\")'"
        else:
            command = f"./fast-downward.py misc/tests/benchmarks/{folders[args.domain]}/domain.pddl misc/tests/benchmarks/{folders[args.domain]}/task{two_digit_problem}.pddl --search 'ehcbrrw(add(), cost_type=normal, bound=infinity, max_time=infinity, beam_width={args.beam}, max_depth={args.depth})'"
        os.system(command)

        print(f"Task completed: {args.task}")
        print(f"Problem completed: {two_digit_problem}")
# write a bash command to run the experimetns for all tasks in the domain

if args.all:
    print("Running the experiments for all domains and problems.")
    for domain in range(len(folders)):
            two_digit_problem = f"{args.problem:02d}"
            command = generate_fast_downward_commands("misc/tests/benchmarks", folders[domain] + f"/domain{two_digit_problem}", beams, max_depths, 0, args.ehc)

            if args.lama:
                print(f"Running the experiments for domain {folders[domain]} with LAMA.")
                command = command.replace("./fast-downward.py", "./fast-downward.py --alias seq-sat-lama-2011").split("--search")[0]
                print(command)
            else:
                print(f"Running the experiments for domain {folders[domain]}.")
                print(command)

            # check in sta output for solution found
            # if found, write to file
            for c in command:
                result = os.popen(c).read()
                print(result)
                print(c)

            # if "Solution found!" in result:
            #     with open(f"bash/run1.sh", "a+") as f:
            #         f.write(f"python3 Xperiment102.py --domain {domain} --problem {two_digit_problem} --task {task}\n")
            # else:
            #     print("No solution found")

            print(f"Task completed: {args.task}")
            print(f"Problem completed: {two_digit_problem}")