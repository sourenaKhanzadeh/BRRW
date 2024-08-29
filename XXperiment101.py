import multiprocessing
import time
import os
import subprocess
import logging
import argparse


def run_search(domain_file, problem_file, beam, max_depth, output_file):
    command = f"./fast-downward.py {domain_file} {problem_file} --search 'ehcbrrw(add(), cost_type=normal, bound=infinity, max_time=60, beam_width={beam}, max_depth={max_depth}, restart_strategy=\"luby\")'"

    with open(output_file, 'w') as f:
        start = time.time()
        process = subprocess.Popen(command, shell=True, stdout=f, stderr=subprocess.PIPE)
        _, err = process.communicate()
        end = time.time()

        logging.info(f"Time taken: {end - start} seconds")
        if err:
            logging.error(err.decode('utf-8'))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--problem", type=int, default=1)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)

    beams = [1, 2, 4, 8, 16, 32, 64, 128, 256, 512]
    max_depths = [1, 2, 4, 8, 16, 32, 64, 128, 256, 512]

    folder_to_x = os.path.join(os.path.dirname(__file__), "misc", "tests", "benchmarks", "tpp")
    domain_file = os.path.join(folder_to_x, "domain.pddl")
    problem_file = os.path.join(folder_to_x, f"task{args.problem:02d}.pddl")

    output_folder = os.path.join(os.path.dirname(__file__), "OUT")
    os.makedirs(output_folder, exist_ok=True)

    # with multiprocessing.Pool() as pool:
    #     tasks = []
    #     for beam in beams:
    #         for max_depth in max_depths:
    #             output_file = os.path.join(output_folder, f"task{args.problem:02d}_beam{beam}_depth{max_depth}.txt")
    #             tasks.append(pool.apply_async(run_search, (domain_file, problem_file, beam, max_depth, output_file)))

        # Wait for all tasks to complete
        # for task in tasks:
        #     task.get()

    for beam in beams:
        for max_depth in max_depths:
            output_file = os.path.join(output_folder, f"task{args.problem:02d}_beam{beam}_depth{max_depth}.txt")
            run_search(domain_file, problem_file, beam, max_depth, output_file)
    logging.info("All searches completed.")
