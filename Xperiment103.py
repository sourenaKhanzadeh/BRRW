import argparse
import os
import sys
from pathlib import Path


__FOLDERS__ = ["airport"]
BENCHMARK = Path(os.path.join(os.pardir, "benchmarks"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate commands for Fast Downward.")
    parser.add_argument("--beam", type=int, default=2, help="Beam width.")
    parser.add_argument("--depth", type=int, default=10000, help="Max depth.")
    parser.add_argument("--problem", type=int, help="Problem index.")
    parser.add_argument("--ehc", action="store_true", help="Use EHC instead of EHC-BRRW.")
    args = parser.parse_args()

    for folder in __FOLDERS__:
        if folder == "airport":
            path = BENCHMARK / folder
            # sort the files in the folder
            files = [file for file in path.iterdir()]
            # find the domain file
            domain = files.index([file for file in files if file.name.startswith(f"p{args.problem:02d}-domain")][0])
            domain = files[domain]
            # find the problem file
            problem = files.index([file for file in files if file.name.startswith(f"p{args.problem:02d}") and "domain" not in file.name][0])
            problem = files[problem]

            if not args.ehc:
                command = f"./fast-downward.py {domain} {problem} --search 'ehcbrrw(add(), cost_type=normal, bound=infinity, max_time=infinity, beam_width={args.beam}, max_depth={args.depth})'"
            elif args.ehc:
                command = f"./fast-downward.py {domain} {problem} --search 'ehc(add(), cost_type=normal, bound=infinity, max_time=infinity)'"
            os.system(command)
