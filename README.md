<img src="misc/images/fast-downward.svg" width="800" alt="Fast Downward">

Fast Downward is a domain-independent classical planning system. This repo adds the 
algorithm for Beam Random Restart Walk (BRRW) to the Fast Downward planner. 
located in the [enforced_hill_climbing_beam_random_restarting_walk_search.cc](src/search/search_algorithms/enforced_hill_climbing_beam_random_restarting_walk_search.cc)

Copyright 2003-2023 Fast Downward contributors (see below).

For further information:
- Fast Downward website: <https://www.fast-downward.org>
- Report a bug or file an issue: <https://issues.fast-downward.org>
- Fast Downward mailing list: <https://groups.google.com/forum/#!forum/fast-downward>
- Fast Downward main repository: <https://github.com/aibasel/downward>


Author: Sourena Khanzadeh

# Project Overview

## Table of Contents
- [Introduction](#introduction)
- [Directory Structure](#directory-structure)
  - [Main Project](#main-project)
  - [Source Code](#source-code)
  - [Other Directories](#other-directories)
- [Build Instructions](#build-instructions)
- [Running Experiments](#running-experiments)
- [License](#license)

## Introduction
This project is designed to implement and evaluate various search algorithms, including beam search and random restarting walk (RRW), for solving planning problems. The project includes both a C++ codebase for core algorithms and Python scripts for experiments and testing. Testing is done using the Fast Downward Lab which is the wrapper for the Fast Downward planner.

## Directory Structure

### Main Project
The main directory of the project contains high-level scripts and documentation for building and running the project.

```
.
├── BUILD.md
├── CHANGES.md
├── LICENSE.md
├── README.md
├── XXperiment101.py
├── Xperiment101.py
├── Xperiment102.py
├── Xperiment103.py
├── build.py
├── build_configs.py
├── driver
├── experiments
├── fast-downward.py
├── misc
├── parser
├── res101.txt
└── src
```

- **README.md**: This file, which provides an overview of the project.
- **LICENSE.md**: License information for the project.
- **Xperiment101.py**: Various Python scripts for running experiments and testing algorithms. (Not important)
- **Xperiment102.py**: Can selectively run the experiments. mostly for tpp benchmarks inside the misc folder.(Usefull)
- **Xperiment103.py**: runs the airport domain benchmarks. was mostly used to test `beam__width=2` (No longer important)
- **XXperiment101.py**: Inefficiet way to run the experiments. mostly for testing early stages. (Not important)
- **build.py**: Script for building the project.
- **misc/**: Contains images and other miscellaneous files, including some useful early stage benchmark.
- **fast-downward.py**: Entry point for running the Fast Downward planner.
- **parser/**: Contains utilities for parsing and handling experiment data.
- **src/**: Contains the source code for the core algorithms and tools. (Algorithm implementations)
- **experiments/**: Contains experiment data and results. This is the main folder where downward lab will write the results. (Very Important)

### Source Code
The `src/` directory contains the C++ source code for the search algorithms, as well as scripts for translation and planning.

```
src
├── CMakeLists.txt
├── cmake-build-debug
├── search
└── translate
```

- **search/**: Core search algorithms, including beam search and random restart walk.
- **translate/**: Scripts and utilities for translating planning tasks into a format that the planner can use.
- **cmake-build-debug/**: Build artifacts generated by CMake.

### Other Directories
- **VAL**: Directory containing various validation tools.
- **benchmarks**: Contains benchmark tasks for testing and comparing algorithms.
- **BRRW**: Contains implementations and configurations for Beam Random Restart Walk. (This is the main directory for the project)

## Build Instructions
To build the project, follow these steps:

   ```sh
   ./build.py
   ./fast-downward.py <domain.pddl> <problem.pddl> --search "ehcbrrw(...)"
   ```

## Running Experiments
To run selected experiments for testing on tpp domain benchmarks, use the following command:

```sh
python3 Xperiment102.py --domain <int> --problem <int> --beam <int> --depth <int> [--luby --l_seq <int>] [--ehc] [--build]
```

These scripts will execute the search algorithms on the tpp benchmarks and display the results.

### Setup for Running Experiments via Fast Downward Lab
To run experiments using the Fast Downward Lab, follow these steps:

- please start by making the current directory the parent directory of this project.
```sh
# Install required packages.
sudo apt install bison cmake flex g++ git make python3 python3-venv


# Install the plan validator VAL.
git clone https://github.com/KCL-Planning/VAL.git
cd VAL
# Newer VAL versions need time stamps, so we use an old version
# (https://github.com/KCL-Planning/VAL/issues/46).
git checkout a556539
make clean  # Remove old binaries.
sed -i 's/-Werror //g' Makefile  # Ignore warnings.
make

mkdir ~/bin/
cp validate ~/bin/  # Add binary to a directory on your ``$PATH``.

cd ../

# Download planning tasks.
git clone https://github.com/aibasel/downward-benchmarks.git benchmarks


cd BRRW # go to the project directory
cd experiments
cd cg-vs-ff

# If PYTHONPATH is set, unset it to obtain a clean environment.
unset PYTHONPATH

# Create and activate a Python virtual environment for Lab.
python3 -m venv --prompt my-paper .venv
source .venv/bin/activate

# Install Lab in the virtual environment.
pip install -U pip wheel

pip install -r requirements.txt
```

When working on experiments please switch to the experiments branch. This branch is used to run experiments and test the algorithms.


### Running Experiments
To run experiments, use the following command:

```sh
cd experiments
cd cg-vs-ff
pythhon3 lab_templates.py --all
```

To modify the experiments, edit the `lab_templates.py` file. This file contains the configurations for the experiments.
or make your own experiment file and configure in according to your need with the guide of the `lab_templates.py` file.


## License
This project is licensed under the MIT License. See the [LICENSE.md](LICENSE.md) file for details.
