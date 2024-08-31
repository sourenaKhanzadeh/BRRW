<img src="misc/images/fast-downward.svg" width="800" alt="Fast Downward">

Fast Downward is a domain-independent classical planning system.

Copyright 2003-2023 Fast Downward contributors (see below).

For further information:
- Fast Downward website: <https://www.fast-downward.org>
- Report a bug or file an issue: <https://issues.fast-downward.org>
- Fast Downward mailing list: <https://groups.google.com/forum/#!forum/fast-downward>
- Fast Downward main repository: <https://github.com/aibasel/downward>


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
This project is designed to implement and evaluate various search algorithms, including beam search and random restarting walk (RRW), for solving planning problems. The project includes both a C++ codebase for core algorithms and Python scripts for experiments and testing.

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
├── bash
├── build.py
├── build_configs.py
├── builds
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
- **Xperiment101.py**: Various Python scripts for running experiments and testing algorithms.
- **build.py**: Script for building the project.
- **fast-downward.py**: Entry point for running the Fast Downward planner.
- **parser/**: Contains utilities for parsing and handling experiment data.
- **src/**: Contains the source code for the core algorithms and tools.

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
- **BRRW**: Contains implementations and configurations for Beam Random Restart Walk.

## Build Instructions
To build the project, follow these steps:

1. **Navigate to the src directory**:
   ```sh
   cd src
   ```

2. **Run CMake to configure the build**:
   ```sh
   cmake .
   ```

3. **Build the project using Make**:
   ```sh
   make
   ```

4. **Run the Fast Downward planner**:
   ```sh
   ./fast-downward.py <domain.pddl> <problem.pddl> --search "ehcbrrw(...)"
   ```

## Running Experiments
To run experiments, use the provided Python scripts in the main project directory. For example:

```sh
python3 Xperiment101.py --beam_width 5 --max_depth 10
```

These scripts will execute the search algorithms on the provided benchmarks and record the results.

## License
This project is licensed under the MIT License. See the [LICENSE.md](LICENSE.md) file for details.
