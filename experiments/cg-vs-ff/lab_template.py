import sys
import subprocess
from lab.experiment import Experiment
from downward.experiment import FastDownwardExperiment, FastDownwardRun, FastDownwardAlgorithm
from downward.cached_revision import CachedFastDownwardRevision
from downward.reports.absolute import AbsoluteReport
from downward import suites
import os
import project
import custom_parser


REPO = project.get_repo_base()
BENCHMARKS_DIR = os.path.join(REPO, os.pardir, "benchmarks")
SUITE = ["tpp"]
ENV = project.LocalEnvironment(processes=2)
CONFIGS = [
    ("ehcbrrw", ["--search", 'ehcbrrw(add(), max_depth=100, beam_width=2, bound=NULL)']),
]

N_RUNS = 5


# CONFIGS = [
#     ("simple_search", ["--search", "astar(lmcut())"]) #max_time=5
# ]
BUILD_OPTIONS = []
DRIVER_OPTIONS = ["--overall-time-limit", "5m"]
# DRIVER_OPTIONS = []
REVISION_CACHE = project.DIR / "data" / "revision-cache"
REV_NICKS = [
    ("main", ""),
]
ATTRIBUTES = [
    "error",
    "run_dir",
    "search_start_time",
    "search_start_memory",
    "total_time",
    "h_values",
    "coverage",
    "expansions",
    "memory",
    project.EVALUATIONS_PER_TIME,
]

problems = []

exp = Experiment(environment=ENV)
for rev, rev_nick in REV_NICKS:
    cached_rev = CachedFastDownwardRevision(REVISION_CACHE, REPO, rev, BUILD_OPTIONS)
    cached_rev.cache()
    exp.add_resource("", cached_rev.path, cached_rev.get_relative_exp_path())
    for config_nick, config in CONFIGS:
        for i in range(N_RUNS):

            for task in suites.build_suite(BENCHMARKS_DIR, SUITE):
                algo_name = f"{rev_nick}-{config_nick}-{task.domain}-{task.problem}-{i}" if rev_nick else f"{config_nick}-{task.domain}-{task.problem}-{i}"
                domain_prob = f"{task.domain}:{task.problem}"
                config[-1] = config[-1].replace("bound=NULL", f"bound={domain_prob}")
                algo = FastDownwardAlgorithm(
                    algo_name,
                    cached_rev,
                    DRIVER_OPTIONS,
                    config,
                )
                run = FastDownwardRun(exp, algo, task)
                exp.add_run(run)

# exp.add_suite(BENCHMARKS_DIR, SUITE)


exp.add_parser(project.FastDownwardExperiment.EXITCODE_PARSER)
exp.add_parser(project.FastDownwardExperiment.TRANSLATOR_PARSER)
exp.add_parser(project.FastDownwardExperiment.SINGLE_SEARCH_PARSER)
exp.add_parser(custom_parser.get_parser())
exp.add_parser(project.FastDownwardExperiment.PLANNER_PARSER)


exp.add_step("build", exp.build)
exp.add_step("start", exp.start_runs)
exp.add_step("parse", exp.parse)
exp.add_fetcher(name="fetch")

project.add_absolute_report(
    exp,
    attributes=ATTRIBUTES,
    filter=[project.add_evaluations_per_time, project.group_domains],
)



exp.run_steps()
