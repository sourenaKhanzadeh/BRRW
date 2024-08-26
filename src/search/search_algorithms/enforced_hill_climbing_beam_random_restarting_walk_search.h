#ifndef SEARCH_ALGORITHMS_ENFORCED_HILL_CLIMBING_SEARCH_H
#define SEARCH_ALGORITHMS_ENFORCED_HILL_CLIMBING_SEARCH_H

#include "../evaluation_context.h"
#include "../open_list.h"
#include "../search_algorithm.h"

#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>
#include <random>

namespace plugins {
class Options;
}

namespace enforced_hill_climbing_beam_rrw_search {
enum class PreferredUsage {
    PRUNE_BY_PREFERRED,
    RANK_PREFERRED_FIRST
};

/*
  Enforced hill-climbing-Beam-RRW with deferred evaluation.

  TODO: We should test if this lazy implementation really has any benefits over
  an eager one. We hypothesize that both versions need to evaluate and store
  the same states anyways.
*/
class EnforcedHillClimbingBRRWSearch : public SearchAlgorithm {
    std::unique_ptr<EdgeOpenList> open_list;

    std::shared_ptr<Evaluator> evaluator;
    std::vector<std::shared_ptr<Evaluator>> preferred_operator_evaluators;
    std::set<Evaluator *> path_dependent_evaluators;
    bool use_preferred;
    PreferredUsage preferred_usage;

    EvaluationContext current_eval_context;
    int current_phase_start_g;
    int beam_width = 1;
    int max_depth = 1000;


    std::vector<StateID> path_to_minima;

    // Statistics
    std::map<int, std::pair<int, int>> d_counts;
    int num_ehc_phases;
    int last_num_expanded;

    void insert_successor_into_open_list(
        const EvaluationContext &eval_context,
        int parent_g,
        OperatorID op_id,
        bool preferred);
    void expand(EvaluationContext &eval_context);
    void reach_state(
        const State &parent, OperatorID op_id, const State &state);
    SearchStatus ehcbrrw();

    OperatorID sample_random_operator(const State &state, std::mt19937 &rng);

    SearchStatus random_restart_walk();
    SearchStatus beam_search(std::mt19937 &rng);
    SearchStatus ehc_random_walk_search();
    SearchStatus ehc();

    long luby_sequence(long n);


protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;



public:
    EnforcedHillClimbingBRRWSearch(
        const std::shared_ptr<Evaluator> &h,
        PreferredUsage preferred_usage,
        const std::vector<std::shared_ptr<Evaluator>> &preferred,
        OperatorCost cost_type, int bound, double max_time,
        int beam_width, int max_depth, const std::string &description, utils::Verbosity verbosity);
    virtual void print_statistics() const override;

};
}

#endif
