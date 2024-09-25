#ifndef SEARCH_ALGORITHMS_ENFORCED_HILL_CLIMBING_SEARCH_H
#define SEARCH_ALGORITHMS_ENFORCED_HILL_CLIMBING_SEARCH_H

#include "../evaluation_context.h"
#include "../open_list.h"
#include "../search_algorithm.h"
#include "../utils/countdown_timer.h"

#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>
#include <random>
#include <cstdint>
#include <string>  // Include for std::string

namespace plugins {
    class Options;
}

namespace enforced_hill_climbing_beam_rrw_search {

    enum class RestartStrategyType {
        NONE,
        LUBY
    };


    enum class PreferredUsage {
        PRUNE_BY_PREFERRED,
        RANK_PREFERRED_FIRST
    };

    class RestartStrategy {
    public:
        RestartStrategy();
        RestartStrategy(long sequence_start_value);
        virtual ~RestartStrategy();

        virtual uint64_t next_sequence_value() = 0;
        virtual uint64_t sequence(long sequence_number) = 0;
        virtual void reset_sequence();

    protected:
        long internal_sequence_count;
    };

    class LubyRestartStrategy : public RestartStrategy {
    public:
        LubyRestartStrategy();
        LubyRestartStrategy(long sequence_start_value);
        ~LubyRestartStrategy();

        virtual uint64_t next_sequence_value() override;
        virtual uint64_t sequence(long sequence_number) override;
    };

    class EnforcedHillClimbingBRRWSearch : public SearchAlgorithm {
        shared_ptr<utils::RandomNumberGenerator> rng;
        std::unique_ptr<StateOpenList> open_list;
        std::shared_ptr<Evaluator> evaluator;
        std::vector<std::shared_ptr<Evaluator>> preferred_operator_evaluators;
        std::set<Evaluator *> path_dependent_evaluators;
        bool use_preferred;
        PreferredUsage preferred_usage;

        bool improvement_found;
        EvaluationContext current_eval_context;
        int current_phase_start_g;
        int beam_width;
        int max_depth;
        int luby_start_value;
        bool cluster;


        std::map<int, std::pair<int, int>> d_counts;
        int num_ehc_phases;
        int last_num_expanded;

        std::unique_ptr<RestartStrategy> r_strategy;
        std::string restart_strategy;

        bool insert_successor_into_open_list(
                EvaluationContext &eval_context,
                int parent_g,
                OperatorID op_id,
                bool preferred);
        bool expand(EvaluationContext &eval_context);
        void reach_state(
                const State &parent, OperatorID op_id, const State &state);
        SearchStatus rrw();
        SearchStatus ehc();

        OperatorID sample_random_operator(const State &state, std::mt19937 &rng);

        SearchStatus random_restart_walk();
        SearchStatus beam_search(std::mt19937 &rng);

        long luby_sequence(long n);

        utils::CountdownTimer *timer;

    protected:
        virtual void initialize() override;
        virtual SearchStatus step() override;

    public:
        EnforcedHillClimbingBRRWSearch(
                int random_seed,
                const std::shared_ptr<Evaluator> &h,
                PreferredUsage preferred_usage,
                const std::vector<std::shared_ptr<Evaluator>> &preferred,
                OperatorCost cost_type, int bound, double max_time,
                int beam_width, int max_depth, const std::string & restart_strategy, int luby_start_value,
                bool cluster,
                const std::string &description, utils::Verbosity verbosity);

        virtual void print_statistics() const override;
        virtual void search() override;
    };
}

#endif
