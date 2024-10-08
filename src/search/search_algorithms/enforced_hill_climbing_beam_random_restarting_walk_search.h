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
        std::unique_ptr<EdgeOpenList> open_list;
        std::shared_ptr<Evaluator> evaluator;
        std::vector<std::shared_ptr<Evaluator>> preferred_operator_evaluators;
        std::set<Evaluator *> path_dependent_evaluators;
        bool use_preferred;
        PreferredUsage preferred_usage;

        EvaluationContext current_eval_context;
        int current_phase_start_g;
        int beam_width;
        int max_depth;
        int luby_start_value;
        bool cluster;


        std::map<int, std::pair<int, int>> d_counts;
        int num_ehc_phases;
        int last_num_expanded;

        std::vector<State> const_level_beam;
        int const_level_beam_depth = 0;

        std::unique_ptr<RestartStrategy> r_strategy;
        std::string restart_strategy;

        void insert_successor_into_open_list(
                const EvaluationContext &eval_context,
                int parent_g,
                OperatorID op_id,
                bool preferred);
        void expand(EvaluationContext &eval_context);
        void reach_state(
                const State &parent, OperatorID op_id, const State &state);


        SearchStatus ehc();

        long luby_sequence(long n);

        utils::CountdownTimer *timer;
        std::vector<OperatorID> sample_random_operators(const State &state, std::mt19937 &rng, int num_samples);
        OperatorID sample_random_operator(const State &state, std::mt19937 &rng);
        State sample_random_successor(const State &state, OperatorID op_id, std::mt19937 &rng);
        SearchStatus perform_single_walk(std::mt19937& rng, int current_hvalue);
        SearchStatus perform_beam_walk(std::mt19937& rng, int current_hvalue);

    protected:
        virtual void initialize() override;
        virtual SearchStatus step() override;

    public:
        EnforcedHillClimbingBRRWSearch(
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
