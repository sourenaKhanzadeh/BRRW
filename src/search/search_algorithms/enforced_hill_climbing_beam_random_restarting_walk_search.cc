#include "enforced_hill_climbing_beam_random_restarting_walk_search.h"

#include "../algorithms/ordered_set.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/pref_evaluator.h"
#include "../open_lists/best_first_open_list.h"
#include "../open_lists/tiebreaking_open_list.h"
#include "../plugins/plugin.h"
#include "../task_utils/successor_generator.h"
#include "../utils/logging.h"
#include "../utils/system.h"
#include <random>  // Include for random selection
#include <chrono>  // For time-based operations (if needed)


struct OperatorIDHash {
    std::size_t operator()(const OperatorID& op_id) const noexcept {
        return std::hash<int>()(op_id.get_index());
    }
};

struct OperatorIDEqual {
    bool operator()(const OperatorID &lhs, const OperatorID &rhs) const {
        return lhs.get_index() == rhs.get_index();
    }
};



using namespace std;
using utils::ExitCode;

namespace enforced_hill_climbing_beam_rrw_search {
using GEval = g_evaluator::GEvaluator;
using PrefEval = pref_evaluator::PrefEvaluator;

static shared_ptr<OpenListFactory> create_ehc_open_list_factory(
        utils::Verbosity verbosity, bool use_preferred,
        PreferredUsage preferred_usage) {
    shared_ptr<Evaluator> g_evaluator = make_shared<GEval>(
            "ehc.g_eval", verbosity);

    if (!use_preferred ||
        preferred_usage == PreferredUsage::PRUNE_BY_PREFERRED) {
        return make_shared<standard_scalar_open_list::BestFirstOpenListFactory>(
                g_evaluator, false);
    } else {
        vector<shared_ptr<Evaluator>> evals = {
                g_evaluator, make_shared<PrefEval>(
                        "ehc.pref_eval", verbosity)};
        return make_shared<tiebreaking_open_list::TieBreakingOpenListFactory>(
                evals, false, true);
    }
}

    EnforcedHillClimbingBRRWSearch::EnforcedHillClimbingBRRWSearch(
            const shared_ptr<Evaluator> &h, PreferredUsage preferred_usage,
            const vector<shared_ptr<Evaluator>> &preferred,
            OperatorCost cost_type, int bound, double max_time,
            int beam_width, int max_depth, const string &description, utils::Verbosity verbosity)
            : SearchAlgorithm(
            cost_type, bound, max_time, description, verbosity),
              evaluator(h),
              preferred_operator_evaluators(preferred),
              preferred_usage(preferred_usage),
              current_eval_context(state_registry.get_initial_state(), &statistics),
              current_phase_start_g(-1),
              num_ehc_phases(0),
              last_num_expanded(-1),
              beam_width(beam_width),
              max_depth(max_depth),  // Add max_depth initialization
              current_depth(0) {  // Initialize current_depth to 0
        for (const shared_ptr<Evaluator> &eval : preferred_operator_evaluators) {
            eval->get_path_dependent_evaluators(path_dependent_evaluators);
        }
        evaluator->get_path_dependent_evaluators(path_dependent_evaluators);

        State initial_state = state_registry.get_initial_state();
        for (Evaluator *evaluator : path_dependent_evaluators) {
            evaluator->notify_initial_state(initial_state);
        }
        use_preferred = find(preferred_operator_evaluators.begin(),
                             preferred_operator_evaluators.end(), evaluator) !=
                        preferred_operator_evaluators.end();

        open_list = create_ehc_open_list_factory(
                verbosity, use_preferred, preferred_usage)->create_edge_open_list();
    }


void EnforcedHillClimbingBRRWSearch::reach_state(
        const State &parent, OperatorID op_id, const State &state) {
    for (Evaluator *evaluator : path_dependent_evaluators) {
        evaluator->notify_state_transition(parent, op_id, state);
    }
}

    void EnforcedHillClimbingBRRWSearch::initialize() {
        assert(evaluator);
        log << "Conducting enforced hill-climbing search, (real) bound = "
            << bound << endl;
        if (use_preferred) {
            log << "Using preferred operators for "
                << (preferred_usage == PreferredUsage::RANK_PREFERRED_FIRST ?
                    "ranking successors" : "pruning") << endl;
        }

        bool dead_end = current_eval_context.is_evaluator_value_infinite(evaluator.get());
        statistics.inc_evaluated_states();
        print_initial_evaluator_values(current_eval_context);

        if (dead_end) {
            log << "Initial state is a dead end, no solution" << endl;
            if (evaluator->dead_ends_are_reliable())
                utils::exit_with(ExitCode::SEARCH_UNSOLVABLE);
            else
                utils::exit_with(ExitCode::SEARCH_UNSOLVED_INCOMPLETE);
        }

        SearchNode node = search_space.get_node(current_eval_context.get_state());
        node.open_initial();

        current_phase_start_g = 0;
    }








    void EnforcedHillClimbingBRRWSearch::insert_successor_into_open_list(
        const EvaluationContext &eval_context,
        int parent_g,
        OperatorID op_id,
        bool preferred) {
    OperatorProxy op = task_proxy.get_operators()[op_id];
    int succ_g = parent_g + get_adjusted_cost(op);
    const State &state = eval_context.get_state();
    EdgeOpenListEntry entry = make_pair(state.get_id(), op_id);
    EvaluationContext new_eval_context(
            eval_context, succ_g, preferred, &statistics);
    open_list->insert(new_eval_context, entry);
    statistics.inc_generated_ops();
}

    void EnforcedHillClimbingBRRWSearch::expand(EvaluationContext &eval_context) {
        SearchNode node = search_space.get_node(eval_context.get_state());
        int node_g = node.get_g();

        ordered_set::OrderedSet<OperatorID> preferred_operators;
        if (use_preferred) {
            for (const shared_ptr<Evaluator> &preferred_operator_evaluator : preferred_operator_evaluators) {
                collect_preferred_operators(eval_context,
                                            preferred_operator_evaluator.get(),
                                            preferred_operators);
            }
        }

        if (use_preferred && preferred_usage == PreferredUsage::PRUNE_BY_PREFERRED) {
            for (OperatorID op_id : preferred_operators) {
                insert_successor_into_open_list(
                        eval_context, node_g, op_id, true);
            }
        } else {
            /* The successor ranking implied by RANK_BY_PREFERRED is done
               by the open list. */
            vector<OperatorID> successor_operators;
            successor_generator.generate_applicable_ops(
                    eval_context.get_state(), successor_operators);
            for (OperatorID op_id : successor_operators) {
                bool preferred = use_preferred &&
                                 preferred_operators.contains(op_id);
                insert_successor_into_open_list(
                        eval_context, node_g, op_id, preferred);
            }
        }

        statistics.inc_expanded();
        node.close();
    }


SearchStatus EnforcedHillClimbingBRRWSearch::step() {
    last_num_expanded = statistics.get_expanded();
    search_progress.check_progress(current_eval_context);

    if (check_goal_and_set_plan(current_eval_context.get_state())) {
        return SOLVED;
    }

    expand(current_eval_context);
    return ehcbrrw();
}



    SearchStatus EnforcedHillClimbingBRRWSearch::ehcbrrw() {
        std::random_device rd;  // Random number generator
        std::mt19937 gen(rd()); // Seed for randomness

        int restart_count = 0;
        const int max_restarts = 100; // Set a reasonable limit for restarts

        while (true) {  // Infinite loop for continuous searching with random selection

            // Remove the best entry from the open list
            EdgeOpenListEntry entry = open_list->remove_min();
            StateID parent_state_id = entry.first;
            OperatorID last_op_id = entry.second;

            OperatorProxy last_op = task_proxy.get_operators()[last_op_id];
            State parent_state = state_registry.lookup_state(parent_state_id);
            SearchNode parent_node = search_space.get_node(parent_state);



            // Distance from the start node in this EHC phase
            int d = parent_node.get_g() - current_phase_start_g + get_adjusted_cost(last_op);

            if (parent_node.get_real_g() + last_op.get_cost() >= bound)
                continue;

            // Generate the successor state
            State successor_state = state_registry.get_successor_state(parent_state, last_op);
            statistics.inc_generated();

            SearchNode node = search_space.get_node(successor_state);

            if (node.is_new()) {
                EvaluationContext eval_context(successor_state, &statistics);
                reach_state(parent_state, last_op_id, successor_state);
                statistics.inc_evaluated_states();

                if (eval_context.is_evaluator_value_infinite(evaluator.get())) {
                    node.mark_as_dead_end();
                    statistics.inc_dead_ends();
                    continue;
                }

                int h = eval_context.get_evaluator_value(evaluator.get());
                node.open_new_node(parent_node, last_op, get_adjusted_cost(last_op));

                current_depth++;

                // Restart when the maximum depth is reached
                if (current_depth >= max_depth) {
                    log << "Maximum depth reached, performing random restart." << endl;
                    restart_count++;

                    if (restart_count >= max_restarts) {
                        log << "Maximum number of restarts reached, terminating search." << endl;
                        return FAILED;
                    }

                    initialize();
                    current_depth = 0;  // Reset depth counter after restart
                    continue;
                }

                // Check if we have reached a goal state
                if (check_goal_and_set_plan(successor_state)) {
                    return SOLVED;  // Goal found
                }

                // Perform Beam Search: limit the expansion to beam width
                if (current_depth % beam_width == 0) {
                    // Here you can add logic to randomly shuffle or select a subset of nodes
                    // if the beam width is reached, or perform a random walk.
                    std::vector<EdgeOpenListEntry> entries_to_expand;

                    // Collect entries to expand within the beam width
                    for (int i = 0; i < beam_width && !open_list->empty(); ++i) {
                        entries_to_expand.push_back(open_list->remove_min());
                    }

                    // Randomly expand one of the selected entries
                    std::shuffle(entries_to_expand.begin(), entries_to_expand.end(), gen);
                    for (const auto &entry : entries_to_expand) {
                        // Handle the expansion of each node here
                        // For each entry, apply the operator, generate successor states, and so on
                    }
                }
            }
            if (open_list->empty()) {
                log << "Open list is empty, performing random restart." << endl;
                restart_count++;

                if (restart_count >= max_restarts) {
                    log << "Maximum number of restarts reached, terminating search." << endl;
                    return FAILED;
                }

                initialize();  // Random restart when open list is empty
                current_depth = 0;  // Reset depth counter
                continue;  // Restart the loop after initialization
            }
        }

        log << "No solution - FAILED" << endl;
        return FAILED;
    }












    void EnforcedHillClimbingBRRWSearch::print_statistics() const {
        statistics.print_detailed_statistics();

        log << "EHC phases: " << num_ehc_phases << endl;
        assert(num_ehc_phases != 0);
        log << "Average expansions per EHC phase: "
            << static_cast<double>(statistics.get_expanded()) / num_ehc_phases
            << endl;

        for (auto count : d_counts) {
            int depth = count.first;
            int phases = count.second.first;
            assert(phases != 0);
            int total_expansions = count.second.second;
            log << "EHC phases of depth " << depth << ": " << phases
                << " - Avg. Expansions: "
                << static_cast<double>(total_expansions) / phases << endl;
        }
}



    class EnforcedHillClimbingSearchFeature
            : public plugins::TypedFeature<SearchAlgorithm, EnforcedHillClimbingBRRWSearch> {
    public:
        EnforcedHillClimbingSearchFeature() : TypedFeature("ehcbrrw") {
            document_title("Beam Search with Random Walk and Enforced Hill Climbing");
            document_synopsis("");

            add_option<shared_ptr<Evaluator>>("h", "heuristic");
            add_option<PreferredUsage>(
                    "preferred_usage",
                    "preferred operator usage",
                    "prune_by_preferred");
            add_list_option<shared_ptr<Evaluator>>(
                    "preferred",
                    "use preferred operators of these evaluators",
                    "[]");
            add_option<int>(
                    "beam_width",
                    "Width of the beam in beam search. Set to 1 for random restarting walk.",
                    "1");
            add_option<int>(
                    "max_depth",
                    "Maximum depth before restarting the search.",
                    "100");  // Default to 100 if not specified
            add_search_algorithm_options_to_feature(*this, "ehcbrrw");
        }

        virtual shared_ptr<EnforcedHillClimbingBRRWSearch> create_component(
                const plugins::Options &opts,
                const utils::Context &) const override {
            return make_shared<EnforcedHillClimbingBRRWSearch>(
                    opts.get<shared_ptr<Evaluator>>("h"),
                    opts.get<PreferredUsage>("preferred_usage"),
                    opts.get_list<shared_ptr<Evaluator>>("preferred"),
                    opts.get<OperatorCost>("cost_type"),
                    opts.get<int>("bound"),
                    opts.get<double>("max_time"),
                    opts.get<int>("beam_width"),
                    opts.get<int>("max_depth"),  // Pass max_depth to constructor
                    opts.get<string>("description"),
                    opts.get<utils::Verbosity>("verbosity"));
        }
    };


static plugins::FeaturePlugin<EnforcedHillClimbingSearchFeature> _plugin;

static plugins::TypedEnumPlugin<PreferredUsage> _enum_plugin({
         {"prune_by_preferred",
                 "prune successors achieved by non-preferred operators"},
         {"rank_preferred_first",
                 "first insert successors achieved by preferred operators, "
                 "then those by non-preferred operators"}
 });
}
