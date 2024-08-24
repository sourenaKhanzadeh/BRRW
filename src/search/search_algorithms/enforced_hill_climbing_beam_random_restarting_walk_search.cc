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

        int c = max_depth;  // Maximum depth for random walk
        int B = beam_width; // Beam width for beam search
        State initial_state = state_registry.get_initial_state();

        // Check if the initial state is already the goal state
        if (check_goal_and_set_plan(initial_state)) {
            log << "Initial state is the goal state." << endl;
            return SOLVED;
        }

        // Beam to store the best states
        vector<pair<std::shared_ptr<SearchNode>, EvaluationContext>> beam;

        // Initialize the beam with the initial state
        EdgeOpenListEntry entry(initial_state.get_id(), OperatorID::no_operator);
        StateID parent_state_id = entry.first;
        State parent_state = state_registry.lookup_state(parent_state_id);
        std::shared_ptr<SearchNode> parent_node = std::make_shared<SearchNode>(search_space.get_node(parent_state));
        EvaluationContext eval_context(parent_state, &statistics);

        beam.push_back({parent_node, eval_context});

        while (!beam.empty()) {
            vector<pair<std::shared_ptr<SearchNode>, EvaluationContext>> next_beam;

            // Expand each node in the current beam
            for (auto &beam_entry : beam) {
                auto &current_node = beam_entry.first;
                auto &current_eval_context = beam_entry.second;

                // Expand the current node
                for (int i = 0; i < task_proxy.get_operators().size(); ++i) {
                    OperatorProxy op = task_proxy.get_operators()[i];
                    OperatorID op_id = OperatorID(op.get_id());
                    State next_state = state_registry.get_successor_state(current_node->get_state(), op);
                    statistics.inc_generated();

                    SearchNode next_node = search_space.get_node(next_state);

                    if (next_node.is_new()) {
                        EvaluationContext next_eval_context(next_state, &statistics);
                        reach_state(current_node->get_state(), op_id, next_state);
                        statistics.inc_evaluated_states();

                        if (!next_eval_context.is_evaluator_value_infinite(evaluator.get())) {
                            next_node.open_new_node(*current_node, op, get_adjusted_cost(op));

                            if (next_beam.size() < B) {
                                next_beam.push_back({std::make_shared<SearchNode>(std::move(next_node)), std::move(next_eval_context)});
                            } else {
                                // Replace the worst node in the beam if the new node is better
                                auto worst_node = std::max_element(next_beam.begin(), next_beam.end(),
                                                                   [](const auto &a, const auto &b) {
                                                                       return a.first->get_g() < b.first->get_g();
                                                                   });

                                if (next_node.get_g() < worst_node->first->get_g()) {
                                    // Replace the worst node in the beam with the next node
                                    worst_node->first.~shared_ptr();
                                    worst_node->second.~EvaluationContext();
                                    new (&worst_node->first) std::shared_ptr<SearchNode>(std::make_shared<SearchNode>(std::move(next_node)));
                                    new (&worst_node->second) EvaluationContext(std::move(next_eval_context));
                                }
                            }

                            // Check if the goal state is found
                            if (check_goal_and_set_plan(next_state)) {
                                log << "Goal state found." << endl;
                                return SOLVED;
                            }
                        }
                    }
                }
            }

            // Randomly select a node from the beam to continue searching
            if (!next_beam.empty()) {
                std::uniform_int_distribution<int> dist(0, next_beam.size() - 1);
                int selected_index = dist(gen);
                auto selected_node = next_beam[selected_index];
                current_eval_context = std::move(selected_node.second);
                beam = {std::move(selected_node)};
            } else {
                log << "Beam is empty. Restarting search." << endl;
                return IN_PROGRESS;
            }
        }

        log << "No solution found - FAILED" << endl;
        return FAILED;
    }


    OperatorID EnforcedHillClimbingBRRWSearch::sample_random_operator(const State &state, std::mt19937 &rng) {
        vector<OperatorID> applicable_ops;
        successor_generator.generate_applicable_ops(state, applicable_ops);
        std::uniform_int_distribution<int> dist(0, applicable_ops.size() - 1);
        return applicable_ops[dist(rng)];
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
