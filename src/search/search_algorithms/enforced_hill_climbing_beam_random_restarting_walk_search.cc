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


namespace utils {
    bool check_time(int max_time, const std::chrono::steady_clock::time_point& start_time) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        return elapsed_time > max_time;
    }
}



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
        int initial_max_depth = max_depth;  // Store the initial max depth
        const int depth_increment = initial_max_depth;  // Increment max_depth by its initial value after successful random walk
        int start_g_value = current_eval_context.get_g_value();  // Initialize this at the beginning of your search phase
        int depth = 0;
        if (beam_width == 1) {
            while(true){
            int subgoal_h = current_eval_context.get_evaluator_value(evaluator.get());
            log << "Starting EHC phase with initial subgoal_h: " << subgoal_h << " and max_depth: " << max_depth
                << endl;

            // Check if the current state is a goal state
            if (check_goal_and_set_plan(current_eval_context.get_state())) {
                log << "Goal state reached with h: " << subgoal_h << endl;
                return SOLVED;
            }

            // Perform random walk to find a new minima (subgoal)
            log << "Starting random walk towards minima..." << endl;
            SearchStatus walk_status;
            do {
                walk_status = random_walk_to_subgoal(subgoal_h);
                if (walk_status == FAILED) {
                    log << "Random walk failed to find a better subgoal. Returning FAILED." << endl;
                    return FAILED;
                } else if (walk_status == IN_PROGRESS)
                    break;
            } while (current_eval_context.get_evaluator_value(evaluator.get()) >= subgoal_h);

            // After successful random walk, update depth and max_depth
            start_g_value = current_eval_context.get_g_value();  // Update start_g_value after random walk
            depth = current_eval_context.get_g_value() - start_g_value;
            log << "Random walk successfully reached subgoal. Incrementing max_depth by " << depth_increment
                << ". New depth: " << depth << endl;
            max_depth += depth_increment;  // Increment the max_depth by the specified amount

            // Resume search from the new subgoal found by the random walk    while (!open_list->empty()) {
            EdgeOpenListEntry entry = open_list->remove_min();
            StateID parent_state_id = entry.first;
            OperatorID last_op_id = entry.second;
            OperatorProxy last_op = task_proxy.get_operators()[last_op_id];

            State parent_state = state_registry.lookup_state(parent_state_id);
            SearchNode parent_node = search_space.get_node(parent_state);

            depth = parent_node.get_g() - start_g_value;
            log << "Calculated Depth: " << depth << endl;

            if (depth >= max_depth) {
                log << "Reached max depth: " << max_depth << ". Returning FAILED." << endl;
                return FAILED;
            }

            if (parent_node.get_real_g() + last_op.get_cost() >= bound) {
                continue;
            }

            State state = state_registry.get_successor_state(parent_state, last_op);
            statistics.inc_generated();

            SearchNode node = search_space.get_node(state);

            if (node.is_new()) {
                EvaluationContext eval_context(state, &statistics);
                reach_state(parent_state, last_op_id, state);
                statistics.inc_evaluated_states();

                if (eval_context.is_evaluator_value_infinite(evaluator.get())) {
                    node.mark_as_dead_end();
                    statistics.inc_dead_ends();
                    continue;
                }

                int h = eval_context.get_evaluator_value(evaluator.get());
                log << "Evaluating state: h = " << h << ", subgoal_h = " << subgoal_h << endl;
                node.open_new_node(parent_node, last_op, get_adjusted_cost(last_op));

                // Check if this state is a goal
                if (check_goal_and_set_plan(eval_context.get_state())) {
                    log << "Goal state reached with h: " << h << endl;
                    return SOLVED;
                }

                if (h < subgoal_h) {
                    log << "Found a new subgoal with h: " << h << endl;
                    subgoal_h = h;
                    current_eval_context = std::move(eval_context);
                    open_list->clear();  // Clear the open list to restart search from new subgoal
                    start_g_value = node.get_g();  // Update start_g_value for the new phase
                    return IN_PROGRESS;  // Restart the search from this new subgoal
                } else {
                    expand(eval_context);
                }
                // If no further progress is possible, return failure
                if (depth >= max_depth) {
                    log << "No further improvement found at max_depth - FAILED" << endl;
                    return FAILED;
                } else if (walk_status == IN_PROGRESS) {
                    log << "No further improvement found - IN_PROGRESS" << endl;
                    return IN_PROGRESS;
                } else {
                    log << "No further improvement found - TIMEOUT" << endl;
                    return TIMEOUT;
                }
            }
            }
        }
        else{
                while (true) {
            int subgoal_h = current_eval_context.get_evaluator_value(evaluator.get());
            log << "Starting EHC phase with initial subgoal_h: " << subgoal_h << " and max_depth: " << max_depth
                << endl;

            // Check if the current state is a goal state
            if (check_goal_and_set_plan(current_eval_context.get_state())) {
                log << "Goal state reached with h: " << subgoal_h << endl;
                return SOLVED;
            }


                // Perform beam search with restarting random walk for each beam
                log << "Starting beam search with beam_width = " << beam_width << endl;

                std::vector<StateID> beam;  // Store the most promising nodes using StateID
                beam.push_back(current_eval_context.get_state().get_id());

                while (!beam.empty()) {
                    std::vector<std::pair<StateID, int>> next_beam;  // Pair of StateID and heuristic value

                    for (StateID parent_state_id: beam) {
                        SearchNode parent_node = search_space.get_node(
                                state_registry.lookup_state(parent_state_id));
                        State parent_state = state_registry.lookup_state(parent_state_id);

                        for (OperatorProxy op: task_proxy.get_operators()) {
                            if (parent_node.get_real_g() + op.get_cost() >= bound) {
                                continue;
                            }

                            State state = state_registry.get_successor_state(parent_state, op);
                            statistics.inc_generated();

                            SearchNode node = search_space.get_node(state);

                            if (node.is_new()) {
                                EvaluationContext eval_context(state, &statistics);
                                reach_state(parent_state, OperatorID(op.get_id()), state);
                                statistics.inc_evaluated_states();

                                if (eval_context.is_evaluator_value_infinite(evaluator.get())) {
                                    node.mark_as_dead_end();
                                    statistics.inc_dead_ends();
                                    continue;
                                }

                                int h = eval_context.get_evaluator_value(evaluator.get());
                                log << "Evaluating state: h = " << h << ", subgoal_h = " << subgoal_h << endl;
                                node.open_new_node(parent_node, op, get_adjusted_cost(op));

                                // Check if this state is a goal
                                if (check_goal_and_set_plan(eval_context.get_state())) {
                                    log << "Goal state reached with h: " << h << endl;
                                    return SOLVED;
                                }

                                // Perform random walk for this beam
                                log << "Starting random walk for this beam"<< " " << beam_width << "..." << endl;
                                SearchStatus walk_status;
                                do {
                                    walk_status = random_walk_to_subgoal(h);
                                    if (walk_status == FAILED) {
                                        log
                                                << "Random walk failed to find a better subgoal for this beam. Skipping to the next beam."
                                                << endl;
                                        break;
                                    } else if (walk_status == IN_PROGRESS) {
                                        break;
                                    }
                                } while (current_eval_context.get_evaluator_value(evaluator.get()) >= h);

                                if (walk_status == IN_PROGRESS) {
                                    next_beam.emplace_back(state.get_id(), h);
                                }

                                // Prune the next beam if it exceeds beam_width
                                if (next_beam.size() > static_cast<size_t>(beam_width)) {
                                    // Sort by heuristic value and keep only the best nodes
                                    std::partial_sort(next_beam.begin(), next_beam.begin() + beam_width,
                                                      next_beam.end(),
                                                      [](const std::pair<StateID, int> &a,
                                                         const std::pair<StateID, int> &b) {
                                                          return a.second < b.second;
                                                      });
                                    // Erase the elements exceeding the beam width
                                    next_beam.erase(next_beam.begin() + beam_width, next_beam.end());
                                }
                            }
                        }
                    }

                    if (next_beam.empty()) {
                        log << "Beam search exhausted with no improvement. Returning FAILED." << endl;
                        return FAILED;
                    }

                    // Update the current beam with the next beam's StateIDs
                    beam.clear();
                    for (const auto &pair: next_beam) {
                        beam.push_back(pair.first);
                    }

                    // Calculate the new depth
                    depth = search_space.get_node(state_registry.lookup_state(beam.front())).get_g() -
                            start_g_value;
                    log << "Updated Depth: " << depth << endl;

                    if (depth >= max_depth) {
                        log << "Reached max depth: " << max_depth << ". Returning FAILED." << endl;
                        return FAILED;
                    }
                }
            }

        }

    }











    SearchStatus EnforcedHillClimbingBRRWSearch::random_walk_to_subgoal(int subgoal_h) {
        // Set a limit for the number of random steps to avoid infinite loops
        const int max_random_steps = 1000;
        int steps_taken = 0;

        State current_state = current_eval_context.get_state();
        SearchNode current_node = search_space.get_node(current_state);

        while (steps_taken < max_random_steps) {
            int current_h = current_eval_context.get_evaluator_value(evaluator.get());
            log << "Step " << steps_taken << ": Current heuristic value: " << current_h << endl;

            if (current_h <= subgoal_h) {
                // Successfully reached the subgoal
                return IN_PROGRESS;
            }

            // Generate applicable operators
            std::vector<OperatorID> applicable_ops;
            for (OperatorProxy op : task_proxy.get_operators()) {
                bool is_applicable = true;
                for (FactProxy precondition : op.get_preconditions()) {
                    // Compare the value of the state's variable with the precondition's value
                    if (current_state[precondition.get_variable().get_id()].get_value() != precondition.get_value()) {
                        is_applicable = false;
                        break;
                    }
                }
                if (is_applicable) {
                    applicable_ops.push_back(OperatorID(op.get_id()));
                }
            }


            if (applicable_ops.empty()) {
                // Dead end: no applicable operators
                log << "Random walk failed - dead end reached" << endl;
                return FAILED;
            }

            // Randomly select one operator
            OperatorID random_op_id = applicable_ops[rand() % applicable_ops.size()];
            OperatorProxy random_op = task_proxy.get_operators()[random_op_id];
            State next_state = state_registry.get_successor_state(current_state, random_op);
            SearchNode next_node = search_space.get_node(next_state);

            if (next_node.is_dead_end()) {
                // Skip dead ends
                continue;
            }

            // Evaluate the new state
            EvaluationContext eval_context(next_state, &statistics);
            statistics.inc_evaluated_states();

            if (eval_context.is_evaluator_value_infinite(evaluator.get())) {
                next_node.mark_as_dead_end();
                statistics.inc_dead_ends();
                continue;
            }

            // Open the new node and perform the transition to next state
            next_node.open_new_node(current_node, random_op, get_adjusted_cost(random_op));
            reach_state(current_state, OperatorID(random_op.get_id()), next_state);

            // Transition to the next state and continue the random walk
            current_state = next_state;
            current_eval_context = std::move(eval_context);

            steps_taken++;
        }

        log << "Random walk failed - max steps reached" << endl;
        return TIMEOUT;
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
