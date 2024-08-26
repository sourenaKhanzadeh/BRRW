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
              max_depth(max_depth){  // Initialize current_depth to 0
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

        return ehc();
    }

    SearchStatus EnforcedHillClimbingBRRWSearch::ehc() {
        while (!open_list->empty()) {
            EdgeOpenListEntry entry = open_list->remove_min();
            StateID parent_state_id = entry.first;
            OperatorID last_op_id = entry.second;
            OperatorProxy last_op = task_proxy.get_operators()[last_op_id];

            State parent_state = state_registry.lookup_state(parent_state_id);
            SearchNode parent_node = search_space.get_node(parent_state);

            // Calculate the distance d from the initial node in this EHC phase
            int d = parent_node.get_g() - current_phase_start_g + get_adjusted_cost(last_op);

            // Check if we exceed the bound
            if (parent_node.get_real_g() + last_op.get_cost() >= bound)
                continue;

            State state = state_registry.get_successor_state(parent_state, last_op);
            statistics.inc_generated();

            SearchNode node = search_space.get_node(state);

            if (node.is_new()) {
                EvaluationContext eval_context(state, &statistics);
                reach_state(parent_state, last_op_id, state);
                statistics.inc_evaluated_states();

                // Check if the state is a dead end
                if (eval_context.is_evaluator_value_infinite(evaluator.get())) {
                    node.mark_as_dead_end();
                    statistics.inc_dead_ends();
                    continue;
                }

                // Get the heuristic value of the current state
                int h = eval_context.get_evaluator_value(evaluator.get());
                node.open_new_node(parent_node, last_op, get_adjusted_cost(last_op));



                // If we found a better node, reset the open list and start a new phase
                if (h < current_eval_context.get_evaluator_value(evaluator.get())) {
                    ++num_ehc_phases;
                    if (d_counts.count(d) == 0) {
                        d_counts[d] = make_pair(0, 0);
                    }
                    pair<int, int> &d_pair = d_counts[d];
                    d_pair.first += 1;
                    d_pair.second += statistics.get_expanded() - last_num_expanded;

                    current_eval_context = std::move(eval_context);
                    open_list->clear();
                    current_phase_start_g = node.get_g();
                    log << "Starting new EHC phase with depth " << d << endl;
                    return IN_PROGRESS;
                } else {
//                    expand(eval_context);
                }
            }
        }

        if (beam_width == 1) {
//             Perform random walk to escape plateau
           return random_restart_walk();
        }
        // Perform beam search to escape plateau
        std::random_device rd;
        std::mt19937 rng(rd());
        SearchNode current_node = search_space.get_node(current_eval_context.get_state());
        return beam_search(rng);
    }





    SearchStatus EnforcedHillClimbingBRRWSearch::random_restart_walk() {
        int current_hvalue = current_eval_context.get_evaluator_value(evaluator.get());
        EvaluationContext eval_context = current_eval_context;
        int h_value = current_hvalue;
        log << "Entering Random Walk to escape plateau with heuristic value: " << h_value << endl;

        vector<State> walk;
        uint64_t MAX_TIMESTEP = 100; // Fixed max timestep for the random walk

        do {
            uint64_t timestep = 0;
            eval_context = current_eval_context;
            walk.clear();

            while (h_value >= current_hvalue && timestep < MAX_TIMESTEP) {
                vector<OperatorID> ops;
                successor_generator.generate_applicable_ops(current_eval_context.get_state(), ops);

                if (ops.empty()) {
                    log << "Dead end encountered during random walk..." << endl;
                    break;  // Dead end, restart the walk
                }

                OperatorID random_op = ops.at(rand() % ops.size());
                State next_state = state_registry.get_successor_state(current_eval_context.get_state(), task_proxy.get_operators()[random_op]);

                eval_context = EvaluationContext(next_state, &statistics);
                statistics.inc_evaluated_states();
                statistics.inc_expanded();
                statistics.inc_generated();

                h_value = eval_context.get_evaluator_value(evaluator.get());

                walk.push_back(next_state);

                if (h_value < current_hvalue) {
                    current_eval_context = eval_context;
                    log << "Found better state during random walk, restarting EHC..." << endl;
                    return IN_PROGRESS;  // Restart EHC from the new state
                }

                ++timestep;
            }
        } while (h_value >= current_hvalue);

        current_eval_context = move(eval_context);
        return IN_PROGRESS;
    }




    long EnforcedHillClimbingBRRWSearch::luby_sequence(long sequence_number) {
        long focus = 2L;
        while (sequence_number > (focus - 1)) {
            focus = focus << 1;
        }

        if (sequence_number == (focus - 1)) {
            return focus >> 1;
        }
        else {
            return luby_sequence(sequence_number - (focus >> 1) + 1);
        }
    }



















    SearchStatus EnforcedHillClimbingBRRWSearch::beam_search(std::mt19937 &rng) {
        int current_hvalue = current_eval_context.get_evaluator_value(evaluator.get());
        EvaluationContext eval_context = current_eval_context;
        log << "Starting Beam Search with initial heuristic value: " << current_hvalue << endl;

        vector<State> beam;  // The current beam of states
        beam.push_back(current_eval_context.get_state()); // Start with the initial state

        int depth = 0;

        while (depth < max_depth) {
            vector<pair<int, State>> successors;  // To store successors and their heuristic values

            // Expand each state in the current beam and collect successors
            for (const State &current_state : beam) {
                eval_context = EvaluationContext(current_state, &statistics);

                vector<OperatorID> ops;
                successor_generator.generate_applicable_ops(eval_context.get_state(), ops);

                if (ops.empty()) {
                    log << "All operators pruned for current state, continuing to next state in beam..." << endl;
                    continue;  // Skip to the next state in the beam
                }

                // Generate all successors for the current state
                for (OperatorID op_id : ops) {
                    const OperatorProxy op = task_proxy.get_operators()[op_id];
                    State next_state = state_registry.get_successor_state(eval_context.get_state(), op);

                    EvaluationContext successor_eval_context(next_state, &statistics);
                    statistics.inc_evaluated_states();
                    statistics.inc_expanded();
                    statistics.inc_generated();

                    int state_hvalue = successor_eval_context.get_evaluator_value(evaluator.get());

                    // Store the successor and its heuristic value
                    successors.push_back(make_pair(state_hvalue, next_state));
                }
            }

            // If no successors were found, terminate the search
            if (successors.empty()) {
                log << "No successors found, beam search failed." << endl;
                return FAILED;
            }

            // Sort successors based on their heuristic values in ascending order
            sort(successors.begin(), successors.end(), [](const pair<int, State> &a, const pair<int, State> &b) {
                return a.first < b.first;
            });

            // Keep only the best successors according to the beam width
            beam.clear();
            for (int i = 0; i < min(beam_width, static_cast<int>(successors.size())); ++i) {
                beam.push_back(successors[i].second);

                // If we find a better state than the current, restart EHC
                if (successors[i].first < current_hvalue) {
                    current_eval_context = EvaluationContext(successors[i].second, &statistics);
                    current_hvalue = successors[i].first;
                    log << "Found better state with heuristic " << current_hvalue << ", restarting EHC..." << endl;
                    insert_successor_into_open_list(current_eval_context, 0, OperatorID::no_operator, false);
                    return IN_PROGRESS;
                }
            }

            // Move to the next depth level
            ++depth;
            log << "Moving to depth " << depth << " with " << beam.size() << " states in the beam." << endl;

            // restart the depth counter if we reach the max depth
            if (depth == max_depth) {
                log << "Max depth reached, restarting depth counter..." << endl;
                return IN_PROGRESS;
            }
        }

        log << "Max depth reached without finding a better state, beam search failed." << endl;
        return FAILED;  // If max depth is reached without improvement, return failure
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
