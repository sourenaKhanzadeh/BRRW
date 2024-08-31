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

#include <random>
#include <chrono>

using namespace std;
using utils::ExitCode;

namespace enforced_hill_climbing_beam_rrw_search {
    using GEval = g_evaluator::GEvaluator;
    using PrefEval = pref_evaluator::PrefEvaluator;

    static shared_ptr <OpenListFactory> create_ehc_open_list_factory(
            utils::Verbosity verbosity, bool use_preferred,
            PreferredUsage preferred_usage) {
        shared_ptr<Evaluator> g_evaluator = make_shared<GEval>("ehc.g_eval", verbosity);

        if (!use_preferred || preferred_usage == PreferredUsage::PRUNE_BY_PREFERRED) {
            return make_shared<standard_scalar_open_list::BestFirstOpenListFactory>(g_evaluator, false);
        } else {
            vector<shared_ptr<Evaluator>> evals = {
                    g_evaluator, make_shared<PrefEval>("ehc.pref_eval", verbosity)};
            return make_shared<tiebreaking_open_list::TieBreakingOpenListFactory>(evals, false, true);
        }
    }

    RestartStrategyType parse_restart_strategy(const std::string &strategy) {
        if (strategy == "luby") {
            return RestartStrategyType::LUBY;
        } else {
            return RestartStrategyType::NONE;
        }
    }

    EnforcedHillClimbingBRRWSearch::EnforcedHillClimbingBRRWSearch(
            const shared_ptr<Evaluator> &h, PreferredUsage preferred_usage,
            const vector<shared_ptr<Evaluator>> &preferred,
            OperatorCost cost_type, int bound, double max_time,
            int beam_width, int max_depth, const string &restart_strategy,
            const string &description, utils::Verbosity verbosity)
            : SearchAlgorithm(cost_type, bound, max_time, description, verbosity),
              evaluator(h),
              preferred_operator_evaluators(preferred),
              preferred_usage(preferred_usage),
              current_eval_context(state_registry.get_initial_state(), &statistics),
              current_phase_start_g(-1),
              num_ehc_phases(0),
              last_num_expanded(-1),
              beam_width(beam_width),
              max_depth(max_depth),
              restart_strategy(restart_strategy) {

        assert(beam_width >= 1);
        assert(max_depth >= 1);

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

        // Initialize the restart strategy based on enum
        if (restart_strategy == "luby") {
            r_strategy = std::make_unique<LubyRestartStrategy>(1);
        } else {
            r_strategy = nullptr;  // Default to no restart strategy (fixed timestep)
        }
    }




    void EnforcedHillClimbingBRRWSearch::reach_state(
            const State &parent, OperatorID op_id, const State &state) {
        for (Evaluator *evaluator: path_dependent_evaluators) {
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
        EvaluationContext new_eval_context(eval_context, succ_g, preferred, &statistics);
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
        std::mt19937 rng(std::chrono::system_clock::now().time_since_epoch().count());
        while (!open_list->empty()) {
            EdgeOpenListEntry entry = open_list->remove_min();
            StateID parent_state_id = entry.first;
            OperatorID last_op_id = entry.second;
            OperatorProxy last_op = task_proxy.get_operators()[last_op_id];

            State parent_state = state_registry.lookup_state(parent_state_id);
            SearchNode parent_node = search_space.get_node(parent_state);

            // d: distance from initial node in this EHC phase
            int d = parent_node.get_g() - current_phase_start_g +
                    get_adjusted_cost(last_op);

            if (parent_node.get_real_g() + last_op.get_cost() >= bound)
                continue;

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
                node.open_new_node(parent_node, last_op,
                                   get_adjusted_cost(last_op));

                if (h < current_eval_context.get_evaluator_value(evaluator.get())) {
                    ++num_ehc_phases;
                    if (d_counts.count(d) == 0) {
                        d_counts[d] = make_pair(0, 0);
                    }
                    pair<int, int> &d_pair = d_counts[d];
                    d_pair.first += 1;
                    d_pair.second += statistics.get_expanded() - last_num_expanded;

                    current_eval_context = move(eval_context);
                    open_list->clear();
                    current_phase_start_g = node.get_g();
                    return IN_PROGRESS;
                } else {
//                    expand(eval_context);

//                    if(beam_width == 1) {
//                        std::mt19937 rng(std::chrono::system_clock::now().time_since_epoch().count());
//                        return random_restart_walk();
//                    } else {
//                        std::mt19937 rng(std::chrono::system_clock::now().time_since_epoch().count());
//                        return beam_search(rng);
//                    }
                }
            }
        }


        // open list is empty..perform RWs or return no solution after enough trials

        int current_hvalue = current_eval_context.get_evaluator_value(evaluator.get());
        EvaluationContext eval_context = current_eval_context;
        int hvalue = current_hvalue;
        cout << "Entering RWs(" << beam_width << "): to beat: " << hvalue << endl;
        //int num_restarts = 1;
        if(r_strategy){
            r_strategy->reset_sequence();
        }

        uint64_t MAX_TIMESTEP = 1;
        MAX_TIMESTEP <<= 63;
        if (beam_width == 1){

            // start a timer
            auto start = std::chrono::high_resolution_clock::now();

            do {
                // check time exceeds max_time
                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = end - start;
                if (elapsed.count() > max_time) {
                    cout << "Time limit exceeded" << endl;
                    return FAILED;
                }
                uint64_t restart_length;
                if(r_strategy){
                    restart_length = r_strategy->next_sequence_value();
                }else{
                    restart_length = max_depth;
                }
                if (restart_length < (MAX_TIMESTEP >> 1)) {
                    restart_length = restart_length << 1;  // scale by 2 because we know depth 1 successors are no good
                }
                uint64_t timestep = 0;
                eval_context = current_eval_context;

                while (hvalue >= current_hvalue && timestep < restart_length) {
                    vector<OperatorID> ops;
                    successor_generator.generate_applicable_ops(eval_context.get_state(), ops);

                    if (ops.size() == 0) {
                        cout << "Pruned all operators -- doing a pseudo-restart" << endl;
                        eval_context = current_eval_context;
                    } else {
                        std::uniform_int_distribution<int> dist(0, ops.size() - 1);
                        OperatorID random_op_id = ops[dist(rng)];
                        const OperatorProxy random_op = task_proxy.get_operators()[random_op_id];
                        State state = state_registry.get_successor_state(eval_context.get_state(), random_op);

                        // Ensure the state is properly linked to its parent
                        reach_state(eval_context.get_state(), random_op_id, state);

                        // Get the search node for the new state and ensure proper initialization
                        SearchNode successor_node = search_space.get_node(state);
                        SearchNode parent_node = search_space.get_node(eval_context.get_state());
                        if (successor_node.is_new()) {
                            successor_node.open_new_node(parent_node, random_op, parent_node.get_g() + get_adjusted_cost(random_op));
                        }

                        eval_context = EvaluationContext(state, &statistics);  // Eval Context of successor
                        statistics.inc_evaluated_states();  // Evaluating random state
                        statistics.inc_expanded();  // Expanding current state
                        statistics.inc_generated();  // Only generating one (random) state

                        hvalue = eval_context.get_evaluator_value(evaluator.get());
                    }
                    ++timestep;
                }
                // cout << eval_context.get_state().get_id() << "(" << hvalue << ")" << endl << "---" << endl;
            } while (hvalue >= current_hvalue);

        }else {
            // beam rrw
            auto start = std::chrono::high_resolution_clock::now();
            do {
                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = end - start;
                if (elapsed.count() > max_time) {
                    cout << "Time limit exceeded" << endl;
                    return FAILED;
                }
                uint64_t restart_length;
                if(r_strategy){
                    restart_length = r_strategy->next_sequence_value();
                }else{
                    restart_length = max_depth;
                }
                if (restart_length < (MAX_TIMESTEP >> 1)) {
                    restart_length = restart_length << 1;  // scale by 2 because we know depth 1 successors are no good
                }
                uint64_t timestep = 0;

                while (hvalue >= current_hvalue && timestep < restart_length) {
                    vector<OperatorID> ops;
                    successor_generator.generate_applicable_ops(eval_context.get_state(), ops);

                    if (ops.size() == 0) {
                        cout << "Pruned all operators -- doing a pseudo-restart" << endl;
                        eval_context = current_eval_context;
                    } else {
                        vector<StateID> unique_states;
                        vector<State> beam;

                        // Generate unique state successors
                        for (int i = 0; i < beam_width && ops.size() > 0; ++i) {
                            std::uniform_int_distribution<int> dist(0, ops.size() - 1);
                            OperatorID random_op_id = ops[dist(rng)];
                            const OperatorProxy random_op = task_proxy.get_operators()[random_op_id];
                            State state = state_registry.get_successor_state(eval_context.get_state(), random_op);
                            statistics.inc_generated();

                            // Check for uniqueness
                            if (std::find(unique_states.begin(), unique_states.end(), state.get_id()) == unique_states.end()) {
                                unique_states.push_back(state.get_id());
                                beam.push_back(state);

                                // Record the parent state and operator for this state
                                reach_state(eval_context.get_state(), random_op_id, state);

                                // Also, ensure that the state correctly tracks its parent node
                                SearchNode successor_node = search_space.get_node(state);
                                SearchNode parent_node = search_space.get_node(eval_context.get_state());
                                if (successor_node.is_new()) {
                                    successor_node.open_new_node(parent_node, random_op, parent_node.get_g() + get_adjusted_cost(random_op));
                                }
                            } else {
                                ops.erase(ops.begin() + dist(rng));  // Remove duplicate state operator
                                --i;  // Ensure correct number of beams
                            }
                        }

                        if (beam.empty()) {
                            eval_context = current_eval_context;  // No unique beams generated, restart needed
                        } else {
                            // Evaluate successors and find the best one
                            vector<int> hvalues_vec;
                            for (const State &state : beam) {
                                EvaluationContext new_eval_context(state, &statistics);
                                statistics.inc_evaluated_states();
                                statistics.inc_expanded();
                                int beam_hvalue = new_eval_context.get_evaluator_value(evaluator.get());
                                hvalues_vec.push_back(beam_hvalue);
                            }

                            int best_index = distance(hvalues_vec.begin(), min_element(hvalues_vec.begin(), hvalues_vec.end()));
                            eval_context = EvaluationContext(beam[best_index], &statistics);
                            hvalue = hvalues_vec[best_index];

                            if (hvalue < current_hvalue) {
                                current_eval_context = eval_context;
                                insert_successor_into_open_list(eval_context, 0, OperatorID::no_operator, false);
                                return IN_PROGRESS;
                            }
                        }
                    }
                    ++timestep;  // Move this inside the loop
                }
            } while (hvalue >= current_hvalue);

        }




        //for (std::vector<const GlobalState*>::iterator it = walk.begin(); it != walk.end(); ++it)
        //GlobalState parent_state = current_eval_context.get_state();
        //for (unsigned int i = 0; i < walk.size(); ++i)
        //for (GlobalState state : walk)
        //{
        //GlobalState state = walk.at(i);
        //cout << state.get_id() << "-->";
        //const GlobalOperator* random_op = actions.at(i);
        //const GobalState* state = *it;
        //SearchNode search_node = search_space.get_node(state);
        //search_node.update_parent(search_space.get_node(parent_state), random_op);
        //parent_state = state;
        //}

//        used_actions[current_eval_context.get_state().get_id()] = actions;
//        next_state[current_eval_context.get_state().get_id()] = eval_context.get_state().get_id();
        current_eval_context = eval_context;
        return IN_PROGRESS;
    }


    SearchStatus EnforcedHillClimbingBRRWSearch::random_restart_walk() {
        std::mt19937 rng(std::chrono::system_clock::now().time_since_epoch().count());
        log << "Starting random restart walk..." << endl;

        int depth = 0;
        State current_state = current_eval_context.get_state();
        SearchNode current_node = search_space.get_node(current_state);

        while (depth < max_depth) {
            std::vector<OperatorID> applicable_ops;
            successor_generator.generate_applicable_ops(current_state, applicable_ops);

            if (applicable_ops.empty()) {
                return IN_PROGRESS; // Dead end, restart needed
            }

            // Randomly select a successor
            std::uniform_int_distribution<int> dist(0, applicable_ops.size() - 1);
            OperatorID selected_op = applicable_ops[dist(rng)];
            OperatorProxy op = task_proxy.get_operators()[selected_op];
            State successor_state = state_registry.get_successor_state(current_state, op);
            statistics.inc_generated();

            SearchNode successor_node = search_space.get_node(successor_state);

            if (successor_node.is_new()) {
                EvaluationContext eval_context(successor_state, &statistics);
                reach_state(current_state, selected_op, successor_state);  // Ensure proper state linkage
                statistics.inc_evaluated_states();

                if (!eval_context.is_evaluator_value_infinite(evaluator.get())) {
                    int h = eval_context.get_evaluator_value(evaluator.get());

                    if (h < current_eval_context.get_evaluator_value(evaluator.get())) {
                        successor_node.open_new_node(current_node, op, current_node.get_g() + get_adjusted_cost(op));
                        open_list->insert(eval_context, std::make_pair(successor_state.get_id(), selected_op));

                        if (check_goal_and_set_plan(successor_state)) {
                            log << "Goal found during random restart walk: " << successor_state.get_id() << endl;
                            return SOLVED;
                        }
                        log << "Found new node with better evaluation, restarting EHC..." << endl;
                        return IN_PROGRESS;
                    }
                }
            }

            current_state = successor_state;
            current_node = successor_node;
            ++depth;
        }

        return IN_PROGRESS; // Depth limit reached, need to restart
    }






















    SearchStatus EnforcedHillClimbingBRRWSearch::beam_search(std::mt19937 &rng) {
        int current_hvalue = current_eval_context.get_evaluator_value(evaluator.get());
        EvaluationContext eval_context = current_eval_context;
        log << "Starting Beam Search with initial heuristic value: " << current_hvalue << endl;

        vector<State> beam;
        beam.push_back(current_eval_context.get_state());

        int depth = 0;
        uint64_t restart_length;

        // Initialize the sequence count if necessary
        if (r_strategy) {
            r_strategy->reset_sequence();
        }

        while (true) {  // Loop indefinitely and use restart_length to control depth
            vector<pair<int, State>> successors;

            for (const State &current_state: beam) {
                eval_context = EvaluationContext(current_state, &statistics);

                vector<OperatorID> ops;
                successor_generator.generate_applicable_ops(eval_context.get_state(), ops);

                if (ops.empty()) {
                    log << "All operators pruned for current state, continuing to next state in beam..." << endl;
                    continue;
                }

                for (OperatorID op_id: ops) {
                    const OperatorProxy op = task_proxy.get_operators()[op_id];
                    State next_state = state_registry.get_successor_state(eval_context.get_state(), op);

                    EvaluationContext successor_eval_context(next_state, &statistics);
                    statistics.inc_evaluated_states();
                    statistics.inc_expanded();
                    statistics.inc_generated();

                    int state_hvalue = successor_eval_context.get_evaluator_value(evaluator.get());
                    successors.push_back(make_pair(state_hvalue, next_state));
                }
            }

            if (successors.empty()) {
                log << "No successors found, beam search failed." << endl;
                return FAILED;
            }

            sort(successors.begin(), successors.end(), [](const pair<int, State> &a, const pair<int, State> &b) {
                return a.first < b.first;
            });

            beam.clear();
            for (int i = 0; i < min(beam_width, static_cast<int>(successors.size())); ++i) {
                beam.push_back(successors[i].second);

                if (successors[i].first < current_hvalue) {
                    current_eval_context = EvaluationContext(successors[i].second, &statistics);
                    current_hvalue = successors[i].first;
                    log << "Found better state during Beam Search, restarting EHC..." << endl;
                    insert_successor_into_open_list(current_eval_context, 0, OperatorID::no_operator, false);
                    return IN_PROGRESS;
                }
            }

            ++depth;
            log << "Moving to depth " << depth << " with " << beam.size() << " states in the beam." << endl;

            // Get the next sequence value from the Luby restart strategy or use the maximum depth directly
            if (r_strategy) {
                restart_length = r_strategy->next_sequence_value();
            } else {
                restart_length = max_depth;  // Use max_depth if no strategy is set
            }

            if (depth >= restart_length) {
                log << "Restart length reached (" << restart_length << "), resetting depth counter and beam..." << endl;
                depth = 0;  // Reset depth
                beam.clear();
                beam.push_back(current_eval_context.get_state());  // Restart from the current state
                return IN_PROGRESS;
            }
        }

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

        for (auto count: d_counts) {
            int depth = count.first;
            int phases = count.second.first;
            assert(phases != 0);
            int total_expansions = count.second.second;
            log << "EHC phases of depth " << depth << ": " << phases
                << " - Avg. Expansions: "
                << static_cast<double>(total_expansions) / phases << endl;
        }
    }

    RestartStrategy::RestartStrategy()
            : internal_sequence_count(1L) {}

    RestartStrategy::RestartStrategy(long sequence_start_value)
            : internal_sequence_count(sequence_start_value)
    {
        // Assert sequence_start_value > 0
    }

    RestartStrategy::~RestartStrategy()
    {}

    void RestartStrategy::reset_sequence()
    {
        internal_sequence_count = 1L;
    }

    LubyRestartStrategy::LubyRestartStrategy()
            : RestartStrategy()
    {}

    LubyRestartStrategy::LubyRestartStrategy(long sequence_start_value)
            : RestartStrategy(sequence_start_value)
    {}

    LubyRestartStrategy::~LubyRestartStrategy()
    {}

    uint64_t LubyRestartStrategy::next_sequence_value()
    {
        return sequence(internal_sequence_count++);
    }


    uint64_t LubyRestartStrategy::sequence(long sequence_number)
    {
        long focus = 2L;
        while (sequence_number > (focus - 1)) {
            focus = focus << 1;
        }

        if (sequence_number == (focus - 1)) {
            return focus >> 1;
        }
        else {
            return sequence(sequence_number - (focus >> 1) + 1);
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
                    "100");

            add_option<std::string>(
                    "restart_strategy",
                    "Restart strategy to use during random walks (options: 'luby', 'none').",
                    "\"none\"");  // Default to no strategy (fixed timestep)

            add_search_algorithm_options_to_feature(*this, "ehcbrrw");
        }

        virtual shared_ptr<EnforcedHillClimbingBRRWSearch> create_component(
                const plugins::Options &opts,
                const utils::Context &) const override {
            RestartStrategyType restart_strategy = parse_restart_strategy(
                    opts.get<std::string>("restart_strategy")
            );

            return make_shared<EnforcedHillClimbingBRRWSearch>(
                    opts.get<shared_ptr<Evaluator>>("h"),
                    opts.get<PreferredUsage>("preferred_usage"),
                    opts.get_list<shared_ptr<Evaluator>>("preferred"),
                    opts.get<OperatorCost>("cost_type"),
                    opts.get<int>("bound"),
                    opts.get<double>("max_time"),
                    opts.get<int>("beam_width"),
                    opts.get<int>("max_depth"),
                    opts.get<std::string>("restart_strategy"),  // Use the enum here
                    opts.get<std::string>("description"),
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