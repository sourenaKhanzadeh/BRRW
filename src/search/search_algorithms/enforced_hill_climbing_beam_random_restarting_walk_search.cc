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
#include "../utils/rng.h"
#include "../utils/rng_options.h"

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
            int random_seed,
            const shared_ptr<Evaluator> &h, PreferredUsage preferred_usage,
            const vector<shared_ptr<Evaluator>> &preferred,
            OperatorCost cost_type, int bound, double max_time,
            int beam_width, int max_depth, const string &restart_strategy, int luby_start_value,
            bool cluster,
            const string &description, utils::Verbosity verbosity)
            : SearchAlgorithm(cost_type, bound, max_time, description, verbosity),
              rng(utils::get_rng(random_seed)),
              evaluator(h),
              preferred_operator_evaluators(preferred),
              preferred_usage(preferred_usage),
              improvement_found(true),
              current_eval_context(state_registry.get_initial_state(), &statistics), // can this be done in initialize(), or the body?  state_registry.get_initial_state() is called twice  [dawson]
              current_phase_start_g(-1),
              num_ehc_phases(0),
              last_num_expanded(-1),
              beam_width(beam_width),
              max_depth(max_depth),
              restart_strategy(restart_strategy),
              luby_start_value(luby_start_value),
              cluster(cluster) {

        assert(beam_width >= 1);
        assert(max_depth >= 3);

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
                verbosity, use_preferred, preferred_usage)->create_state_open_list();

        // Initialize the restart strategy based on enum
        if (restart_strategy == "luby") {
            r_strategy = std::make_unique<LubyRestartStrategy>(luby_start_value);
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

    bool EnforcedHillClimbingBRRWSearch::insert_successor_into_open_list(
            EvaluationContext &eval_context,
            int parent_g,
            OperatorID op_id,
            bool preferred) {
        OperatorProxy op = task_proxy.get_operators()[op_id];
        int succ_g = parent_g + get_adjusted_cost(op);
        const State &state = eval_context.get_state();
        SearchNode node = search_space.get_node(state);

        State succ_state = state_registry.get_successor_state(state, op);
        statistics.inc_generated();
        bool is_preferred = preferred;
        SearchNode succ_node = search_space.get_node(succ_state);
        EvaluationContext succ_eval_context(
            succ_state, succ_g, is_preferred, &statistics);
        statistics.inc_evaluated_states();
        succ_node.open_new_node(node, op, get_adjusted_cost(op));
        open_list->insert(succ_eval_context, succ_state.get_id());

        return succ_eval_context.get_evaluator_value_or_infinity(evaluator.get())
                < eval_context.get_evaluator_value_or_infinity(evaluator.get());
    }

    bool EnforcedHillClimbingBRRWSearch::expand(EvaluationContext &eval_context) {
        SearchNode node = search_space.get_node(eval_context.get_state());
        int node_g = node.get_g();
        bool improvement_found = false;

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
                improvement_found = insert_successor_into_open_list(
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
                improvement_found = insert_successor_into_open_list(
                        eval_context, node_g, op_id, preferred);
            }
        }

        statistics.inc_expanded();
        node.close();
        return improvement_found;
    }

    void EnforcedHillClimbingBRRWSearch::search() {
        initialize();
        SearchStatus status = IN_PROGRESS;
        utils::CountdownTimer timer(max_time);
        while (status == IN_PROGRESS) {
            status = step();
            if (timer.is_expired()) {
                log << "Time limit reached. Abort search." << endl;
                status = TIMEOUT;
                break;
            }
        }
        // TODO: Revise when and which search times are logged.
        log << "Actual search time: " << timer.get_elapsed_time() << endl;
    }



    SearchStatus EnforcedHillClimbingBRRWSearch::step() {
        last_num_expanded = statistics.get_expanded();         // see line 324
        search_progress.check_progress(current_eval_context);
        if (check_goal_and_set_plan(current_eval_context.get_state())) {
            return SOLVED;
        }

        if (improvement_found) {
            bool expansion_succeeeded = expand(current_eval_context);
            if (expansion_succeeeded) {
                StateID next_id = open_list->remove_min();

                // making a new eval_context here is kind of stupid
                EvaluationContext eval_context(state_registry.lookup_state(next_id), &statistics);
                current_eval_context = move(eval_context);
                open_list->clear();

                ++num_ehc_phases;  
                // if (d_counts.count(d) == 0) {
                //     d_counts[d] = make_pair(0, 0);
                // }
                // pair<int, int> &d_pair = d_counts[d];
                // d_pair.first += 1;
                // d_pair.second += statistics.get_expanded() - last_num_expanded;

                const State &state = eval_context.get_state();
                SearchNode node = search_space.get_node(state);
                current_phase_start_g = node.get_g();
            } else {
                improvement_found = false;
            }
            return IN_PROGRESS;
        } else {
            return rrw();
        }
    }

    SearchStatus EnforcedHillClimbingBRRWSearch::rrw() { // why the wrapper?   [dawson]
        int current_hvalue = current_eval_context.get_evaluator_value(evaluator.get());
        int hvalue = current_hvalue;
        std::cout << "Entering RWs(" << beam_width << "): to beat: " << hvalue << endl;
        //int num_restarts = 1;
        if(r_strategy){
            r_strategy->reset_sequence();
        }

        if (beam_width == 1){
            do {
                uint64_t restart_length;
                if(r_strategy){
                    restart_length = r_strategy->next_sequence_value();
                }else{
                    restart_length = max_depth;
                }
                uint64_t timestep = 0; // 1. timestep is 0 [dawson] [infinite loop]
                EvaluationContext eval_context = current_eval_context;

                while (timestep < restart_length) { //2. timestep must be less than restart_length to exit [dawson] [infinite loop]
                    vector<OperatorID> ops;
                    successor_generator.generate_applicable_ops(eval_context.get_state(), ops);

                    if (ops.size() == 0) { 
                        std::cout << "Pruned all operators -- doing a pseudo-restart" << endl; // this gets caught in an infinite loop [dawson] [infinite loop]
                        eval_context = current_eval_context; // 3. if current_eval_context gives no ops eval_context will continue being current_eval_context [dawson] [infinite loop]
                    } else {
                        OperatorID random_op_id = *rng->choose(ops);
                        const OperatorProxy random_op = task_proxy.get_operators()[random_op_id];
                        State state = state_registry.get_successor_state(eval_context.get_state(), random_op);

                        // Ensure the state is properly linked to its parent
                        reach_state(eval_context.get_state(), random_op_id, state);

                        // Get the search node for the new state and ensure proper initialization
                        SearchNode successor_node = search_space.get_node(state);
                        SearchNode parent_node = search_space.get_node(eval_context.get_state());
                        if (successor_node.is_new()) { // do we want to open every new node? or just the path to the improving state?
                            successor_node.open_new_node(parent_node, random_op, parent_node.get_g() + get_adjusted_cost(random_op));
                        }

                        EvaluationContext eval_context(state, &statistics);  // Eval Context of successor
                        statistics.inc_evaluated_states();  // Evaluating random state
                        statistics.inc_generated();  // Only generating one (random) state

                        hvalue = eval_context.get_evaluator_value(evaluator.get());  // 4. if (ops.size() == 0) is, hvalue never gets updated so step 5 never exits [dawson] [infinite loop]
                        if (hvalue < current_hvalue)
                            break;
                    }
                    ++timestep;
                }
                // cout << eval_context.get_state().get_id() << "(" << hvalue << ")" << endl << "---" << endl;
            } while (hvalue >= current_hvalue); // 5. hvalue must be less than current_hvalue to exit [dawson] [infinite loop]

        } else {
            // beam rrw
            State initial_beam_state = current_eval_context.get_state();
            vector<State> beam;
            beam.push_back(current_eval_context.get_state());

            do {
                uint64_t restart_length;
                
                // Determine the restart length
                if (r_strategy) {
                    restart_length = r_strategy->next_sequence_value();
                } else {
                    restart_length = max_depth;
                }

                uint64_t timestep = 0;

                while (timestep < restart_length) {
                    vector<pair<int, State>> evaluated_states;
                    vector<pair<State, State>> cluster_st;

                    // Explore each beam node
                    for (const State &current_state : beam) {
                        vector<OperatorID> ops;
                        successor_generator.generate_applicable_ops(current_state, ops);

                        if (ops.empty()) {
                            log << "[Dead End] No applicable operations for state ID: " << current_state.get_id() << ". Continuing..." << endl;
                            continue;
                        }

                        if (cluster) {
                            // Cluster mode: Select random successors from each beam node
                            for (int i = 0; i < beam_width && !ops.empty(); ++i) {
                                std::uniform_int_distribution<int> dist(0, ops.size() - 1);
                                OperatorID random_op_id = ops[dist(rng)];
                                const OperatorProxy random_op = task_proxy.get_operators()[random_op_id];
                                State next_state = state_registry.get_successor_state(current_state, random_op);

                                reach_state(current_state, random_op_id, next_state);

                                SearchNode successor_node = search_space.get_node(next_state);
                                SearchNode parent_node = search_space.get_node(current_state);
                                if (successor_node.is_new()) {
                                    successor_node.open_new_node(parent_node, random_op, parent_node.get_g() + get_adjusted_cost(random_op));
                                }

                                EvaluationContext new_eval_context(next_state, &statistics);
                                statistics.inc_evaluated_states();
                                statistics.inc_expanded(); // see line 324 [dawson]
                                statistics.inc_generated();

                                int h_value = new_eval_context.get_evaluator_value(evaluator.get());
                                evaluated_states.push_back(make_pair(h_value, std::move(next_state)));
                                cluster_st.push_back(make_pair(current_state, next_state));

                                ops.erase(ops.begin() + dist(rng));
                            }
                        } else {
                            // Non-cluster mode: Expand all successors
                            shuffle(ops.begin(), ops.end(), rng);
                            for (OperatorID op_id : ops) {
                                const OperatorProxy op = task_proxy.get_operators()[op_id];
                                State next_state = state_registry.get_successor_state(current_state, op);

                                reach_state(current_state, op_id, next_state);

                                SearchNode successor_node = search_space.get_node(next_state);
                                SearchNode parent_node = search_space.get_node(current_state);
                                if (successor_node.is_new()) {
                                    successor_node.open_new_node(parent_node, op, parent_node.get_g() + get_adjusted_cost(op));
                                }

                                EvaluationContext new_eval_context(next_state, &statistics);
                                statistics.inc_evaluated_states();
                                statistics.inc_expanded();  // see line 324 [dawson]
                                statistics.inc_generated();

                                int h_value = new_eval_context.get_evaluator_value(evaluator.get());
                                evaluated_states.push_back(make_pair(h_value, std::move(next_state)));
                            }
                        }
                    }
                    // If no valid successors are found, restart
                    if (evaluated_states.empty()) {
                        log << "[Restart] No valid successors found. Restarting EHC from the current context." << endl;
                        eval_context = current_eval_context;
                        break;
                    }

                    if (cluster) {
                        beam.clear();
                        unordered_set<StateID> seen_states;  // To track added states by their ID for fast lookup
                        seen_states.reserve(beam_width);

                        // Add one state from each cluster until beam_width is reached
                        size_t cluster_idx = 0;

                        // Add unique states from clusters
                        while (beam.size() < beam_width && cluster_idx < cluster_st.size()) {
                            State next_state = cluster_st[cluster_idx].second;
                            StateID next_state_id = next_state.get_id();
                            
                            // Add state if it hasn't been added before
                            if (seen_states.insert(next_state_id).second) {
                                beam.push_back(std::move(next_state));
                            }
                            cluster_idx++;
                        }

                        // If beam is not full, add random unique states from evaluated_states
                        std::uniform_int_distribution<int> dist(0, evaluated_states.size() - 1);
                        while (beam.size() < beam_width && cluster_idx < cluster_st.size()) {
                            State next_state = evaluated_states[dist(rng)].second;
                            StateID next_state_id = next_state.get_id();
                            
                            // Only add if it's a unique state
                            if (seen_states.insert(next_state_id).second) {
                                beam.push_back(std::move(next_state));
                            }

                            cluster_idx++;
                        }

                    } else {
                        // Non-cluster: Clear and update the beam with best states
                        beam.clear();
                        // std::sort(evaluated_states.begin(), evaluated_states.end(),
                        //           [](const pair<int, State> &a, const pair<int, State> &b) {
                        //               return a.first < b.first;
                        //           });
                        for (int i = 0; i < beam_width && i < evaluated_states.size(); ++i) {
                            beam.push_back(std::move(evaluated_states[i].second));
                        }
                    }

                    // Check if we found a better state
                    if (!evaluated_states.empty()) {
                        std::sort(evaluated_states.begin(), evaluated_states.end(),
                                [](const pair<int, State> &a, const pair<int, State> &b) {
                                    return a.first < b.first;
                                });
                        int best_hvalue = evaluated_states.front().first;
                        if (best_hvalue < current_hvalue) {
                            log << "[Improvement] Found a better state with heuristic value: " << best_hvalue << endl;
                            current_eval_context = EvaluationContext(evaluated_states.front().second, &statistics);
                            current_hvalue = best_hvalue;
                            open_list->clear();
                            return IN_PROGRESS;
                        }
                    }

                    current_eval_context = EvaluationContext(evaluated_states.front().second, &statistics);
                    timestep++;
                }

                // Reset the beam to initial state after a restart
                beam.clear();
                beam.push_back(initial_beam_state);

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

    }


    SearchStatus EnforcedHillClimbingBRRWSearch::beam_search(std::mt19937 &rng) {

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

        // for (auto count: d_counts) {
        //     int depth = count.first;
        //     int phases = count.second.first;
        //     assert(phases != 0);
        //     int total_expansions = count.second.second;
        //     log << "EHC phases of depth " << depth << ": " << phases
        //         << " - Avg. Expansions: "
        //         << static_cast<double>(total_expansions) / phases << endl;
        // }
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

            add_option<int>(
                    "luby_start_value",
                    "Initial value for the Luby sequence.",
                    "1");

            add_option<bool>(
                    "cluster",
                    "Cluster mode for beam search.",
                    "false");
            utils::add_rng_options_to_feature(*this);
            add_search_algorithm_options_to_feature(*this, "ehcbrrw");
        }

        virtual shared_ptr<EnforcedHillClimbingBRRWSearch> create_component(
                const plugins::Options &opts,
                const utils::Context &) const override {
            RestartStrategyType restart_strategy = parse_restart_strategy(
                    opts.get<std::string>("restart_strategy")
            );

            return make_shared<EnforcedHillClimbingBRRWSearch>(
                    utils::get_rng_arguments_from_options(opts),
                    opts.get<shared_ptr<Evaluator>>("h"),
                    opts.get<PreferredUsage>("preferred_usage"),
                    opts.get_list<shared_ptr<Evaluator>>("preferred"),
                    opts.get<OperatorCost>("cost_type"),
                    opts.get<int>("bound"),
                    opts.get<double>("max_time"),
                    opts.get<int>("beam_width"),
                    opts.get<int>("max_depth"),
                    opts.get<std::string>("restart_strategy"),  // Use the enum here
                    opts.get<int>("luby_start_value"),
                    opts.get<bool>("cluster"),
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