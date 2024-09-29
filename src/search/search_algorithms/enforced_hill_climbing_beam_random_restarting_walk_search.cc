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
            int beam_width, int max_depth, const string &restart_strategy, int luby_start_value,
            bool cluster,
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
                verbosity, use_preferred, preferred_usage)->create_edge_open_list();

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
        
        // Single EHC phase (one step look-ahead)
        while (!open_list->empty()) {
            EdgeOpenListEntry entry = open_list->remove_min();
            StateID parent_state_id = entry.first;
            OperatorID last_op_id = entry.second;
            OperatorProxy last_op = task_proxy.get_operators()[last_op_id];

            State parent_state = state_registry.lookup_state(parent_state_id);
            SearchNode parent_node = search_space.get_node(parent_state);

            int d = parent_node.get_g() - current_phase_start_g + get_adjusted_cost(last_op);

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
                node.open_new_node(parent_node, last_op, get_adjusted_cost(last_op));

                if (h < current_eval_context.get_evaluator_value(evaluator.get())) {
                    ++num_ehc_phases;
                    update_d_counts(d);
                    current_eval_context = std::move(eval_context);
                    open_list->clear();
                    current_phase_start_g = node.get_g();
                    return IN_PROGRESS;
                }
            }
        }

        // Open list is empty, perform RWs or return no solution after enough trials
        int current_hvalue = current_eval_context.get_evaluator_value(evaluator.get());
        EvaluationContext eval_context = current_eval_context;
        int hvalue = current_hvalue;
        std::cout << "Entering RWs(" << beam_width << "): to beat: " << hvalue << std::endl;

        if (r_strategy) {
            r_strategy->reset_sequence();
        }

        return beam_width == 1 ? perform_single_walk(rng, current_hvalue) : perform_beam_walk(rng, current_hvalue);
    }

    SearchStatus EnforcedHillClimbingBRRWSearch::perform_single_walk(std::mt19937& rng, int current_hvalue) {
        EvaluationContext best_eval_context = current_eval_context;
        int best_hvalue = current_hvalue;

        while (best_hvalue >= current_hvalue) {
            uint64_t restart_length = r_strategy ? r_strategy->next_sequence_value() : max_depth;
            uint64_t timestep = 0;
            EvaluationContext eval_context = current_eval_context;
            int hvalue = current_hvalue;

            while (timestep < restart_length) {
                vector<OperatorID> ops;
                successor_generator.generate_applicable_ops(eval_context.get_state(), ops);

                if (ops.empty()) {
                    std::cout << "Pruned all operators -- doing a pseudo-restart" << std::endl;
                    eval_context = current_eval_context;
                    hvalue = current_hvalue;
                } else { 
                    std::uniform_int_distribution<int> dist(0, ops.size() - 1);
                    OperatorID random_op_id = ops[dist(rng)];
                    const OperatorProxy random_op = task_proxy.get_operators()[random_op_id];
                    State state = state_registry.get_successor_state(eval_context.get_state(), random_op);

                    reach_state(eval_context.get_state(), random_op_id, state);

                    SearchNode successor_node = search_space.get_node(state);
                    SearchNode parent_node = search_space.get_node(eval_context.get_state());
                    if (successor_node.is_new()) {
                        successor_node.open_new_node(parent_node, random_op, parent_node.get_g() + get_adjusted_cost(random_op));
                    }

                    eval_context = EvaluationContext(state, &statistics);
                    update_statistics();

                    hvalue = eval_context.get_evaluator_value(evaluator.get());

                    if (hvalue < best_hvalue) {
                        current_eval_context = eval_context;
                        best_hvalue = hvalue;
                        return IN_PROGRESS;
                    }
                }
                ++timestep;
            }
        }

        current_eval_context = best_eval_context;
        return IN_PROGRESS;
    }

    SearchStatus EnforcedHillClimbingBRRWSearch::perform_beam_walk(std::mt19937& rng, int current_hvalue) {
        EvaluationContext eval_context = current_eval_context;
        while (true) {
            State initial_beam_state = eval_context.get_state();
            vector<State> active_beam;
            vector<State> active_frontier;
            active_beam.push_back(initial_beam_state);

            bool improvement_found = false;
            int best_frontier_hvalue = current_hvalue;
            int best_i;

            uint64_t restart_length;
            
                // Determine the restart length
            if (r_strategy) {
                restart_length = r_strategy->next_sequence_value();
            } else {
                restart_length = max_depth;
            }

            uint64_t timestep = 0;

            while (timestep < restart_length) {
                // vector<pair<int, State>> evaluated_states;
                // vector<pair<State, State>> cluster_st;

                // Explore each beam node
                for (const State &current_state : active_beam) {
                    vector<OperatorID> ops;
                    successor_generator.generate_applicable_ops(current_state, ops);

                    if (ops.empty()) {
                        log << "[Dead End] No applicable operations for state ID: " << current_state.get_id() << ". Continuing..." << endl;
                        continue;
                    }

                    // Non-cluster mode: Expand all successors
                    // shuffle(ops.begin(), ops.end(), rng);
                    for (OperatorID op_id : ops) {
                        const OperatorProxy op = task_proxy.get_operators()[op_id];
                        State next_state = state_registry.get_successor_state(current_state, op);

                        reach_state(current_state, op_id, next_state);

                        SearchNode successor_node = search_space.get_node(next_state);
                        SearchNode parent_node = search_space.get_node(current_state);
                        if (successor_node.is_new()) {
                            successor_node.open_new_node(parent_node, op, parent_node.get_g() + get_adjusted_cost(op));
                        }

                        // statistics.inc_expanded();
                        statistics.inc_generated();
                        active_frontier.push_back(std::move(next_state));
                    }
                }

                // If no valid successors are found, restart
                if (active_frontier.empty()) {
                    log << "[Restart] No valid successors found. Restarting EHC from the current context." << endl;
                    eval_context = current_eval_context;
                    break;
                }

                // Non-cluster: Clear and update the beam with best states
                active_beam.clear();
                std::shuffle(active_frontier.begin(), active_frontier.end(), rng);
                for (int i = 0; i < beam_width && i < active_frontier.size(); ++i) {
                    State tmp = active_frontier[i];
                    EvaluationContext new_eval_context(tmp, &statistics);
                    int h_value = new_eval_context.get_evaluator_value(evaluator.get());
                    statistics.inc_evaluated_states();
                    if (h_value < best_frontier_hvalue) {
                        improvement_found = true;
                        best_frontier_hvalue = h_value;
                        best_i = i;
                    }
                    active_beam.push_back(std::move(tmp));
                }

                if (improvement_found) {
                    EvaluationContext new_eval_context(active_frontier[best_i], &statistics);
                    current_eval_context = std::move(new_eval_context);
                    return IN_PROGRESS;
                }

                active_frontier.clear();
                timestep++;
            }

            // Reset the beam to initial state after a restart
            active_beam.clear();
        } 
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

            add_option<int>(
                    "luby_start_value",
                    "Initial value for the Luby sequence.",
                    "1");

            add_option<bool>(
                    "cluster",
                    "Cluster mode for beam search.",
                    "false");

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