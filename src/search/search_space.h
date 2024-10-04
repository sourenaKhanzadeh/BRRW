#ifndef SEARCH_SPACE_H
#define SEARCH_SPACE_H

#include "operator_cost.h"
#include "per_state_information.h"
#include "search_node_info.h"

#include <vector>

class OperatorProxy;
class State;
class TaskProxy;

namespace utils {
class LogProxy;
}

class SearchNode {
    State state;
    SearchNodeInfo &info;

    void update_parent(const SearchNode &parent_node,
                       const OperatorProxy &parent_op,
                       int adjusted_cost);
public:
    // Constructor
    SearchNode(const State &state, SearchNodeInfo &info);

    // Copy constructor (if needed)
    SearchNode(const SearchNode &other) : state(other.state), info(other.info) {}

    // Assignment operator
    SearchNode& operator=(const SearchNode &other) {
        if (this == &other) {
            return *this;  // Handle self-assignment
        }

        // Copy the state and info from the other object
        state = other.state;
        info = other.info;

        // Return the current object
        return *this;
    }

    const State &get_state() const;

    bool is_new() const;
    bool is_open() const;
    bool is_closed() const;
    bool is_dead_end() const;

    int get_g() const;
    int get_real_g() const;

    void open_initial();
    void open_new_node(const SearchNode &parent_node,
                       const OperatorProxy &parent_op,
                       int adjusted_cost);
    void reopen_closed_node(const SearchNode &parent_node,
                            const OperatorProxy &parent_op,
                            int adjusted_cost);
    void update_open_node_parent(const SearchNode &parent_node,
                                 const OperatorProxy &parent_op,
                                 int adjusted_cost);
    void update_closed_node_parent(const SearchNode &parent_node,
                                   const OperatorProxy &parent_op,
                                   int adjusted_cost);
    void close();
    void mark_as_dead_end();

    void dump(const TaskProxy &task_proxy, utils::LogProxy &log) const;
};



class SearchSpace {

    StateRegistry &state_registry;
    utils::LogProxy &log;
public:
    PerStateInformation<SearchNodeInfo> search_node_infos;
    SearchSpace(StateRegistry &state_registry, utils::LogProxy &log);

    SearchNode get_node(const State &state);
    void trace_path(const State &goal_state,
                    std::vector<OperatorID> &path) const;

    void dump(const TaskProxy &task_proxy) const;
    void print_statistics() const;
};

#endif
