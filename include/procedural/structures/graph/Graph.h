#ifndef PROCEDURAL_GRAPH_H
#define PROCEDURAL_GRAPH_H

#include <exception>
#include <string>
#include <unordered_set>
#include <memory>
#include "procedural/structures/graph/Node.h"
#include "procedural/utils/WordTable.h"

namespace procedural {

class Action;

enum class GraphState {
    UnClosed = 0,
    Closed = 1,
    Factory = 2,
    Active = 3,
    Hypothesis = 4,
    Finished = 5,
    Completed = 6


};

std::string GraphStateToString(GraphState state) {
    switch (state) {
        case GraphState::UnClosed:
            return "UnClosed";
        case GraphState::Closed:
            return "Closed";
        case GraphState::Factory:
            return "Factory";
        case GraphState::Active:
            return "Active";
        case GraphState::Hypothesis:
            return "Hypothesis";
        case GraphState::Finished:
            return "Finished";
        case GraphState::Completed:
            return "Completed";
        default:
            return "Unknown GraphState";
    }
}

struct GraphException : public std::exception {
    std::string msg_;

    explicit GraphException(const std::string& msg) : msg_(msg) {}

    const char* what() const throw()
    {
        return msg_.c_str();
    }
};

struct NoInitialNodeGraphException : public GraphException {
    NoInitialNodeGraphException() : GraphException(
            "Invalid State Machine due to no initial State detected") {};
};

struct NoFinalNodeGraphException : public GraphException {
    NoFinalNodeGraphException() : GraphException(
            "Invalid State Machine due to no final State detected") {};
};

struct MultiInitialNodeGraphException : public GraphException {
    explicit MultiInitialNodeGraphException(const std::unordered_set<std::shared_ptr<Node>>& invalid_nodes) :
            GraphException(
                    "Invalid State Machine due to no initial State detected\nState detected as initial are : ")
    {
        for (auto& node: invalid_nodes)
            msg_ += node->toString() + "\n";
    }
};

struct MultiFinalNodeGraphException : public GraphException {
    explicit MultiFinalNodeGraphException(const std::unordered_set<std::shared_ptr<Node>>& invalid_nodes) :
            GraphException(
                    "Invalid State Machine due to no final State detected\nState detected as final are : ")
    {
        for (auto& node: invalid_nodes)
            msg_ += node->toString() + "\n";
    }
};


class Graph {

public:
    Graph(const std::string& name, int64_t id, const std::string& type_str_);

    Graph(const Graph& other) = delete;


    Graph* clone(int new_id);

    bool evolve(const Observation* observation);


    bool addTransition(std::shared_ptr<Transition> transition);

//    bool addDescription(Description* description);
    bool close();

    std::string getName() const { return name_; }

    GraphState getState() const { return state_; }

    std::string toString() const;

    friend std::ostream& operator<<(std::ostream& os, const Graph& graph);


    static WordTable graph_table;

    std::map<uint64_t, std::shared_ptr<Node>>& getNodes() { return nodes_; }

    std::shared_ptr<Node> getInitialNode() { return initial_node_; }
    std::shared_ptr<Node> getFinalNode() { return final_node_; }
    std::shared_ptr<Node> getCurrentNode() { return current_node_; }

    void saveDot(const std::string& path);

    VariableTable_t& getTableVariables() { return table_variables_; }

    double getCompletionRatio() const;

    double getAdvancementRatio() const;

    void completeRemap(const std::vector<std::shared_ptr<Action>>& actions);

    void addRemap(const std::map<std::string,std::string>& remap);

private:

    std::string name_;
    std::string type_str_;
    int64_t id_;
    uint32_t type_id_;


    std::shared_ptr<Node> initial_node_;
    std::shared_ptr<Node> final_node_;
    std::shared_ptr<Node> current_node_;
    std::map<uint64_t, std::shared_ptr<Node>> nodes_;

    GraphState state_ = GraphState::UnClosed;
    VariableTable_t table_variables_;



    void linkGraph();

    bool processInitialNode();

    bool processFinalNode();

    void addNode(uint64_t id);
};

} // procedural

#endif //PROCEDURAL_GRAPH_H
