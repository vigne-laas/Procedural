#ifndef PROCEDURAL_ACTION_H
#define PROCEDURAL_ACTION_H

#include <string>
#include "procedural/action_recognition/reader/types/ParsedSimpleAction.h"
#include "procedural/action_recognition/reader/types/ParsedComposedAction.h"
#include "procedural/structures/graph/Graph.h"
#include "procedural/utils/WordTable.h"

namespace procedural {

class Action {
public:
    Action(const std::string& name);

    bool build(const ParsedSimpleAction_t& simple_action, const std::string& path = "");

    bool build(const ParsedComposedAction_t& composed_action,const std::vector<Action*>& actions_build,const std::string& path = "");

    bool evolve(Observation* observation);

    std::string getName() const { return name_; }
    uint32_t getId() const { return id_; }
    Graph* getFactory() { return &factory_; }
    std::vector<Graph*> getActiveGraphs() { return active_graphs_; }
    std::vector<Graph*> getHypothesisGraphs() { return hypothesis_graphs_; }
    std::vector<Graph*> getFinishedGraphs() { return finished_graphs_; }

    void clearFinishedGraphs() { finished_graphs_.clear(); }
    ParsedDescriptions_t getDescriptions() const { return descriptions_; }

    void completeRemap(const std::vector<std::shared_ptr<Action>>& actions);
    std::map<std::string, std::string> getArgs() { return args_; }

//    void display() const;

//    static WordTable table_actions_;

private:
    static int graph_id;
    std::string name_;
    std::map<std::string, std::string> args_;
    ParsedDescriptions_t descriptions_;
    Graph factory_;
    std::vector<std::string> parameters_;
    std::vector<Graph*> active_graphs_;
    std::vector<Graph*> hypothesis_graphs_;
    std::vector<Graph*> finished_graphs_;
    uint32_t id_;
};

} // action_recognition

#endif //PROCEDURAL_ACTION_H
