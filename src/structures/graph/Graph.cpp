#include "procedural/structures/graph/Graph.h"
#include "procedural/utils/Logger.h"
#include <memory>
#include "procedural/structures/ObservationFact.h"
#include <filesystem>
#include "procedural/action_recognition/core/internal_structures/Action.h"


namespace procedural {
WordTable Graph::graph_table;

Graph::Graph(const std::string& name, int64_t id, const std::string& type_str) : name_(name), id_(id),
                                                                                 type_str_(type_str)
{
    table_variables_.variables["self"] = std::make_shared<Variable_t>(getName());
    type_id_ = graph_table.get(type_str_);
}

Graph* Graph::clone(int new_id)
{
    LOG_DEBUG << "Cloning graph: " + name_ + " with id: " + std::to_string(id_) + " to id: " + std::to_string(new_id);
    if (state_ < GraphState::Closed)
        return nullptr;
    auto new_graph = new Graph(name_, new_id, type_str_);
    for (const auto& node: nodes_) {
        for (const auto& transition: node.second->getTransitions()) {
            Observation* new_obs;
            if (transition->getObservation()->getId() < 0) {
                auto new_fact = (dynamic_cast<ObservationFact*>(transition->getObservation()))->getFact();
                new_obs = new ObservationFact(new_fact);
            } else
                new_obs = new Observation(*(transition->getObservation()));

            auto new_transition = std::make_shared<Transition>(transition->getSourceId(), new_obs,
                                                               transition->getTargetId(), transition->getId());
//            LOG_DEBUG << "Adding transition: " + new_transition->toString();
            new_graph->addTransition(new_transition);
//            new_graph->saveDot(
//                    "/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/clone_" + name_ + "_" +
//                    std::to_string(new_id) + "T_" + std::to_string(transition->getId()) + ".dot");
        }
    }
//    for (const auto& var: table_variables_.variables) {
//        new_graph->table_variables_.variables[var.first] = std::make_shared<Variable_t>(*var.second);
//    }
    new_graph->close();
    return new_graph;
}

bool Graph::evolve(const Observation* observation)
{
    if (state_ < GraphState::Closed)
        return false;
    auto res = current_node_->match(observation);
    if (res == 0) {
        LOG_DEBUG << "No match found for observation: " + observation->toString();
        return false;
    }
    current_node_ = nodes_[res];
    if (current_node_->getId() == final_node_->getId()) {
        state_ = GraphState::Finished;
        if (getCompletionRatio() == 1.0) {
            state_ = GraphState::Completed;
        }
    }
    return true;
}

bool Graph::addTransition(std::shared_ptr<Transition> transition)
{
    if (state_ != GraphState::UnClosed)
        return false;
    if (nodes_.find(transition->getSourceId()) == nodes_.end()) {
        addNode(transition->getSourceId());
    }
    if (nodes_.find(transition->getTargetId()) == nodes_.end()) {
        addNode(transition->getTargetId());
    }
    nodes_[transition->getSourceId()]->addTransition(transition);
    nodes_[transition->getTargetId()]->addParent(transition->getSourceId());
    table_variables_.set(transition->getTableVariables());
    return true;
}

bool Graph::close()
{
    auto res = true;
    linkGraph();
    res |= processInitialNode();
    res |= processFinalNode();
    if (res)
        state_ = GraphState::Closed;
    return res;
}

std::string Graph::toString() const
{
    return name_;
}

void Graph::linkGraph()
{

    for (const auto& node: nodes_) {
        for (const auto& transition: node.second->getTransitions()) {
            transition->linkVariables(table_variables_.variables);
        }
    }

}

bool Graph::processInitialNode()
{
    std::unordered_set<uint64_t> id_next_nodes;
    std::unordered_set<uint64_t> id_nodes;
    for (const auto& node: nodes_) {
        for (const auto& transition: node.second->getTransitions()) {
            id_next_nodes.insert(transition->getTargetId());
        }
        id_nodes.insert(node.first);
    }


    std::unordered_set<uint64_t> diff;
    std::set_difference(id_nodes.begin(), id_nodes.end(), id_next_nodes.begin(),
                        id_next_nodes.end(), std::inserter(diff, diff.begin()));

    if (diff.size() == 1) {
        initial_node_ = nodes_[*diff.begin()];
        current_node_ = initial_node_;
        return true;
    }
    if (diff.size() > 1) {
        LOG_ERROR << "Multiple initial nodes found\n";
        std::unordered_set<std::shared_ptr<Node>> invalid_nodes;
        for (const auto& id: diff) {
            invalid_nodes.insert(nodes_[id]);
        }
        throw MultiInitialNodeGraphException(invalid_nodes);
    }
    if (diff.empty()) {
        LOG_ERROR << "No initial node found \n";
        throw NoInitialNodeGraphException();
    }
    return false;

}

bool Graph::processFinalNode()
{
    std::unordered_set<std::shared_ptr<Node>> final_nodes;
    for (const auto& node: nodes_) {
        if (node.second->isFinal()) {
            final_nodes.insert(node.second);
        }
    }
    if (final_nodes.size() == 1) {
        final_node_ = *final_nodes.begin();
        return true;
    }
    if (final_nodes.size() > 1) {
        LOG_ERROR << "Multiple final nodes found\n";
        throw MultiFinalNodeGraphException(final_nodes);
    }
    if (final_nodes.empty()) {
        LOG_ERROR << "No final node found \n";
        throw NoFinalNodeGraphException();
    }
    return false;
}

double Graph::getCompletionRatio() const
{
    double res = 0;
    for (const auto& var: table_variables_.variables) {
        if (var.second->isSet())
            res += 1;
    }
    if (!table_variables_.agents.empty()) {
        res += 1;
        res = res / ((double) (table_variables_.variables.size() - 1) + 1.0);
    } else
        res = res / ((double) (table_variables_.variables.size() - 1));

    return res;
}

double Graph::getAdvancementRatio() const
{
    return current_node_->getDepth() / final_node_->getDepth();
}

void Graph::addNode(uint64_t id)
{
    nodes_[id] = std::make_shared<Node>(id, name_, (int) (id / 10));
}

std::ostream& operator<<(std::ostream& os, const Graph& graph)
{
    return os << graph.toString() + "_" + std::to_string(graph.id_);
}

void Graph::saveDot(const std::string& path)
{
    std::filesystem::path fs_path(path);
    std::string file_path = path;

    if (std::filesystem::is_directory(fs_path)) {
        file_path = fs_path.append(this->getName() + "_" + std::to_string(this->id_) + ".dot").string();
    }
    std::ofstream file(file_path);
    if (!file) {
        throw std::runtime_error("Unable to open file: " + path);
    }
    int current_node_id = 0;
    if (current_node_ != nullptr) {
        current_node_id = (int) current_node_->getId();
    }
    file << "digraph {\n";
    for (const auto& pair: nodes_) {
        file << pair.second->toDot(current_node_id);
    }
    file << "}\n";

    file.close();

}

void Graph::completeRemap(const std::vector<std::shared_ptr<Action>>& actions)
{
    for (const auto& node : nodes_) {
        node.second->completeRemap(actions);
    }

}

} // procedural