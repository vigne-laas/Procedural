#include "procedural/structures/graph/Node.h"

namespace procedural {

uint64_t Node::match(const Observation* observation) const
{
    for (const auto& transition: transitions_) {
        uint64_t target_id = transition->match(observation);
//        LOG_DEBUG << "Transition " << *transition << " match result: " << target_id;
        if (target_id != 0) {
            return target_id;
        }
    }
    return 0;
}

std::string Node::toDot(int current_node_id) const
{
    std::string shape;
    if (id_ == current_node_id) {
        shape = "box";
    } else if (id_ == 0) { // initial node
        shape = "ellipse";
    } else if (isFinal()) {
        shape = "doublecircle";
    } else {
        shape = "circle";
    }

    std::string result = "\"" + std::to_string(id_) + "\" [shape=" + shape + ", label=\"" + getFullName() + "\"];\n";
    for (const auto& transition: transitions_) {
        result += transition->toDot();
    }
    return result;
}

void Node::completeRemap(const std::vector<std::shared_ptr<Action>>& actions)
{
    for (const auto& transition: transitions_) {
        transition->completeRemap(actions);
    }

}

} // procedural

