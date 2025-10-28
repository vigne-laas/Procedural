#include "procedural/structures/graph/Transition.h"

namespace procedural {
void Transition::expandProperty(onto::ObjectPropertyClient* object_property_client)
{
    observation_->expandProperty(object_property_client);
}

void Transition::linkVariables(std::map<std::string, std::shared_ptr<Variable_t>>& variables)
{
    observation_->linkVariables(variables);
}

uint64_t Transition::match(const Observation* observation) const
{
    return (*observation_) == (*observation) ? target_id_ : 0;
}

std::string Transition::toDot() const
{
    std::string label = "T" + std::to_string(id_) + ": " + observation_->toString();
    return "\"" + std::to_string(source_id_) + "\" -> \"" + std::to_string(target_id_) + "\" [label=\"" +
           label + "\", fontsize=10];\n";
}

std::string Transition::toString() const
{
    std::string result = "Transition " + std::to_string(id_) + " from " + std::to_string(source_id_) + " to " +
                         std::to_string(target_id_) + "\n";
    result += observation_->toString();
    return result;
}

std::ostream& operator<<(std::ostream& os, const Transition& transition)
{
    os << transition.toString();
    return os;
}

void Transition::completeRemap(const std::vector<std::shared_ptr<Action>>& actions)
{
    observation_->completeVar(actions);

}
} // procedural