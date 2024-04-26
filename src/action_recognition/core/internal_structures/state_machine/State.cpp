#include "procedural/action_recognition/core/internal_structures/state_machine/State.h"

namespace procedural {

State::State(const std::string& name, int id):name_(name), id_(id),initial_node_(false), final_node_(false), level_(0)
{

}

StateEvolveResult_t* State::evolve(ActionObservation_t* obs)
{
    auto res = new StateEvolveResult_t;
//    for(auto& pair: childreens)
//        if(pair.first.match(obs))
//        {
//            res->result = EvolveResult::EVOLVE;
//            res->next_state = pair.second;
//            if(pair.first.hasRemap)
//                res->remap = pair.first.remap;
//        }
    return res;
}

void State::close(std::map<std::string, Variable_t>& variables_)
{
    for (auto& pair: childreens)
    {
        pair.first->linkVariables(variables_);
    }
}

void State::expandTransitions(onto::OntologyManipulator* onto_manipulator)
{

}

std::string State::toString() const
{
    return std::string();
}

std::string State::toShortString() const
{
    return std::string();
}

void State::saveDOTFile(std::ofstream& dot_file) const
{

}


} // procedural