//#include "procedural/core/Graph/Transitions/TransitionActionMethod.h"
//#include "procedural/core/Types/ActionMethod.h"
//
//namespace procedural {
//TransitionActionMethod::TransitionActionMethod(uint32_t id_action, int next_state,
//                                               const std::map<std::string, std::string>& arguments)
//        : action_id_(id_action),
//          id_next_state_(next_state), arguments_(arguments)
//{
//    for (const auto& arg: arguments)
//        variables_.insert(std::make_pair(arg.first, nullptr));
//}
//
//TransitionActionMethod::TransitionActionMethod(const TransitionActionMethod& transition, int id_next_state) :
//        action_id_(transition.action_id_),
//        id_next_state_(id_next_state),
//        arguments_(transition.arguments_)
//{
//    for (const auto& arg: arguments_)
//        variables_.insert(std::make_pair(arg.first, nullptr));
//}
//
//void TransitionActionMethod::linkVariables(std::map<std::string, Variable_t>& variables)
//{
////    std::cout << "link variables action method : " << std::endl;
//    for (auto& pair: variables_)
//    {
////        std::cout << "var : " << pair.first << " count : " << std::to_string(variables.count(pair.first)) << std::endl;
////        if (variables.count(pair.first) > 0)
//            pair.second = &(variables.at(pair.first));
////        else
////            std::cout << "variables not found : " << pair.first << std::endl;
//    }
//
//}
//bool TransitionActionMethod::match(Action* action)
//{
//    if (action->getType() != action_id_)
//        return false;
//    for (auto action: action->getFinishedStateMachine())
//        for (auto var_key: action->getLiteralVariables())
//
//            return false;
//    return true;
//}
//std::string TransitionActionMethod::toString() const
//{
//    std::string res =
//            "Action Method Transitions type : " + std::to_string(action_id_) + "(" + ActionMethod::action_method_types.get(action_id_) +
//            ") to " + std::to_string(id_next_state_) + " \n";
//    if (!variables_.empty())
//    {
//        res += "Variables : \n";
//        for (auto& var: variables_)
//            res += "\t" + var.first + " : " + var.second->toString() + "\n";
//    }
//    return res;
//
//
//}
//void TransitionActionMethod::setOntologyClient(onto::IndividualClient* indiv_client)
//{
//
//}
//
//} // procedural