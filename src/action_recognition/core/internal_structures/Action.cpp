#include "procedural/action_recognition/core/internal_structures/Action.h"
#include "procedural/utils/Logger.h"
#include "procedural/structures/ObservationFact.h"
#include "procedural/structures/graph/Transition.h"
#include <algorithm>

namespace procedural {
int Action::graph_id = 0;
//WordTable Action::table_actions_;

Action::Action(const std::string& name) : name_(name), factory_(Graph(name, 0, "Action_" + name))
{
    WordTable::actions_table.add(name);
    id_ = WordTable::actions_table.get(name);
}

bool Action::build(const procedural::ParsedSimpleAction_t& simple_action, const std::string& path)
{
    int last_required = 0;
    int count_transition = 0;
//    LOG_DEBUG << "start build simple_action: " << simple_action.getName() << "\n";
//    LOG_DEBUG << "Args : " << simple_action.args << "\n";
    args_ = simple_action.args.args;
    for (const auto& parsed_fact: simple_action.facts.facts_) {
//        LOG_DEBUG << "parsed fact: " << parsed_fact << "\n";
        WordTable::properties_table.add(parsed_fact.property);
        auto new_fact = Fact(parsed_fact, args_);
        auto obs = new ObservationFact(new_fact);
        int id_dest = (parsed_fact.level + 1) * 10;
        for (uint64_t i = last_required / 10; i < (id_dest / 10); i++) {
//            LOG_DEBUG << "add transition from " << i * 10 << " to " << id_dest << "with : "<<obs->toString() <<"\n";
            factory_.addTransition(std::make_shared<Transition>(i * 10, obs, id_dest, count_transition));
            count_transition++;
        }
        if (parsed_fact.required) {
            last_required = id_dest;
        }
    }
    for (const auto& description: simple_action.descriptions.descriptions) {
//        LOG_DEBUG << "description: " << description << "\n";
    }
    if (!path.empty()) {
        LOG_INFO << "Save dot : " << name_ << "\n";
        factory_.saveDot(path);
    }
    graph_id++;
    return factory_.close();

}

bool
Action::build(const ParsedComposedAction_t& composed_action, const std::vector<Action*>& actions_build,
              const std::string& path)
{
//    LOG_DEBUG << "path: " << path << "\n";
    int last_required = 0;
    int count_transition = 0;
//    LOG_DEBUG << ">>>>>>>>>>> start build composed_action: " << composed_action.getName() << "\n";
//    LOG_DEBUG << "Args : " << composed_action.args << "\n";
    args_ = composed_action.args.args;
    for (int step = 0; step < composed_action.pattern.max_level; step++) {
//        LOG_DEBUG << "step: " << step << "\n";

        // Chercher dans sub_state_machines
        auto sub_machine_it = std::find_if(composed_action.pattern.sub_state_machines.begin(),
                                           composed_action.pattern.sub_state_machines.end(),
                                           [step](const SubStateMachine_t& sub_machine) {
                                               return sub_machine.level == step;
                                           });
        if (sub_machine_it != composed_action.pattern.sub_state_machines.end()) {
//            LOG_DEBUG << "sub_machine_it: " << *sub_machine_it << "\n";
//            LOG_DEBUG << "sub_machine_it->type: " << sub_machine_it->type << "\n";
            auto action = std::find_if(actions_build.begin(), actions_build.end(),
                                       [sub_machine_it](const Action* action) {
                                           return action->getName() == sub_machine_it->type;
                                       });
            if (action == actions_build.end()) {
                LOG_ERROR << "Action not found\n";
                return false;
            }
            auto obs = new Observation(WordTable::actions_table[sub_machine_it->type],
                                       (*action)->getFactory()->getTableVariables());
            obs->setRemap(sub_machine_it->remap);
//            obs->table_variables_.set(sub_machine_it->.args)
            int id_dest = (sub_machine_it->level + 1) * 10;
//            LOG_DEBUG << "id_dest: " << id_dest << "\n";
            for (uint64_t i = last_required / 10; i < (id_dest / 10); i++) {
//                LOG_DEBUG << "add transition sub machine from " << i * 10 << " to " << id_dest << "\n";
                factory_.addTransition(std::make_shared<Transition>(i * 10, obs, id_dest, count_transition));
                count_transition++;
            }
//            factory_.addRemap(sub_machine_it->remap);
            if (sub_machine_it->required) {
                last_required = id_dest;
            }
        }

        // Chercher dans facts
        auto fact_it = std::find_if(composed_action.pattern.facts.begin(),
                                    composed_action.pattern.facts.end(),
                                    [step](const ParsedFact_t& fact) { return fact.level == step; });
        if (fact_it != composed_action.pattern.facts.end()) {
//            LOG_DEBUG << "fact_it: " << *fact_it << "\n";
            WordTable::properties_table.add(fact_it->property);
            auto new_fact = Fact(*fact_it, args_);
            auto obs = new ObservationFact(new_fact);
            int id_dest = (fact_it->level + 1) * 10;
            for (uint64_t i = last_required / 10; i < (id_dest / 10); i++) {
//                LOG_DEBUG << "add transition from " << i * 10 << " to " << id_dest << "\n";
                factory_.addTransition(std::make_shared<Transition>(i * 10, obs, id_dest, count_transition));
                count_transition++;
            }
            if (fact_it->required) {
                last_required = id_dest;
            }


        }
        this->factory_.saveDot(path + "build_step_" + std::to_string(step) + ".dot");

    }
    if (!path.empty()) {
        LOG_INFO << "Save dot : " << name_ << "\n";
        factory_.saveDot(path + "before_close.dot");
    }
//    LOG_DEBUG << "Before Close factory\n";
//    LOG_DEBUG << "Var before link : \n" << factory_.getTableVariables().toString();
    graph_id++;
    auto res = factory_.close();
//    LOG_DEBUG << "After Close factory\n";
//    LOG_DEBUG << "Var after link : \n" << factory_.getTableVariables().toString();
    return res;
}

bool Action::evolve(Observation* observation)
{
    bool match = false;
    for (auto graph_it = active_graphs_.begin(); graph_it != active_graphs_.end(); ++graph_it) {
        if ((*graph_it)->evolve(observation)) {
//            (*graph_it)->saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/evolve_" + name_ + "_.dot");
            if ((*graph_it)->getState() > GraphState::Finished) {
                finished_graphs_.push_back((*graph_it));
                graph_it = active_graphs_.erase(graph_it); //check if it works
                graph_it--;
            }
            match = true;
        }
    }
    if (!match) {
//        LOG_DEBUG << "Try to evolve factory\n";
//        factory_.saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/_factory_" + name_ + "_.dot");
        auto clone = factory_.clone(graph_id);
//        clone->saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/before_evolve_factory_" + name_ + "_.dot");
        if (clone->evolve(observation)) {
//            clone->saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/evolve_factory_" + name_ + "_.dot");
//            LOG_DEBUG << "Factory evolved\n";
            graph_id++;
            active_graphs_.push_back(clone);
            match = true;
        }
    }
    return match; // split to have evolve factory and evolve and evolve hypothesis
}

void Action::completeRemap(const std::vector<std::shared_ptr<Action>>& actions)
{
    factory_.completeRemap(actions);
}

} // action_recognition