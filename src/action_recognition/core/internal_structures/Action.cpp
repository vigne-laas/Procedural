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
    LOG_INFO << "[Action::build] Building simple action: " << simple_action.getName();
    LOG_INFO << "[Action::build] Number of facts to process: " << simple_action.facts.facts_.size();
    args_ = simple_action.args.args;

    // Log the arguments available
    LOG_DEBUG << "[Action::build] Available arguments in type_map:";
    for (const auto& arg : args_) {
        LOG_DEBUG << "[Action::build]   " << arg.first << " : " << arg.second;
    }

    // Debug: log action arguments
    LOG_DEBUG << "[Action::build] Action arguments (args_):";
    for (const auto& arg : args_) {
        LOG_DEBUG << "[Action::build]   '" << arg.first << "' -> '" << arg.second << "'";
    }

    for (const auto& parsed_fact: simple_action.facts.facts_) {
        // Create a modifiable copy to enrich types from action arguments
        ParsedFact_t enriched_fact = parsed_fact;

        // Enrich subject type from action arguments if it's a variable without type
        if (enriched_fact.subject.length() > 0 && enriched_fact.subject_type.empty()) {
            // Special case: ?executor or @executor is always of type Agent
            if (enriched_fact.subject == "?executor" || enriched_fact.subject == "?@executor" || enriched_fact.subject == "@executor") {
                enriched_fact.subject_type = "Agent";
                LOG_DEBUG << "[Action::build] Enriched subject '" << enriched_fact.subject << "' with type 'Agent' (special variable)";
            } else {
                // Try to find in args_ - handle both with and without '?' prefix
                std::string var_name = enriched_fact.subject;
                if (var_name[0] == '?') {
                    var_name = var_name.substr(1);  // Remove '?' prefix
                }
                auto it = args_.find(var_name);
                if (it != args_.end()) {
                    enriched_fact.subject_type = it->second;
                    LOG_DEBUG << "[Action::build] Enriched subject '" << enriched_fact.subject << "' with type '" << it->second << "' from args['" << var_name << "']";
                } else {
                    LOG_DEBUG << "[Action::build] No type found for subject '" << enriched_fact.subject << "' (searched '" << var_name << "' in args)";
                }
            }
        }
        // Enrich object type from action arguments if it's a variable without type
        if (enriched_fact.object.length() > 0 && enriched_fact.object_type.empty()) {
            // Special case: ?executor or @executor is always of type Agent
            if (enriched_fact.object == "?executor" || enriched_fact.object == "?@executor" || enriched_fact.object == "@executor") {
                enriched_fact.object_type = "Agent";
                LOG_DEBUG << "[Action::build] Enriched object '" << enriched_fact.object << "' with type 'Agent' (special variable)";
            } else {
                // Try to find in args_ - handle both with and without '?' prefix
                std::string var_name = enriched_fact.object;
                if (var_name[0] == '?') {
                    var_name = var_name.substr(1);  // Remove '?' prefix
                }
                auto it = args_.find(var_name);
                if (it != args_.end()) {
                    enriched_fact.object_type = it->second;
                    LOG_DEBUG << "[Action::build] Enriched object '" << enriched_fact.object << "' with type '" << it->second << "' from args['" << var_name << "']";
                } else {
                    LOG_DEBUG << "[Action::build] No type found for object '" << enriched_fact.object << "' (searched '" << var_name << "' in args)";
                }
            }
        }

        LOG_DEBUG << "[Action::build] Processing fact: " << enriched_fact.toString();
        LOG_DEBUG << "[Action::build]   Subject: '" << enriched_fact.subject << "' (type: '" << enriched_fact.subject_type << "')";
        LOG_DEBUG << "[Action::build]   Object: '" << enriched_fact.object << "' (type: '" << enriched_fact.object_type << "')";
        LOG_DEBUG << "[Action::build]   Level: " << enriched_fact.level << ", Required: " << enriched_fact.required;

        WordTable::properties_table.add(enriched_fact.property);
        // Use the simple constructor with enriched types
        auto new_fact = Fact(enriched_fact);
        auto obs = new ObservationFact(new_fact);
        int id_dest = (parsed_fact.level + 1) * 10;
        LOG_DEBUG << "[Action::build]   Destination node ID: " << id_dest << " (from level " << parsed_fact.level << ")";
        for (uint64_t i = last_required / 10; i < (id_dest / 10); i++) {
            LOG_DEBUG << "[Action::build]   Adding transition from " << i * 10 << " to " << id_dest << " (transition #" << count_transition << ")";
            factory_.addTransition(std::make_shared<Transition>(i * 10, obs, id_dest, count_transition));
            count_transition++;
        }
        if (parsed_fact.required) {
            last_required = id_dest;
            LOG_DEBUG << "[Action::build]   Updated last_required to: " << last_required;
        }
    }
    LOG_INFO << "[Action::build] Total transitions added: " << count_transition;

    // Store descriptions for later use when action is recognized
    descriptions_ = simple_action.descriptions;
    LOG_DEBUG << "[Action::build] Stored " << descriptions_.descriptions.size() << " descriptions";
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
            auto new_fact = Fact(*fact_it);
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
            if ((*graph_it)->getState() >= GraphState::Finished) {
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