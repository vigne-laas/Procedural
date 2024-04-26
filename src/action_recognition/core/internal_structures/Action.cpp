#include "procedural/action_recognition/core/internal_structures/Action.h"
#include "procedural/utils/Logger.h"
#include "procedural/structures/ObservationFact.h"
#include "procedural/structures/graph/Transition.h"
#include <algorithm>

namespace procedural {
int Action::graph_id = 0;
WordTable Action::table_actions_;

Action::Action(const std::string& name) : name_(name), factory_(Graph(name, 0, "Action_" + name))
{
    table_actions_.add(name);
    id_ = table_actions_.get(name);
}

bool Action::build(const procedural::ParsedSimpleAction_t& simple_action, const std::string& path)
{
    int last_required = 0;
    int count_transition = 0;
    for (const auto& parsed_fact: simple_action.facts.facts_) {
//        LOG_DEBUG << "parsed fact: " << parsed_fact << "\n";
        Fact::properties_table.add(parsed_fact.property);
        auto new_fact = Fact(parsed_fact.insertion, parsed_fact.subject, parsed_fact.property, parsed_fact.object);
        auto obs = new ObservationFact(new_fact);
        int id_dest = parsed_fact.level * 10;
        for (uint64_t i = last_required / 10; i < (id_dest / 10); i++) {
            factory_.addTransition(std::make_shared<Transition>(i * 10, obs, id_dest, count_transition));
            count_transition++;
        }
        if (parsed_fact.required) {
            last_required = id_dest;
        }
    }
    for (const auto& description: simple_action.descriptions.descriptions) {
        LOG_DEBUG << "description: " << description << "\n";
    }
    if (!path.empty()) {
        LOG_INFO << "Save dot\n";
        factory_.saveDot(path);
    }
    graph_id++;
    return factory_.close();

}

bool Action::build(const ParsedComposedAction_t& composed_action, const std::string& path)
{

    int last_required = 0;
    int count_transition = 0;

    for (int step = 0; step < composed_action.pattern.max_level; step++) {
        LOG_DEBUG << "step: " << step << "\n";

        // Chercher dans sub_state_machines
        auto sub_machine_it = std::find_if(composed_action.pattern.sub_state_machines.begin(),
                                           composed_action.pattern.sub_state_machines.end(),
                                           [step](const SubStateMachine_t& sub_machine) {
                                               return sub_machine.level == step;
                                           });
        if (sub_machine_it != composed_action.pattern.sub_state_machines.end()) {
            LOG_DEBUG << "sub_machine_it: " << *sub_machine_it << "\n";
            auto obs = new Observation(table_actions_[sub_machine_it->type]);
            obs->setRemap(sub_machine_it->remap); // a discuter
            int id_dest = (sub_machine_it->level + 1) * 10;
            LOG_DEBUG << "id_dest: " << id_dest << "\n";
            for (uint64_t i = last_required / 10; i < (id_dest / 10); i++) {
                LOG_DEBUG << "add transition sub machine from " << i * 10 << " to " << id_dest << "\n";
                factory_.addTransition(std::make_shared<Transition>(i * 10, obs, id_dest, count_transition));
                count_transition++;
            }
            if (sub_machine_it->required) {
                last_required = id_dest;
            }

        }

        // Chercher dans facts
        auto fact_it = std::find_if(composed_action.pattern.facts.begin(),
                                    composed_action.pattern.facts.end(),
                                    [step](const ParsedFact_t& fact) { return fact.level == step; });
        if (fact_it != composed_action.pattern.facts.end()) {
            LOG_DEBUG << "fact_it: " << *fact_it << "\n";
            Fact::properties_table.add(fact_it->property);
            auto new_fact = Fact(fact_it->insertion, fact_it->subject, fact_it->property,
                                 fact_it->object);
            auto obs = new ObservationFact(new_fact);
            int id_dest = (fact_it->level + 1) * 10;
            for (uint64_t i = last_required / 10; i < (id_dest / 10); i++) {
                LOG_DEBUG << "add transition from " << i * 10 << " to " << id_dest << "\n";
                factory_.addTransition(std::make_shared<Transition>(i * 10, obs, id_dest, count_transition));
                count_transition++;
            }
            if (fact_it->required) {
                last_required = id_dest;
            }
        }
    }
    if (!path.empty()) {
        LOG_INFO << "Save dot\n";
        factory_.saveDot(path);
    }
    graph_id++;
    return factory_.close();
}

bool Action::evolve(Observation* observation)
{
    bool match = false;
    for (auto graph_it = active_graphs_.begin(); graph_it != active_graphs_.end(); ++graph_it) {
        if ((*graph_it)->evolve(observation)) {
            if ((*graph_it)->getState() > GraphState::Finished) {
                finished_graphs_.push_back((*graph_it));
                graph_it = active_graphs_.erase(graph_it); //check if it works
                graph_it--;
            }
            match = true;
        }
    }
    if (!match) {
        LOG_DEBUG << "Try to evolve factory\n";
        auto clone = factory_.clone(graph_id);
        if (clone->evolve(observation)) {
            LOG_DEBUG << "Factory evolved\n";
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