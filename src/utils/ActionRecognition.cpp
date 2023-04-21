#include <set>
#include "procedural/core/Types/Action.h"
#include "procedural/utils/ActionRecognition.h"
#include <iostream>

namespace procedural {
ActionRecognition::ActionRecognition(const std::vector<Action*>& actions, double ttl, int max_size) : actions_(
        actions), buffer_(ttl, max_size)
{

}

void ActionRecognition::addToQueue(Fact* fact)
{
    buffer_.addFact(fact);
}

void ActionRecognition::processQueue(TimeStamp_t current_time)
{
    std::vector<Fact*> list_facts = buffer_.getFacts(current_time);
    std::set<uint32_t> facts_used;
    for (auto fact: list_facts)
    {

//            std::cout << "--------------" << std::endl;
//            std::cout << "fact in Action recognition: " << fact->toString() << std::endl;
        std::set<uint32_t> set_id_facts;
        std::vector<procedural::Action*> complete_actions;
        for (auto& action: actions_)
            action->feed(fact);

        int nb_update;
        do
        {
            nb_update = 0;
            for (auto& action: actions_)
            {
                std::set<uint32_t> temp_set = action->checkCompleteNetworks(current_time);
                if (temp_set.empty() == false)
                {
                    set_id_facts.insert(temp_set.begin(), temp_set.end());
                    complete_actions.push_back(action);
                    nb_update++;
                }

            }
//            std::cout<< "nb complete network : " << nb_update << std::endl;
//            nb_update = (int)complete_actions.size();
            for (auto& action_complete: complete_actions)
                for (auto& action: actions_)
                    if (action != action_complete)
                        if (action->checkSubAction(action_complete))
                        {
                            std::cout << "\t\t\t update for " << action->getName()
                                      << "evolve thanks to complete of sub action : " << action_complete->getName()
                                      << std::endl;
                            nb_update++;
                        }
            if (nb_update != 0)
                for (auto& action: actions_)
                    action->cleanPatterns(set_id_facts);

            complete_actions.clear();
        } while (nb_update != 0);

        for (auto& action: actions_)
            action->clean();

        facts_used.insert(set_id_facts.begin(), set_id_facts.end());
    }
    buffer_.cleanUsedFacts(facts_used);
}


} // procedural