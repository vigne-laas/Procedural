#include <set>
#include "procedural/core/Types/ActionType.h"
#include "procedural/utils/ActionRecognition.h"
#include <iostream>

namespace procedural {
ActionRecognition::ActionRecognition(const std::vector<ActionType*>& actions, double ttl, int max_size) : actions_(
        actions), callback_output_(ActionRecognition::defaultCallback)
{
    double max_ttl = 0;
    for (auto& action: actions_)
        if (action->maxTtl() > max_ttl)
            max_ttl = action->maxTtl();
    if (ttl > max_ttl)
        max_ttl = ttl;
    buffer_ = new BufferFacts(max_ttl, max_size);
}

void ActionRecognition::addToQueue(Fact* fact) const
{
    buffer_->addFact(fact);
}

void ActionRecognition::processQueue(TimeStamp_t current_time)
{
    std::vector<Fact*> list_facts = buffer_->getFacts(current_time);
    std::set<uint32_t> facts_used;
    for (auto fact: list_facts)
    {

         std::cout << "--------------" << std::endl;
         std::cout << "fact in Action recognition: " << fact->toString() << " " << fact->getTimeStamp() << "\n\n"
                   << std::endl;
        std::set<uint32_t> set_id_facts;
        std::vector<procedural::ActionType*> complete_actions;
        for (auto& action: actions_)
            if(action->feed(fact, current_time))
                facts_used.insert(fact->getId());

        int nb_update;
        do
        {
            nb_update = 0;
            std::vector<NetworkOutput> outputs;
            for (auto& action: actions_)
            {
                std::set<uint32_t> temp_set = action->checkCompleteNetworks(current_time);
                if (temp_set.empty() == false)
                {
                    set_id_facts.insert(temp_set.begin(), temp_set.end());
                    complete_actions.push_back(action);
                    nb_update++;
                } else
                {
                    if (action->checkNewExplanation())
                    {
                        auto nets = action->getNewExplanation();
                        for (auto& net: nets)
                            outputs.emplace_back(net, true);
                    }
                }

            }
//            std::cout<< "nb complete network : " << nb_update << std::endl;
//            nb_update = (int)complete_actions.size();
            for (auto& action_complete: complete_actions)
            {
                for (auto& action: actions_)
                    if (action != action_complete)
                    {
                        if (action->checkSubAction(action_complete))
                        {
                            // std::cout << "\t\t\t update for " << action->getName()
                            //           << "evolve thanks to complete of sub action : " << action_complete->getName()
                            //           << std::endl;
                            if (action->checkNewExplanation())
                            {
                                auto nets = action->getNewExplanation();
                                for (auto& net: nets)
                                    outputs.emplace_back(net, true);
                            }
                            nb_update++;
                        }
                    }

                auto networks = action_complete->getCompleteNetworks();
                for (auto& net: networks)
                {
//                    auto output = NetworkOutput(net);
//                    outputs.push_back(output);
                    outputs.emplace_back(net);
                }


            }

            callback_output_(outputs);


            if (nb_update != 0)
                for (auto& action: actions_)
                    action->cleanPatterns(set_id_facts);

            complete_actions.clear();
        } while (nb_update != 0);

        for (auto& action: actions_)
            action->clean();

        facts_used.insert(set_id_facts.begin(), set_id_facts.end());
    }
    buffer_->cleanUsedFacts(facts_used);
}
void ActionRecognition::defaultCallback(const std::vector<NetworkOutput>& outputs)
{
    for (const auto& output: outputs)
        std::cout << ">> " << output << std::endl;

}


} // procedural