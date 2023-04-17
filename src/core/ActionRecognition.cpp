#include <set>
#include "procedural/core/Types/Action.h"
#include "procedural/core/ActionRecognition.h"
#include <iostream>

namespace procedural {
    ActionRecognition::ActionRecognition(std::vector<Action *> *actions) : actions_(actions), primary_queue(),
                                                                           secondary_queue(),
                                                                           write_queue(&primary_queue),
                                                                           read_queue(&secondary_queue) {
    }

    void ActionRecognition::addToQueue(Fact *fact) {
        mutex_lock.lock();
        write_queue->push(fact);
        mutex_lock.unlock();
    }

    void ActionRecognition::processQueue() {
        mutex_lock.lock();
        auto temp = write_queue;
        write_queue = read_queue;
        read_queue = temp;
        mutex_lock.unlock();

        while(!read_queue->empty())
        {
            auto fact = read_queue->front();
            std::cout << "--------------" << std::endl;
            std::cout << "fact in Action recognition: " << fact->toString() << std::endl;
            std::set<uint32_t> set_id_facts;
            std::vector<procedural::Action*> complete_actions;
            for(auto& action : (*actions_))
                action->feed(fact);

            int nb_update = 0;
            do
            {
                nb_update = 0;
                for(auto& action: (*actions_))
                {
                    std::set<uint32_t> temp_set = action->checkCompleteNetworks();
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
                    for (auto& action: (*actions_))
                        if (action != action_complete)
                            if(action->checkSubAction(action_complete))
                            {
                                std::cout << "\t\t\t update for "<<action->getName()<<"evolve thanks to complete of sub action : " << action_complete->getName() << std::endl;
                                nb_update++;
                            }
                if(nb_update!=0)
                    for(auto& action: (*actions_))
                        action->cleanPatterns(set_id_facts);

                complete_actions.clear();
            } while (nb_update !=0);

            for (auto& action: (*actions_))
                action->clean();

            read_queue->pop();
        }


    }


} // procedural