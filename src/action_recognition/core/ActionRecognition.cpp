#include "procedural/action_recognition/core/ActionRecognition.h"
#include "procedural/structures/ObservationFact.h"
#include "procedural/utils/Logger.h"
#include "procedural/utils/WordTable.h"

namespace procedural {

void ActionRecognition::init(std::vector<Action*> actions, double tll, int max_size)
{
    actions_ = actions;
    buffer_ = new BufferFacts(tll, max_size);
    callback_output_ = ActionRecognition::defaultCallback;
    task_recognition_ = ActionRecognition::defaultTaskRecognition;


}

void ActionRecognition::addToQueue(Fact* fact) const
{
    LOG_INFO << "Add to queue";
    buffer_->addFact(fact);
    LOG_INFO << "Buffer Size : " << buffer_->size();
}

void ActionRecognition::processQueue(TimeStamp_t current_time)
{
    auto facts = buffer_->getFacts(current_time);
    // LOG_INFO << "processQueue called - retrieved " << facts.size() << " facts from buffer";
    if (!facts.empty()) {
        LOG_INFO << "------------------- Process queue ---------------------";
        LOG_INFO << "Facts size: " << facts.size();
        LOG_INFO << "Facts: ";
        for (const auto& fact: facts) {
            LOG_INFO << "  Fact to process: " << fact->toString();
        }
    }
    int nb_update = 0;
    int step = 0;
     std::set<uint32_t> used_facts;
    do {
        nb_update = 0;
        std::vector<Graph*> local_uncompleted_graphs_;
        std::vector<Graph*> local_completed_graphs_;
        for (const auto& fact: facts) {
            LOG_DEBUG << ">>>>>>>>>>> Process fact: " << fact->toString();
            Observation* obs = new ObservationFact(*fact);
            for (const auto& action: actions_) {
                LOG_DEBUG << ">>>>>>>>>>> Process action: " << action->getName();
                if (action->evolve(obs)) {
                    LOG_INFO << "<<<<<<<<<<<<<<< " <<action->getName() << " evolved";
                    used_facts.insert(fact->getId());
                    auto graphs = action->getActiveGraphs();
                    auto finished = action->getFinishedGraphs();
                    LOG_DEBUG << "Active graphs size: " << graphs.size();
                    LOG_DEBUG << "Finished graphs size: " << finished.size();
                    for (const auto& graph: graphs) {
                        LOG_DEBUG << "Active graph: " << graph->getName() << " completion: " << graph->getCompletionRatio();
//                        LOG_DEBUG << "Graph: " << graph->toString();
//                        graph->saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/recognition/" + graph->getName() +"_"+std::to_string(step)+ ".dot");
                    }

                    // Combiner actifs + terminés pour notification complète
                    std::vector<Graph*> all_graphs;
                    all_graphs.insert(all_graphs.end(), graphs.begin(), graphs.end());
                    all_graphs.insert(all_graphs.end(), finished.begin(), finished.end());

                    // Publier les graphes actifs ET terminés pour notifier le visualiseur
                    if (callback_active_graphs_update_ && !all_graphs.empty()) {
                        callback_active_graphs_update_(all_graphs);
                    }
                    for (const auto& finished_graph: action->getFinishedGraphs()) {
                        if (finished_graph->getState() == GraphState::Completed) {
//                            LOG_INFO << "Action completed: " << finished_graph->getName();
//                            finished_graph->saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/recognition/completed_" +finished_graph->getName() +"_"+std::to_string(step)+ ".dot");
                            local_completed_graphs_.push_back(finished_graph);
                        } else {
//                            LOG_INFO << "Action finished: " << finished_graph->getName();
//                            finished_graph->saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/recognition/finished_" +finished_graph->getName() +"_"+std::to_string(step)+ ".dot");
                            local_uncompleted_graphs_.push_back(finished_graph);
                        }
                    }
                    nb_update++;
                }
            }
            step++;
        }
        std::vector<Observation> observations;
        for (const auto& graph: local_completed_graphs_) {
            observations.emplace_back(WordTable::actions_table.get(graph->getName()), graph->getTableVariables());
        }
        for (const auto& graph: local_uncompleted_graphs_) {
            observations.emplace_back(WordTable::actions_table.get(graph->getName()), graph->getTableVariables());
        }
        for (auto& obs : observations) {
//            LOG_INFO << "<<<<<<<<<<<<<<<< Observation: " << obs.toString();
            for (const auto& action: actions_) {
//                LOG_DEBUG << ">>>>>>>>>>> Process action: " << action->getName();
                if (action->evolve(&obs)) {
//                    LOG_INFO << "<<<<<<<<<<<<<<< " <<action->getName() << " evolved";
//                    auto graphs = action->getActiveGraphs();
//                    LOG_DEBUG << "Graphs size: " << graphs.size();
//                    for (const auto& graph: graphs) {
//                        LOG_DEBUG << "Graph completion : " << graph->getName() << " completion rate : "<< graph->getCompletionRatio();
//                        graph->saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/recognition/" + graph->getName() +"_"+std::to_string(step)+ ".dot");
//                    }
                    for (const auto& finished_graph: action->getFinishedGraphs()) {
                        if (finished_graph->getState() == GraphState::Completed) {
//                            LOG_INFO << "Action completed: " << finished_graph->getName();
                            local_completed_graphs_.push_back(finished_graph);
                        } else {
//                            LOG_INFO << "Action finished: " << finished_graph->getName();
                            local_uncompleted_graphs_.push_back(finished_graph);
                        }
                    }
                    nb_update++;
                }
            }
            step++;
        }
        facts.clear();
//        all_observations.insert(all_observations.end(), observations.begin(), observations.end());
        uncompleted_graphs_.insert(uncompleted_graphs_.end(), local_uncompleted_graphs_.begin(), local_uncompleted_graphs_.end());
        completed_graphs_.insert(completed_graphs_.end(), local_completed_graphs_.begin(), local_completed_graphs_.end());
    } while (nb_update > 0);

    if(!uncompleted_graphs_.empty()) {
        callback_output_(uncompleted_graphs_);
    }
    if(!completed_graphs_.empty()) {
        callback_output_(completed_graphs_);
    }
    std::vector<Observation> observations;
    for (const auto& graph: completed_graphs_) {
        observations.emplace_back(WordTable::actions_table.get(graph->getName()), graph->getTableVariables());
    }
    for (const auto& graph: uncompleted_graphs_) {
        observations.emplace_back(WordTable::actions_table.get(graph->getName()), graph->getTableVariables());
    }
    buffer_->cleanUsedFacts(used_facts);
    task_recognition_(observations);
    completed_graphs_.clear();

    // Clear finished graphs from actions to prevent memory accumulation
    for (auto* action : actions_) {
        action->clearFinishedGraphs();
    }
}

void ActionRecognition::defaultCallback(const std::vector<Graph*>& outputs)
{
    LOG_INFO << "Default callback finished actions:\n";
    for (const auto& graph: outputs) {
        LOG_INFO << graph->toString();
    }
}

void ActionRecognition::defaultTaskRecognition(const std::vector<Observation>& observations)
{
    if(observations.empty()) {
        return;
    }
    LOG_INFO << "Default task recognition\n :";
    for (const auto& observation: observations) {
        LOG_INFO << observation.toString();
    }
}

} // procedural