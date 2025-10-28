#ifndef PROCEDURAL_ACTION_RECOGNITION_H
#define PROCEDURAL_ACTION_RECOGNITION_H

#include "procedural/action_recognition/core/internal_structures/Action.h"
#include "procedural/utils/BufferFacts.h"


namespace procedural {

class ActionRecognition {
public:
    ActionRecognition() = default;

    ~ActionRecognition() = default;

    void init(std::vector<Action*> actions, double tll = 25, int max_size = 500);

    void addToQueue(Fact* fact) const;

    void processQueue(TimeStamp_t current_time);

    void setCallback(const std::function<void(const std::vector<Graph*>&)>& callback) { callback_output_ = callback; }

    void setActiveGraphsCallback(const std::function<void(const std::vector<Graph*>&)>& callback) {
        callback_active_graphs_update_ = callback;
    }

    void linkToTaskRecognition(const std::function<void(
            const std::vector<Observation>&)>& task_recognition) { task_recognition_ = task_recognition; }


private:
    static void defaultCallback(const std::vector<Graph*>& outputs);

    static void defaultTaskRecognition(const std::vector<Observation>& observations);

    std::function<void(const std::vector<Graph*>&)> callback_output_;
    std::function<void(const std::vector<Graph*>&)> callback_active_graphs_update_;
    std::function<void(const std::vector<Observation>&)> task_recognition_;
    BufferFacts* buffer_{};
    std::vector<Action*> actions_;
    std::vector<Graph*> uncompleted_graphs_;
    std::vector<Graph*> completed_graphs_;


};

} // procedural

#endif //PROCEDURAL_ACTION_RECOGNITION_H
