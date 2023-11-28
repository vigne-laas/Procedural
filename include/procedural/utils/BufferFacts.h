#ifndef PROCEDURAL_BUFFERFACTS_H
#define PROCEDURAL_BUFFERFACTS_H

#include <queue>
#include <mutex>
#include "procedural/core/Types/Fact.h"

namespace procedural {

class BufferFacts
{
public:
    BufferFacts(double ttl, int max_size);

    void addFact(Fact *fact);

    std::vector<Fact *> getFacts(TimeStamp_t current_time);
    TimeStamp_t getMoreRecent() const { return more_recent_timestamp_; }
    void cleanUsedFacts(const std::set<uint32_t>& ids);

private:

    struct {
        bool operator()(Fact *left, Fact *right) const {
            return left->getTimeStamp() < right->getTimeStamp();
        }
    } customSort;

    void cleanOldFacts(TimeStamp_t current_time);

    std::mutex mutex_lock;
    std::vector<Fact*> primary_queue;
    std::vector<Fact*> secondary_queue;
    std::vector<Fact*> history_queue;
    std::vector<Fact*>* read_queue;
    std::vector<Fact*>* write_queue;
    TimeStamp_t more_recent_timestamp_;
    double ttl_;
    int  max_size_;
};

} // namespace procedural

#endif //PROCEDURAL_BUFFERFACTS_H
