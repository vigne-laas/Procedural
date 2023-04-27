#ifndef PROCEDURAL_FEEDER_H
#define PROCEDURAL_FEEDER_H

#include <regex>
#include <string>
#include <vector>
#include "procedural/core/Types/Fact.h"

namespace procedural {

class Feeder
{
public:
    Feeder();
    void feed(const std::string& fact,int id,TimeStamp_t fact_timstamp);
    void setCallback(const std::function<void(Fact*)>& new_callback){callback_buffer = new_callback;};

private:
    Fact* parse(const std::string& fact,int id,TimeStamp_t fact_timestamp);
    static void  default_callback(Fact*);
    std::function<void(Fact*)> callback_buffer;
    std::regex const pattern;



};

} // procedural

#endif //PROCEDURAL_FEEDER_H
