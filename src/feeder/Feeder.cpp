#include "procedural/feeder/Feeder.h"
#include <iostream>

namespace procedural {

Feeder::Feeder() : pattern(R"(\[([^\]]+)\]([^|]+)\|([^|]+)\|(.+))"), callback_buffer(&Feeder::default_callback) {

}
Fact* Feeder::parse(const std::string& str_fact, int id, TimeStamp_t fact_timestamp)
{
    std::smatch results;
    std::regex_search(str_fact, results, pattern);
//    for(auto& res : results)
//        std::cout << "res: " << res << std::endl;
//    std::unique_ptr<Fact> fact(new Fact(results[1].str() == "DEL" ? false : true, results[2], results[3], results[4], id, fact_timestamp));
    return new Fact(results[1].str() == "DEL" ? false : true, results[2], results[3], results[4], id, fact_timestamp);

}
void Feeder::feed(const std::string& str_fact, int id, TimeStamp_t fact_timstamp)
{
    auto fact = parse(str_fact, id, fact_timstamp);
    if(fact->isValid())
        callback_buffer(fact);
    else
        std::cout << "rejected fact because invalid :" << str_fact << std::endl;

}
void Feeder::feed(bool added,const std::string& subject,const std::string& property,const std::string& object, int id, TimeStamp_t fact_timstamp)
{
    auto fact = new Fact(added,subject,property,object,id,fact_timstamp);
    if(fact->isValid())
        callback_buffer(fact);
    else
        std::cout << "rejected fact because invalid "<< subject << "|" << property << "|" << object  << std::endl;

}
void Feeder::default_callback(Fact* fact)
{
    std::cout << "<< " << fact->toString() << std::endl;
}


} // procedural