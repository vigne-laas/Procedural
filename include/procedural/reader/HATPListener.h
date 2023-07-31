#ifndef PROCEDURAL_HATPLISTENER_H
#define PROCEDURAL_HATPLISTENER_H

#include "HATPParserBaseListener.h"
#include "procedural/reader/domainTypes/ParsedHTN.h"

namespace procedural {

class HATPListener : public HATPParserBaseListener
{
public:
    void enterHtn(HATPParser::HtnContext * ctx) override;
    void displayParsedValue(){std::cout << htn_ << std::endl;}
    HTNParserd_t getHTN(){return htn_;};
private:
    HTNParserd_t htn_;

};

} // procedural

#endif //PROCEDURAL_HATPLISTENER_H
