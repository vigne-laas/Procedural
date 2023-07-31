#ifndef PROCEDURAL_DOMAINREADER_H
#define PROCEDURAL_DOMAINREADER_H
#include <iostream>

#include "antlr4-runtime.h"
#include "HATPLexer.h"
#include "HATPParser.h"
#include "HATPParserVisitor.h"
#include "procedural/reader/HATPListener.h"


namespace procedural {

class DomainReader
{
public:
    DomainReader();
    void read(std::string path);
    std::vector<MethodParsed_t> getMethods(){return htn_.methods;};
    std::vector<PrimitiveActionParsed_t> getActions(){return htn_.actions;};
private:
    HTNParserd_t htn_;
    antlr4::tree::ParseTree* tree;
    HATPListener listener;

};

} // procedural

#endif //PROCEDURAL_DOMAINREADER_H
