#ifndef PROCEDURAL_DOMAINREADER_H
#define PROCEDURAL_DOMAINREADER_H
#include <iostream>

#include "antlr4-runtime.h"
#include "HATPLexer.h"
#include "HATPParser.h"
#include "HATPParserVisitor.h"
#include "procedural/task_recognition/Reader/HATPListener.h"
//#include "procedural/utils/Logger.h"


namespace procedural {

class DomainReader
{
public:
    DomainReader() = default;
    explicit DomainReader(const std::string& path);
    bool read(const std::string& path);
    std::vector<Abstract_task_t>& getTasks() { return htn_.tasks; }
    std::vector<PrimitiveActionParsed_t> getActions() { return htn_.actions; }
private:
    HTNParserd_t htn_;
    antlr4::tree::ParseTree* tree{};
    HATPListener listener;

};

} // procedural

#endif //PROCEDURAL_DOMAINREADER_H
