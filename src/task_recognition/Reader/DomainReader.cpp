#include "procedural/task_recognition/Reader/DomainReader.h"
#include "procedural/utils/Logger.h"


using namespace antlr4;


namespace procedural {

DomainReader::DomainReader(const std::string& path)
{
    read(path);
}
bool DomainReader::read(const std::string& path)
{
    std::cout  << "Reading domain file: " << path << std::endl;
    std::ifstream stream;
    stream.open(path);
    ANTLRInputStream input(stream);
    HATPLexer lexer(&input);
    CommonTokenStream tokens(&lexer);

    HATPParser parser(&tokens);
    tree = parser.hatp();

    tree::ParseTreeWalker::DEFAULT.walk(&listener, tree);
    htn_ = listener.getHTN();
    LOG_INFO << "HTN parsed";
//    for(auto action : htn_.actions)
//    {
//        LOG_INFO << "Action name: " << action.name;
////        for(auto arg : action.arguments)
////        {
////            LOG_INFO << "Argument: " << arg.first << " " << arg.second;
////        }
////        for(auto pre : action.preconditions)
////        {
////            LOG_INFO << "Precondition: " << pre.subject << " " << pre.op << " " << pre.object;
////        }
////        for(auto eff : action.effects.simple_effects)
////        {
////            LOG_INFO << "Effect: " << eff.subject << " " << eff.op << " " << eff.object;
////        }
////        for(auto eff : action.effects.other_effects)
////        {
////            LOG_INFO << "Effect: " << eff;
////        }
//    }
    return not htn_.empty();
}
} // procedural