#include "procedural/task_recognition/Reader/DomainReader.h"


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
    return not htn_.empty();
}
} // procedural