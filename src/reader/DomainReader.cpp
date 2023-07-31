#include "procedural/reader/DomainReader.h"



using namespace antlr4;


namespace procedural {
DomainReader::DomainReader()
{
    read("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/src/tests/reader/domaine_update.dom");
}
void DomainReader::read(std::string path)
{
    std::ifstream stream;
    stream.open(path);
    ANTLRInputStream input(stream);
    HATPLexer lexer(&input);
    CommonTokenStream tokens(&lexer);

    HATPParser parser(&tokens);
    tree = parser.hatp();

    tree::ParseTreeWalker::DEFAULT.walk(&listener, tree);

    htn_=listener.getHTN();
//    listener.displayParsedValue();

}
} // procedural