
#include "procedural/memory/ProceduralFullReader.h"
#include <ANTLRInputStream.h>
#include <iostream>
#include <fstream>
#include "ExtentedHATPLexer.h"
#include "ExtentedHATPParser.h"

using namespace antlr4;


namespace procedural {
ProceduralFullReader::ProceduralFullReader(const std::string &path)
{
    read(path);
}
bool ProceduralFullReader::read(const std::string &path)
{
    std::cout << "Reading procedural file: " << path << std::endl;
    std::ifstream stream;
    stream.open(path);
    if (!stream.is_open())
    {
        std::cerr << "Error opening file: " << path << std::endl;
        return false;
    }
    ANTLRInputStream input(stream);
    ExtentedHATPLexer lexer(&input);
    CommonTokenStream tokens(&lexer);
    ExtentedHATPParser parser(&tokens);
    tree = parser.root();
    tree::ParseTreeWalker::DEFAULT.walk(&parser_, tree);
    std::cout << "Procedural full parsed" << std::endl;
    return true;
}



} // procedural