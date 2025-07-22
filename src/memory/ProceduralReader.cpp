#include "procedural/memory/ProceduralReader.h"

#include <ANTLRInputStream.h>
#include <iostream>
#include <fstream>
#include "RobotActionLexer.h"
#include "RobotActionParser.h"

using namespace antlr4;

namespace procedural {
ProceduralReader::ProceduralReader(const std::string& path)
{
    read(path);
}

bool ProceduralReader::read(const std::string& path)
{
    std::cout << "Reading procedural file: " << path << std::endl;
    std::ifstream stream;
    stream.open(path);
    ANTLRInputStream input(stream);
    RobotActionLexer lexer(&input);
    CommonTokenStream tokens(&lexer);

    RobotActionParser parser(&tokens);
    tree = parser.root();
    tree::ParseTreeWalker::DEFAULT.walk(&listener, tree);
    memory_ = listener.getActions().actions;
    std::cout << "Procedural parsed" << std::endl;
    std::cout << listener.getActions() << std::endl;

    return !memory_.empty();
}
} // procedural
