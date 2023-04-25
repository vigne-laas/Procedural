#include <iostream>
#include "procedural/reader/YamlReader.h"
#include "procedural/reader/ActionBuilder.h"

int main()
{
    procedural::YamlReader reader = procedural::YamlReader();
    reader.read("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/src/reader/exemple2.yaml");
//    std::cout << "simple action : " << std::endl;
    for (auto& action: reader.getSimpleActions())
        std::cout << action << std::endl;
//    std::cout << "composed action : " << std::endl;
    for (auto& action: reader.getComposedActions())
        std::cout << action << std::endl;

//    procedural::ActionBuilder builder(reader.getSimpleActions(),reader.getComposedActions());
//    builder.display();
    return 0;
}