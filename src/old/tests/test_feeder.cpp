#include "procedural/old/feeder/Feeder.h"
#include "procedural/old/reader/YamlReader.h"
#include "procedural/old/builder/ActionBuilder.h"
#include <iostream>
void test_pick(const procedural::Feeder& feeder)
{
    std::vector<std::string> str_facts;
    str_facts.emplace_back("[ADD]Bastien|MoveThrought|Cube");
//    str_facts.emplace_back("[DEL]Bastien|MoveThrought|Cube");
//    str_facts.emplace_back("[ADD]Bastien|MoveThrought|Cube");
//    feeder.feed()

}

int main()
{
    procedural::YamlReader reader = procedural::YamlReader();
    reader.read("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/src/reader/test.yaml");
    std::cout << "simple action : " << std::endl;
    for (auto& action: reader.getSimpleActions())
        std::cout << action << std::endl;
    std::cout << "composed action : " << std::endl;
    for (auto& action: reader.getComposedActions())
        std::cout << action << std::endl;

    procedural::ActionBuilder builder(reader.getSimpleActions(), reader.getComposedActions());
    auto Actions_ = builder.getActions();
    procedural::ActionRecognition recognition(Actions_);
    procedural::Feeder feeder;
    feeder.setCallback([recognition](procedural::Fact* fact){recognition.addToQueue(fact);});
    feeder.feed("[ADD]Bastien|hasInHand|cube1",0,{0,50});

    return 0;
}
