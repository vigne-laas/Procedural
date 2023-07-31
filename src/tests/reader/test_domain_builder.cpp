#include "procedural/reader/DomainReader.h"
#include "procedural/builder/HTNBuilder.h"
#include "procedural/reader/YamlReader.h"
#include "procedural/builder/ActionBuilder.h"


int main(int , const char **) {
    procedural::DomainReader domainReader;
    procedural::HTNBuilder builder;
    domainReader.getMethods();
    procedural::YamlReader reader = procedural::YamlReader();
    reader.read("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/src/reader/test_link_domain.yaml");
    procedural::ActionBuilder action_builder(reader.getSimpleActions(), reader.getComposedActions());
    auto actions_ = action_builder.getActions();
//    std::cout << std::endl;
    builder.buildTask(domainReader.getMethods(),domainReader.getActions(),action_builder.getActions());
    builder.displayActions();
    return 0;
}

