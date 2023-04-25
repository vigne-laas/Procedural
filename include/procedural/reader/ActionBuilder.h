#ifndef PROCEDURAL_ACTIONBUILDER_H
#define PROCEDURAL_ACTIONBUILDER_H

#include <vector>
#include "procedural/reader/types/ParsedSimpleAction.h"
#include "procedural/reader/types/ParsedComposedAction.h"
#include "procedural/core/Types/Action.h"
namespace procedural {

class ActionBuilder
{
public:
//    ActionBuilder();
    ActionBuilder(const std::vector<ParsedSimpleAction_t>& simple_actions, std::vector<ParsedComposedAction_t>& composed_actions);

    void buildSimpleAction(const std::vector<ParsedSimpleAction_t>& simple_actions);
    void buildComposedAction(std::vector<ParsedComposedAction_t>& composed_actions);
    void display();
private:
    std::vector<SpecializedAction*> actions_;
    bool completeRemap(SubNetwork_t& network);
};

} // procedural

#endif //PROCEDURAL_ACTIONBUILDER_H
