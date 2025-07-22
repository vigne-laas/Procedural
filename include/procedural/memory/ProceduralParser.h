#ifndef PROCEDURAL_PROCEDURALPARSER_H
#define PROCEDURAL_PROCEDURALPARSER_H
#include <procedural_interfaces/action_t.h>
using namespace procedural_interfaces;
#include "RobotActionParserBaseListener.h"

namespace procedural {

class ProceduralParser: public RobotActionParserBaseListener {
public:
    void enterRoot(RobotActionParser::RootContext* ctx) override;
    Actions_t getActions() { return actions_; }
private:
    void parseActionBloc(RobotActionParser::Actions_blocContext* ctx);
    Action_t parseAction(RobotActionParser::ActionContext* ctx);
    std::vector<Argument_t> parseArguments(std::vector<RobotActionParser::ArgumentsContext*> ctx);
    std::vector<Execution_action_t> parseExecutionBloc(RobotActionParser::Execution_blocContext* ctx);
    Execution_action_t parseExecutionAction(RobotActionParser::Exec_actionContext* ctx);
    Execution_argument_t parseExecutionArgument(RobotActionParser::Exec_action_argContext* ctx);
    Description_t parseDescriptionBloc(RobotActionParser::Description_blocContext* ctx);
    Triplet_t parseTriplet(RobotActionParser::TripletContext* ctx);
    TripletVariable_t parseVariable(RobotActionParser::SubjectContext* ctx);
    TripletVariable_t parseVariable(RobotActionParser::ObjectContext* ctx);

    Actions_t actions_;

};

} // procedural

#endif //PROCEDURAL_PROCEDURALPARSER_H
