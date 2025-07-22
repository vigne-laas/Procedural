#include "procedural/memory/ProceduralParser.h"

namespace procedural {
void ProceduralParser::enterRoot(RobotActionParser::RootContext* ctx)
{
    for (auto* const action_bloc: ctx->actions_bloc())
    {
        parseActionBloc(action_bloc);
    }
}
void ProceduralParser::parseActionBloc(RobotActionParser::Actions_blocContext* action_bloc)
{
    for (auto* const action: action_bloc->action())
    {
        actions_.actions.push_back(parseAction(action));
    }
    std::cout << "Parsed action bloc with " << actions_.actions.size() << " actions." << std::endl;
}
Action_t ProceduralParser::parseAction(RobotActionParser::ActionContext* action)
{
    Action_t new_action;
    new_action.name = action->name()->getText();
    new_action.arguments = parseArguments(action->arguments());
    new_action.executions_bloc = parseExecutionBloc(action->execution_bloc());
    new_action.description = parseDescriptionBloc(action->description_bloc());
    return new_action;
}


std::vector<Argument_t> ProceduralParser::parseArguments(std::vector<RobotActionParser::ArgumentsContext*> args)
{
    std::vector<Argument_t> arguments;
    for (auto* const arg: args)
    {
        Argument_t new_arg;
        new_arg.type = arg->type()->getText();
        new_arg.literal = arg->varname()->getText();
        arguments.push_back(new_arg);
    }
    return arguments;
}
std::vector<Execution_action_t> ProceduralParser::parseExecutionBloc(RobotActionParser::Execution_blocContext* bloc)
{
    std::vector<Execution_action_t> exec_actions;
    for (auto* const execution_action: bloc->exec_action())
    {
        Execution_action_t new_execution_action = parseExecutionAction(execution_action);
        exec_actions.push_back(new_execution_action);
    }
    return exec_actions;
}
Execution_action_t ProceduralParser::parseExecutionAction(RobotActionParser::Exec_actionContext* execution_action)
{
    Execution_action_t new_execution_action;
    new_execution_action.name = execution_action->name()->getText();
    for (auto* const arg: execution_action->exec_action_arg())
    {
        Execution_argument_t new_execution_arg = parseExecutionArgument(arg);
        new_execution_action.arguments.push_back(new_execution_arg);
    }
    return new_execution_action;
}
Execution_argument_t ProceduralParser::parseExecutionArgument(RobotActionParser::Exec_action_argContext* arg)
{
    Execution_argument_t new_execution_arg;
    if (arg->arg() != nullptr)
    {
        new_execution_arg.type = "arg";
        new_execution_arg.value = arg->arg()->getText();
    } else if (arg->topic_name() != nullptr)
    {
        new_execution_arg.type = "topic_name";
        new_execution_arg.value = arg->topic_name()->getText();
    } else if (arg->json_struct() != nullptr)
    {
        new_execution_arg.type = "json_struct";
        for (auto* const json: arg->json_struct()->json_pair())
        {
            new_execution_arg.json[json->varname()->getText()] = json->value()->getText();
        }
    }
    return new_execution_arg;
}
Description_t ProceduralParser::parseDescriptionBloc(RobotActionParser::Description_blocContext* ctx)
{
    Description_t new_description;
    if (ctx == nullptr)
        return new_description;
    for (auto const descriptions: ctx->triplet())
    {
        Triplet_t new_triplet = parseTriplet(descriptions);
        new_description.description.push_back(new_triplet);
    }
    return new_description;
}
Triplet_t ProceduralParser::parseTriplet(RobotActionParser::TripletContext* ctx)
{
    Triplet_t new_triplet;
    new_triplet.add = ctx->NOT() == nullptr;
    new_triplet.required = ctx->REQUIRED() != nullptr;
    new_triplet.subject = parseVariable(ctx->subject());
    new_triplet.property = ctx->predicate()->getText();
    new_triplet.object = parseVariable(ctx->object());
    return new_triplet;
}
TripletVariable_t ProceduralParser::parseVariable(RobotActionParser::SubjectContext* ctx)
{
    TripletVariable_t new_variable;
    // std::cout << "Parsing variable" << std::endl;
    // std::cout << "ctx->getText() : " << ctx->getText() << std::endl;
    if (ctx->my_self_var() != nullptr || ctx->variable() != nullptr)
    {
        new_variable.isVariable = true;
        if (ctx->my_self_var() != nullptr)
        {
            new_variable.literal = "action_id";
        } else
        {
            new_variable.literal = ctx->variable()->getText();
        }
    } else
    {
        new_variable.literal = ctx->literal()->getText();
    }
    // std::cout << "[Subject] new_variable : " << new_variable << std::endl;
    return new_variable;
}
TripletVariable_t ProceduralParser::parseVariable(RobotActionParser::ObjectContext* ctx)
{
    TripletVariable_t new_variable;
    // std::cout << "Parsing variable" << std::endl;
    // std::cout << "ctx->getText() : " << ctx->getText() << std::endl;
    if (ctx->my_self_var() != nullptr || ctx->variable() != nullptr)
    {
        new_variable.isVariable = true;
        if (ctx->my_self_var() != nullptr)
        {
            new_variable.literal = "action_id";
        } else
        {
            new_variable.literal = ctx->variable()->getText();
        }
    } else
    {
        new_variable.literal = ctx->literal()->getText();
    }
    // std::cout << "[Object] new_variable : " << new_variable << std::endl;

    return new_variable;
}
} // procedural
