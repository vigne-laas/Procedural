    #include "procedural/memory/FullParser.h"

#include <ExtentedHATPParser.h>
#include <procedural_interfaces/action_t.h>



namespace procedural {
void FullParser::enterRoot(ExtentedHATPParser::RootContext* ctx)
{
    std::cout << "Entering root context" << std::endl;
    for (auto* const inclusion: ctx->include_bloc())
    {
        std::cout << "Parsing include_bloc" << std::endl;
        for (auto* const include: inclusion->inclusion())
        {
            if (include->link() != nullptr)
            {
                std::cout << "Link: " << include->link()->getText() << std::endl;
            }
            if (include->package_link() != nullptr)
            {
                std::cout << "Package Link: " << include->package_link()->link()->getText() << std::endl;

            }
        }

        // actions_.actions.push_back(parseAction(action));
    }
    for (auto* const action_bloc: ctx->actions_bloc())
    {
        std::cout << "Parsing actions_bloc" << std::endl;
        parseActionBloc(action_bloc);
    }

    for (auto* const frames: ctx->pratices_frames_bloc())
    {
        std::cout << "Parsing pratices_frames_bloc" << std::endl;
        for (auto* const frame: frames->practice_frame())
        {
            std::cout << "Parsing frame_action: " << frame->getText() << std::endl;
            practice_frames_.push_back(parsePracticeFrame(frame));
        }
    }

    for (auto* const practices: ctx->practices_bloc())
    {
        std::cout << "Parsing practices_bloc" << std::endl;
        for (auto* const practice_item: practices->practice())
        {
            std::cout << "Parsing practice_action: " << practice_item->getText() << std::endl;
            practices_.insert(
                {practice_item->name()->getText(), parsePractice(practice_item)});
            // actions_.actions.push_back(parseAction(practice_action->action()));
        }
    }
    std::cout << "Finished parsing root context." << std::endl;
    linkPracticesToFrames();


    displayResult();
    std::cout << "Parsed " << actions_.actions.size() << " actions." << std::endl;
    std::cout << "FullParser finished parsing." << std::endl;
}


void FullParser::displayResult()
{
    std::cout << "Displaying parsed actions:" << std::endl;
    std::cout << actions_ << std::endl;
    std::cout << "Total actions parsed: " << actions_.actions.size() << std::endl;

    std::cout << "Total practice frames parsed: " << practice_frames_.size() << std::endl;
    for (const auto& frame: practice_frames_)
    {
        std::cout << "Frame: " << frame->name << std::endl;
        std::cout << "Description: " << frame->description << std::endl;
        std::cout << "Practices: " << frame->practices.size() << std::endl;
        for (const auto& practice: frame->practices)
        {
            std::cout << " - Practice: " << practice.name << std::endl;
            std::cout << "   Description: " << practice.description << std::endl;
        }
    }
    std::cout << "\n\nTotal practices parsed: " << practices_.size() << std::endl;
    for (const auto& [name, practice]: practices_)
    {
        std::cout << "Practice: " << name << std::endl;
        std::cout << "Description: " << practice->description << std::endl;
        std::cout << "Competences: " << practice->competences.size() << std::endl;
        for (const auto& competence: practice->competences)
        {
            std::cout << " - Competence: " << competence << std::endl;
        }
        std::cout << "Objects: " << practice->objects.size() << std::endl;
        for (const auto& object: practice->objects)
        {
            std::cout << " - Object: " << object << std::endl;
        }
        std::cout << "Activation Conditions: " << practice->activation_conditions.size() << std::endl;
        for (const auto& condition: practice->activation_conditions)
        {
            std::cout << " - Condition: " << condition << std::endl;
        }
        std::cout << "Roles: " << practice->roles.size() << std::endl;
        for (const auto& role: practice->roles)
        {
            std::cout << " - Role: " << role << std::endl;
        }
        std::cout << "Rules: " << practice->rules.size() << std::endl;
        for (const auto& rule: practice->rules)
        {
            std::cout << " - Rule: " << rule << std::endl;
        }
        std::cout <<"\n\n\n" <<std::endl;
    }
}
void FullParser::linkPracticesToFrames()
{
    for (auto* const frame: practice_frames_)
    {
        std::cout << "Linking practices to frame: " << frame->name << std::endl;
        for (auto& practice : frame->practices)
        {
            std::cout << "Linking practice: " << practice.name << std::endl;
            auto it = practices_.find(practice.name);
            if (it != practices_.end())
            {
                // Remplace la pratique incomplète par la pratique complète
                practice = *(it->second);
                std::cout << "Successfully linked practice: " << practice.name << std::endl;
            }
            else
            {
                std::cout << "Warning: Practice '" << practice.name << "' not found in practices map" << std::endl;
            }
        }
    }
}
void FullParser::parseActionBloc(ExtentedHATPParser::Actions_blocContext* action_bloc)
{
    for (auto* const action: action_bloc->action())
    {
        // std::cout << "Parsing action: " << action->getText() << std::endl;
        actions_.actions.push_back(parseAction(action));
    }
}
std::string FullParser::parseDescriptionPracticeBloc(
    const std::vector<ExtentedHATPParser::Description_practiceContext*>& vector)
{
    std::string description;
    for (auto* const description_practice: vector)
    {
        if (description_practice->sentence()!= nullptr)
        {
            description += description_practice->sentence()->getText() + "\n";
        }
    }
    if (description.empty())
    {
        return "No description provided";
    }
    return description;
}
Practice* FullParser::parsePractice(ExtentedHATPParser::PracticeContext* practice_context)
{
    Practice* new_practice = new Practice();
    new_practice->name = practice_context->name()->getText();
    new_practice->description = parseDescriptionPracticeBloc(practice_context->description_practice());
    for (auto* const competences_context: practice_context->competences())
    {
        for (auto* const competence: competences_context->competence())
        {
            // std::cout << "Adding competence: " << competence->sentence()->getText() << std::endl;
            new_practice->competences.push_back(competence->sentence()->getText());
        }

    }

    for (auto* const object_ctx : practice_context->objects_bloc())
    {
        for (auto* const object_item: object_ctx->object_item())
        {
            // std::cout << "Adding object: " << object_item->object()->getText() << std::endl;
            new_practice->objects.push_back(object_item->object()->getText());
        }

    }
    for (auto* const condition_ctx: practice_context->conditions_practices())
    {
        // std::cout << "Adding condition: " << condition_ctx->query()->getText() << std::endl;
        new_practice->activation_conditions.push_back(condition_ctx->query()->getText());
    }
    for (auto* const roles_ctx: practice_context->roles_list())
    {
        for (auto* const role: roles_ctx->role_name())
        {
            // std::cout << "Adding role: " << role->name()->getText() << std::endl;
            new_practice->roles.push_back(role->name()->getText());
        }
    }
    for (auto* const rules_ctx: practice_context->rules_bloc())
    {
        for (auto* const rule: rules_ctx->rule_item())
        {
            // std::cout << "Adding rule: " << rule->sentence()->getText() << std::endl;
            new_practice->rules.push_back(rule->sentence()->getText());
        }
    }
    return new_practice;
}
PracticeFrame* FullParser::parsePracticeFrame(ExtentedHATPParser::Practice_frameContext* frame)
{
    PracticeFrame* new_frame = new PracticeFrame();
    new_frame->name = frame->name()->getText();
    new_frame->description = parseDescriptionPracticeBloc(frame->description_practice());
    for (auto* const practice_ctx: frame->practices_list())
    {
        for (auto* const practice: practice_ctx->practice_name())
        {
            // std::cout << "Adding practice: " << practice->name()->getText() << std::endl;
            Practice * temp = new Practice();
            temp->name = practice->name()->getText();
            new_frame->practices.push_back(*temp);

        }
    }

    for (auto* const roles_ctx: frame->roles_list())
    {
        for (auto* const role: roles_ctx->role_name())
        {
            // std::cout << "Adding role: " << role->name()->getText() << std::endl;
            new_frame->roles.push_back(role->name()->getText());
        }
    }
    for (auto* const object_ctx: frame->objects_bloc())
    {
        for (auto* const object_item: object_ctx->object_item())
        {
            // std::cout << "Adding object: " << object_item->object()->getText() << std::endl;
            new_frame->objects.push_back(object_item->object()->getText());
        }
    }
    for (auto* const rules_ctx: frame->rules_bloc())
    {
        for (auto* const rule: rules_ctx->rule_item())
        {
            // std::cout << "Adding rule: " << rule->sentence()->getText() << std::endl;
            new_frame->rules.push_back(rule->sentence()->getText());
        }
    }

    return new_frame;
}
Action_t FullParser::parseAction(ExtentedHATPParser::ActionContext* action)
{
    Action_t new_action;
    new_action.name = action->name()->getText();
    new_action.arguments = parseArguments(action->arguments());
    for (auto* exec_bloc: action->execution_bloc())
    {
        new_action.executions_bloc = parseExecutionBloc(exec_bloc);
    }
    for (auto* const description_bloc: action->description_bloc())
    {
        new_action.description = parseDescriptionBloc(description_bloc);
    }
    return new_action;
}


std::vector<Argument_t> FullParser::parseArguments(std::vector<ExtentedHATPParser::ArgumentsContext*> args)
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
std::vector<Execution_action_t> FullParser::parseExecutionBloc(ExtentedHATPParser::Execution_blocContext* bloc)
{
    if (bloc == nullptr)
    {
        // std::cout << "No execution bloc found" << std::endl;
        return {};
    }
    std::vector<Execution_action_t> exec_actions;
    for (auto* const execution_action: bloc->exec_action())
    {
        Execution_action_t new_execution_action = parseExecutionAction(execution_action);
        exec_actions.push_back(new_execution_action);
    }
    return exec_actions;
}
Execution_action_t FullParser::parseExecutionAction(ExtentedHATPParser::Exec_actionContext* execution_action)
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
Execution_argument_t FullParser::parseExecutionArgument(ExtentedHATPParser::Exec_action_argContext* arg)
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
Description_t FullParser::parseDescriptionBloc(ExtentedHATPParser::Description_blocContext* ctx)
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
Triplet_t FullParser::parseTriplet(ExtentedHATPParser::TripletContext* ctx)
{
    Triplet_t new_triplet;
    new_triplet.add = ctx->NOT() == nullptr;
    new_triplet.required = ctx->REQUIRED() != nullptr;
    new_triplet.subject = parseVariable(ctx->subject());
    new_triplet.property = ctx->predicate()->getText();
    new_triplet.object = parseVariable(ctx->object());
    return new_triplet;
}
TripletVariable_t FullParser::parseVariable(ExtentedHATPParser::SubjectContext* ctx)
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
TripletVariable_t FullParser::parseVariable(ExtentedHATPParser::ObjectContext* ctx)
{
    procedural_interfaces::TripletVariable_t new_variable;
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
