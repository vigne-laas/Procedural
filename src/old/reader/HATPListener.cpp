#include "procedural/old/reader/HATPListener.h"

namespace procedural {
void HATPListener::enterHtn(HATPParser::HtnContext* ctx)
{
    auto actions = ctx->actions();

    for (auto action: actions)
    {
        PrimitiveActionParsed_t action_;
        action_.name = action->action_name()->getText();
        for (auto arg: action->arguments())
            action_.arguments.emplace_back(arg->type()->getText(), arg->varname()->getText());
        for (auto precondition: action->preconditions())
            for (auto expression: precondition->expression())
                action_.preconditions.emplace_back(expression->subject()->getText(), expression->operator_()->getText(),
                                                   expression->object()->getText());
        for (auto effect: action->effects())
        {
            for (auto expression: effect->expression())
                action_.effects.simple_effects.emplace_back(expression->subject()->getText(),
                                                            expression->operator_()->getText(),
                                                            expression->object()->getText());
            for (auto forall: effect->forall())
                action_.effects.other_effects.emplace_back(forall->getText());
        }
        htn_.actions.push_back(action_);
    }

    auto methods = ctx->methods();
//    for(auto method : methods)
//        std::cout << method->getText() << std::endl;
    for (auto method: methods)
    {
        Abstract_task_t method_;
        method_.name = method->IDENTIFIER()->getText();
        for (auto goal: method->goal()->expression())
            method_.goals.emplace_back(goal->subject()->getText(), goal->operator_()->getText(),
                                       goal->object()->getText());
        for (auto arg: method->arguments())
            method_.arguments.emplace_back(arg->type()->getText(), arg->varname()->getText());
        for (auto decomposition: method->decomposition())
        {
            Method_t decomposition_;
            for (auto expression: decomposition->preconditions()->expression())
                decomposition_.preconditions.emplace_back(expression->subject()->getText(),
                                                          expression->operator_()->getText(),
                                                          expression->object()->getText());
            for (auto select: decomposition->subtask()->subselection())
            {
                auto expression = select->selectcase()->expression();
                Expression_t expressionParsed;
                if (expression != nullptr)
                {
                    expressionParsed.object = expression->object()->getText();
                    expressionParsed.property = expression->operator_()->getText();
                    expressionParsed.subject = expression->subject()->getText();
                }

                decomposition_.subtask.selections.emplace_back(select->attribut()->getText(),
                                                               select->selectcase()->IDENTIFIER()->getText(),
                                                               expressionParsed);
            }
            for (auto action: decomposition->subtask()->list())
            {
                Ordered_Action_t action_;
                action_.name = action->function()->IDENTIFIER()->getText();
                action_.id = std::stoi(action->NUMBER()->getText());
                for (const auto& name: action->function()->varname())
                    action_.arguments.push_back(name->getText());
                for (const auto& order: action->order())
                    action_.after_id.push_back(std::stoi(order->NUMBER()->getText()));
                decomposition_.subtask.actions_.push_back(action_);
            }
            method_.methods_.push_back(decomposition_);
        }
        htn_.methods.push_back(method_);


    }
//    std::cout << htn_ << std::endl;

}

} // procedural