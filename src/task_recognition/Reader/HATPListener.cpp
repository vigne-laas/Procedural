#include "procedural/task_recognition/Reader/HATPListener.h"
#include "procedural/utils/Logger.h"
namespace procedural {
void HATPListener::enterHtn(HATPParser::HtnContext* ctx)
{
    auto actions = ctx->actions();

    for (auto action: actions)
    {
        PrimitiveActionParsed_t action_;
        action_.name = action->action_name()->getText();
        std::cout << "Action name: " << action->action_name()->getText() << std::endl;
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

        // Parse COMMITMENTS block if present
        if (action->commitments() != nullptr)
        {
            action_.commitments.has_commitments = true;
            auto commitments = action->commitments();

            // Get the raw text content of commitments
            std::string commitment_text = commitments->getText();

            // Simple text-based parsing for now (can be improved later)
            // Parse INSTRUMENTAL conditions
            size_t instrumental_pos = commitment_text.find("INSTRUMENTAL{");
            if (instrumental_pos != std::string::npos)
            {
                size_t start = commitment_text.find("{", instrumental_pos) + 1;
                size_t end = commitment_text.find("};", start);
                std::string instrumental_block = commitment_text.substr(start, end - start);

                // Extract all SELECT queries
                size_t pos = 0;
                while ((pos = instrumental_block.find("SELECT", pos)) != std::string::npos)
                {
                    size_t query_end = instrumental_block.find(";", pos);
                    if (query_end != std::string::npos)
                    {
                        CommitmentCondition_t cond;
                        cond.sparql_query = instrumental_block.substr(pos, query_end - pos + 1);
                        action_.commitments.instrumental.push_back(cond);
                        pos = query_end + 1;
                    }
                    else
                    {
                        break;
                    }
                }
            }

            // Parse ENGAGEMENT conditions
            size_t engagement_pos = commitment_text.find("ENGAGEMENT{");
            if (engagement_pos != std::string::npos)
            {
                size_t start = commitment_text.find("{", engagement_pos) + 1;
                size_t end = commitment_text.find("};", start);
                std::string engagement_block = commitment_text.substr(start, end - start);

                size_t pos = 0;
                while ((pos = engagement_block.find("SELECT", pos)) != std::string::npos)
                {
                    size_t query_end = engagement_block.find(";", pos);
                    if (query_end != std::string::npos)
                    {
                        CommitmentCondition_t cond;
                        cond.sparql_query = engagement_block.substr(pos, query_end - pos + 1);
                        action_.commitments.engagement.push_back(cond);
                        pos = query_end + 1;
                    }
                    else
                    {
                        break;
                    }
                }
            }

            // Parse COMMON_GROUND conditions
            size_t cg_pos = commitment_text.find("COMMON_GROUND{");
            if (cg_pos != std::string::npos)
            {
                size_t start = commitment_text.find("{", cg_pos) + 1;
                size_t end = commitment_text.find("};", start);
                std::string cg_block = commitment_text.substr(start, end - start);

                size_t pos = 0;
                while ((pos = cg_block.find("SELECT", pos)) != std::string::npos)
                {
                    size_t query_end = cg_block.find(";", pos);
                    if (query_end != std::string::npos)
                    {
                        CommitmentCondition_t cond;
                        cond.sparql_query = cg_block.substr(pos, query_end - pos + 1);
                        action_.commitments.common_ground.push_back(cond);
                        pos = query_end + 1;
                    }
                    else
                    {
                        break;
                    }
                }
            }

            // Parse reaction mappings - extract text between quotes
            auto extractQuotedValue = [](const std::string& text, const std::string& prefix) -> std::string {
                size_t pos = text.find(prefix);
                if (pos != std::string::npos)
                {
                    size_t quote1 = text.find("\"", pos);
                    if (quote1 != std::string::npos)
                    {
                        size_t quote2 = text.find("\"", quote1 + 1);
                        if (quote2 != std::string::npos)
                        {
                            return text.substr(quote1 + 1, quote2 - quote1 - 1);
                        }
                    }
                }
                return "";
            };

            action_.commitments.on_instrumental_failure = extractQuotedValue(commitment_text, "ON_INSTRUMENTAL_FAILURE:");
            action_.commitments.on_engagement_failure = extractQuotedValue(commitment_text, "ON_ENGAGEMENT_FAILURE:");
            action_.commitments.on_common_ground_failure = extractQuotedValue(commitment_text, "ON_COMMON_GROUND_FAILURE:");

            // Parse recovery strategy
            size_t recovery_pos = commitment_text.find("RECOVERY_STRATEGY{");
            if (recovery_pos != std::string::npos)
            {
                size_t mode_pos = commitment_text.find("MODE:", recovery_pos);
                if (mode_pos != std::string::npos)
                {
                    action_.commitments.recovery_strategy.mode = extractQuotedValue(commitment_text, "MODE:");
                }

                size_t attempts_pos = commitment_text.find("MAX_ATTEMPTS:", recovery_pos);
                if (attempts_pos != std::string::npos)
                {
                    size_t num_start = commitment_text.find(":", attempts_pos) + 1;
                    size_t num_end = commitment_text.find(";", num_start);
                    std::string num_str = commitment_text.substr(num_start, num_end - num_start);
                    // Trim whitespace
                    num_str.erase(0, num_str.find_first_not_of(" \t\n\r"));
                    num_str.erase(num_str.find_last_not_of(" \t\n\r;") + 1);
                    action_.commitments.recovery_strategy.max_attempts = std::stoi(num_str);
                }

                size_t timeout_pos = commitment_text.find("TIMEOUT:", recovery_pos);
                if (timeout_pos != std::string::npos)
                {
                    size_t num_start = commitment_text.find(":", timeout_pos) + 1;
                    size_t num_end = commitment_text.find(";", num_start);
                    std::string num_str = commitment_text.substr(num_start, num_end - num_start);
                    // Trim whitespace
                    num_str.erase(0, num_str.find_first_not_of(" \t\n\r"));
                    num_str.erase(num_str.find_last_not_of(" \t\n\r;") + 1);
                    action_.commitments.recovery_strategy.timeout = std::stod(num_str);
                }
            }

            std::cout << "  Commitments parsed successfully" << std::endl;
        }

        htn_.actions.push_back(action_);
    }

    auto tasks = ctx->tasks();
//    for(auto method : methods)
//        std::cout << method->getText() << std::endl;
    for (auto task: tasks)
    {
        Abstract_task_t method_;
        method_.name = task->IDENTIFIER()->getText();
        for (auto goal: task->goal()->expression())
            method_.goals.emplace_back(goal->subject()->getText(), goal->operator_()->getText(),
                                       goal->object()->getText());
        for (auto arg: task->arguments())
            method_.arguments.emplace_back(arg->type()->getText(), arg->varname()->getText());
        for (auto decomposition: task->decomposition())
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
                    action_.after_id.insert(std::stoi(order->NUMBER()->getText()));
                decomposition_.subtask.map_actions[action_.id] = action_;
            }
//            LOG_INFO << "Decomposition: " << decomposition_;
            method_.methods_.push_back(decomposition_);
        }
        htn_.tasks.push_back(method_);


    }
//    std::cout << htn_ << std::endl;

}

} // procedural