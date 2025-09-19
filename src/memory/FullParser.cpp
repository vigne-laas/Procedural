    #include "procedural/memory/FullParser.h"

#include <ExtentedHATPParser.h>
#include <procedural_interfaces/action_t.h>
#include <algorithm>
#include <cctype>



namespace procedural {

// Utility function to trim whitespace from strings
std::string trim(const std::string& str) {
    const auto start = str.find_first_not_of(" \t\n\r");
    if (start == std::string::npos) return "";
    const auto end = str.find_last_not_of(" \t\n\r");
    return str.substr(start, end - start + 1);
}

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
            // std::cout << "Parsing frame_action: " << frame->getText() << std::endl;
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

    // Note: Global ATTENTES parsing is deprecated. 
    // Role expectations are now parsed at the practice level.
    // for (auto* const attentes_bloc: ctx->attentes_bloc())
    // {
    //     std::cout << "Warning: Global ATTENTES bloc is deprecated. Role expectations should be defined at practice level." << std::endl;
    //     for (auto* const role: attentes_bloc->role())
    //     {
    //         std::cout << "Parsing legacy role: " << role->getText() << std::endl;
    //         roles_.insert(std::make_pair(role->name()->getText(),parseRole(role)));
    //     }
    // }

    for (auto* const priorities_bloc: ctx->priorities_bloc())
    {
        std::cout << "Parsing priorities_bloc" << std::endl;
        for (auto* const priority: priorities_bloc->priority())
        {
            std::cout << "Parsing priority: " << priority->name()->getText() << std::endl;
            priorities_.insert(std::make_pair(priority->name()->getText(), parsePriority(priority)));
        }
    }

    for (auto* const tasks_bloc: ctx->tasks_bloc())
    {
        std::cout << "Parsing tasks_bloc" << std::endl;
        parseTasksBloc(tasks_bloc);
    }

    std::cout << "Finished parsing root context." << std::endl;
    
    // std::cout << "\n=================================================================" << std::endl;
    // std::cout << "PHASE 1: PARSING COMPLETE" << std::endl;
    // std::cout << "=================================================================" << std::endl;
    // debugPrintRolesState("BEFORE_LINKING");
    
    // std::cout << "\n=================================================================" << std::endl;
    // std::cout << "PHASE 2: STARTING ROLE LINKING" << std::endl;
    // std::cout << "=================================================================" << std::endl;
    linkRolesToPractices();
    // debugPrintRolesState("AFTER_ROLES_TO_PRACTICES");
    
    // std::cout << "\n=================================================================" << std::endl;
    // std::cout << "PHASE 3: CONTINUING WITH OTHER LINKING" << std::endl;
    // std::cout << "=================================================================" << std::endl;
    linkRolesToFrames();
    linkPracticesToFrames();
    
    // std::cout << "\n=================================================================" << std::endl;
    // std::cout << "PHASE 4: ALL LINKING COMPLETE" << std::endl;
    // std::cout << "=================================================================" << std::endl;
    // debugPrintRolesState("FINAL_STATE");

    std::cout << "\n=================================================================" << std::endl;
    std::cout << "PARSING SUMMARY" << std::endl;
    std::cout << "=================================================================" << std::endl;
    displayResult();
    std::cout << "Parsed " << actions_.actions.size() << " actions." << std::endl;
    std::cout << "Parsed " << practice_frames_.size() << " practice frames." << std::endl;
    std::cout << "Parsed " << practices_.size() << " practices." << std::endl;
    std::cout << "Parsed " << roles_.size() << " legacy roles." << std::endl;
    std::cout << "Parsed " << priorities_.size() << " priorities." << std::endl;
    std::cout << "=================================================================" << std::endl;
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
    std::cout << "Total roles parsed: " << roles_.size() << std::endl;
    for (const auto& role: roles_)
    {
        // std::cout << "Role: " << role->role_name << std::endl;
        // std::cout << "Intents: " << role->attentes.size() << std::endl;
        // for (const auto& attente: role->attentes)
        // {
            // std::cout << " - Intent: " << attente.name << std::endl;
        // }
    }
    
    std::cout << "\n\nTotal priorities parsed: " << priorities_.size() << std::endl;
    for (const auto& [name, priority]: priorities_)
    {
        std::cout << "Priority: " << name << std::endl;
        std::cout << "Level: " << priority->level << std::endl;
        std::cout << "Preconditions: " << priority->preconditions.size() << std::endl;
        for (const auto& precondition: priority->preconditions)
        {
            std::cout << " - Precondition: " << precondition << std::endl;
        }
        std::cout << "Objectives: " << priority->objectives.size() << std::endl;
        for (const auto& objective: priority->objectives)
        {
            std::cout << " - Objective: " << objective << std::endl;
        }
        std::cout << "\n" << std::endl;
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
void FullParser::linkRolesToPractices()
{
    std::cout << "\n=== DEBUG: LINKING ROLES TO PRACTICES ===" << std::endl;
    std::cout << "Available frames: " << practice_frames_.size() << std::endl;
    for (auto* const frame: practice_frames_)
    {
        std::cout << "  Frame '" << frame->name << "' has " << frame->roles.size() << " roles: ";
        for (const auto& role : frame->roles)
        {
            std::cout << "[" << role.role_name << "] ";
        }
        std::cout << std::endl;
    }
    std::cout << "===============================================\n" << std::endl;
    
    for (auto& [name,practice]: practices_)
    {
        std::cout << "DEBUG: Linking roles to practice '" << name << "'" << std::endl;
        std::cout << "  Practice has " << practice->roles.size() << " roles to link" << std::endl;
        
        for (auto& practice_role: practice->roles)
        {
            std::cout << "  DEBUG: Processing role '" << practice_role.role_name << "'" << std::endl;
            std::cout << "    Practice role has " << practice_role.attentes.size() << " practice-specific attentes" << std::endl;
            
            // Find the corresponding role in practice frames
            Role* frame_role = nullptr;
            std::string searched_frames = "";
            for (auto* const frame: practice_frames_)
            {
                searched_frames += "[" + frame->name + "] ";
                for (auto& frame_r: frame->roles)
                {
                    if (trim(frame_r.role_name) == trim(practice_role.role_name))
                    {
                        frame_role = &frame_r;
                        std::cout << "    DEBUG: Found matching role in frame '" << frame->name 
                                  << "' with " << frame_r.capacites.size() << " capacites and " 
                                  << frame_r.attentes.size() << " frame attentes" << std::endl;
                        break;
                    }
                }
                if (frame_role != nullptr) break;
            }
            
            if (frame_role != nullptr)
            {
                // Store original practice-specific attentes
                std::vector<Attente> practice_attentes = practice_role.attentes;
                std::vector<Capacite> practice_capacites = practice_role.capacites;
                std::cout << "    DEBUG: Storing " << practice_attentes.size() << " practice-specific attentes before merge" << std::endl;
                std::cout << "    DEBUG: Storing " << practice_capacites.size() << " practice-specific capacites before merge" << std::endl;
                
                // Copy the complete role from the frame (includes CONDITIONS and CAPACITES)
                practice_role = *frame_role;
                std::cout << "    DEBUG: Copied frame role data (" << frame_role->capacites.size() 
                          << " capacites, " << frame_role->attentes.size() << " frame attentes)" << std::endl;
                
                // Merge attentes: keep frame attentes + add practice-specific attentes
                int added_attentes = 0;
                for (const auto& practice_attente : practice_attentes)
                {
                    // Check if this attente already exists in frame
                    bool exists = false;
                    for (const auto& frame_attente : practice_role.attentes)
                    {
                        if (frame_attente.name == practice_attente.name)
                        {
                            exists = true;
                            std::cout << "      DEBUG: Skipping duplicate attente '" << practice_attente.name << "'" << std::endl;
                            break;
                        }
                    }
                    
                    // If not exists in frame, add practice-specific attente
                    if (!exists)
                    {
                        practice_role.attentes.push_back(practice_attente);
                        added_attentes++;
                        std::cout << "      DEBUG: Added practice attente '" << practice_attente.name << "' (type: " << practice_attente.type << ")" << std::endl;
                    }
                }

                int added_capacites = 0;
                for (const auto& prapractice_capacite : practice_capacites)
                {
                    // Check if this capacite already exists in frame
                    bool exists = false;
                    for (const auto& frame_capacite : practice_role.capacites)
                    {
                        if (frame_capacite.name == prapractice_capacite.name)
                        {
                            exists = true;
                            std::cout << "      DEBUG: Skipping duplicate capacite '" << prapractice_capacite.name << "'" << std::endl;
                            break;
                        }
                    }
                    
                    // If not exists in frame, add practice-specific capacite
                    if (!exists)
                    {
                        practice_role.capacites.push_back(prapractice_capacite);
                        added_capacites++;
                        std::cout << "      DEBUG: Added practice capacite '" << prapractice_capacite.name << "'" << std::endl;
                    }
                }
                std::cout << "    DEBUG: Merged in " << added_capacites << " practice-specific capacites" << std::endl;
                
                std::cout << "    SUCCESS: Role '" << practice_role.role_name 
                         << "' linked with " << practice_role.capacites.size() << " capacites and " 
                         << practice_role.attentes.size() << " total attentes (+" << added_attentes << " from practice)" << std::endl;
            }
            else
            {
                std::cout << "    ERROR: Role '" << practice_role.role_name << "' not found in any frame!" << std::endl;
                std::cout << "    Searched in frames: " << searched_frames << std::endl;
                
                // Fallback to legacy role map for backward compatibility
                auto it = roles_.find(trim(practice_role.role_name));
                if (it != roles_.end())
                {
                    practice_role = *(it->second);
                    std::cout << "    FALLBACK: Used legacy map for role '" << practice_role.role_name << "'" << std::endl;
                }
                else
                {
                    std::cout << "    WARNING: Role '" << practice_role.role_name << "' not found anywhere! (legacy map size: " << roles_.size() << ")" << std::endl;
                }
            }
        }
        std::cout << "" << std::endl; // Empty line between practices
    }
}

void FullParser::debugPrintRolesState(const std::string& phase)
{
    std::cout << "\n========================= ROLES STATE DEBUG: " << phase << " =========================" << std::endl;
    
    std::cout << "\n--- PRACTICE FRAMES ---" << std::endl;
    for (const auto* frame : practice_frames_)
    {
        std::cout << "Frame '" << frame->name << "':" << std::endl;
        for (const auto& role : frame->roles)
        {
            std::cout << "  Role '" << role.role_name << "':" << std::endl;
            std::cout << "    Capacites: " << role.capacites.size() << std::endl;
            for (const auto& capacite : role.capacites)
            {
                std::cout << "      - " << capacite.name << " (expectations: " << capacite.can_satisfy_expectations.size() << ")" << std::endl;
            }
            std::cout << "    Attentes: " << role.attentes.size() << std::endl;
            for (const auto& attente : role.attentes)
            {
                std::cout << "      - " << attente.name << std::endl;
            }
        }
    }
    
    std::cout << "\n--- PRACTICES ---" << std::endl;
    for (const auto& [practice_name, practice] : practices_)
    {
        std::cout << "Practice '" << practice_name << "':" << std::endl;
        for (const auto& role : practice->roles)
        {
            std::cout << "  Role '" << role.role_name << "':" << std::endl;
            std::cout << "    Capacites: " << role.capacites.size() << std::endl;
            for (const auto& capacite : role.capacites)
            {
                std::cout << "      - " << capacite.name << " (expectations: " << capacite.can_satisfy_expectations.size() << ")" << std::endl;
            }
            std::cout << "    Attentes: " << role.attentes.size() << std::endl;
            for (const auto& attente : role.attentes)
            {
                std::cout << "      - " << attente.name << std::endl;
            }
        }
    }
    
    std::cout << "============================================================================\n" << std::endl;
}

void FullParser::linkRolesToFrames()
{
    for (auto* const frame: practice_frames_)
    {
        std::cout << "Linking roles to frame: " << frame->name << std::endl;
        for (auto& frame_role: frame->roles)
        {
            std::cout << "Linking role: " << frame_role.role_name << std::endl;
            auto it = roles_.find(trim(frame_role.role_name));
            if (it != roles_.end())
            {
                // Remplace la pratique incomplète par la pratique complète
                frame_role = *(it->second);
                std::cout << "Successfully linked role in frame: " << frame_role.role_name << std::endl;
            }
            else
            {
                std::cout << "Warning: Role '" << frame_role.role_name << "' not found in role map" << std::endl;
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
    // Handle roles with attentes (new structure)
    std::cout << "DEBUG: Parsing roles in PRACTICE '" << new_practice->name << "'" << std::endl;
    for (auto* const roles_ctx: practice_context->roles_with_attentes())
    {
        for (auto* const role_with_attente: roles_ctx->role_with_attente())
        {
            Role* new_role = new Role();
            new_role->role_name = trim(role_with_attente->name()->getText());
            std::cout << "  DEBUG: Found role '" << new_role->role_name << "' in practice (no capacites at this level)" << std::endl;
            
            // Parse attentes if present
            if (role_with_attente->attentes_list() != nullptr)
            {
                std::cout << "    DEBUG: Parsing practice-specific attentes for role '" << new_role->role_name << "'" << std::endl;
                for (auto* const attente: role_with_attente->attentes_list()->attente_extended())
                {
                    auto new_attente = parseAttenteExtended(attente);
                    new_role->attentes.push_back(new_attente);
                    std::cout << "      DEBUG: Added practice attente '" << new_attente.name << "' (type: " << new_attente.type << ")" << std::endl;
                }
            }

            if (role_with_attente->capacites_list() != nullptr)
            {
                std::cout << "    DEBUG: Parsing capacites for role '" << new_role->role_name << std::endl;
                for (auto* const capacite : role_with_attente->capacites_list()->capacite())
                {
                    auto temp_capacite = parseCapacite(capacite);
                    new_role->capacites.push_back(*temp_capacite);
                    std::cout << "      DEBUG: Added capacite '" << temp_capacite->name << "' (satisfies: " << temp_capacite->can_satisfy_expectations.size() << " expectations)" << std::endl;


                }
            }

            
            std::cout << "    DEBUG: Practice role '" << new_role->role_name << "' has " 
                      << new_role->attentes.size() << " practice-specific attentes " << "and " << new_role->capacites.size() << " nb of cap" << std::endl;
            new_practice->roles.push_back(*new_role);
        }
    }
    
    // Legacy support for old roles_list structure (if needed)
    // Note: roles_list() method not available in new grammar
    // Removed for compatibility with new grammar structure
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
    for (auto* const conditions: frame->conditions_practices())
    {
        new_frame->activation_conditions = conditions->query()->getText();
    }
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

    // Handle roles with conditions (new structure)
    std::cout << "DEBUG: Parsing roles in PRACTICE_FRAME '" << new_frame->name << "'" << std::endl;
    for (auto* const roles_ctx: frame->roles_with_conditions())
    {
        for (auto* const role_with_condition: roles_ctx->role_with_condition())
        {
            Role* new_role = new Role();
            new_role->role_name = trim(role_with_condition->name()->getText());
            std::cout << "  DEBUG: Found role '" << new_role->role_name << "' in frame" << std::endl;
            
            // Parse conditions if present
            if (role_with_condition->conditions() != nullptr)
            {
                for (auto* const query: role_with_condition->conditions()->query())
                {
                    new_role->conditions_query = query->getText();
                    std::cout << "    DEBUG: Added conditions: " << new_role->conditions_query << std::endl;
                }
            }
            
            // Parse capacites if present
            if (role_with_condition->capacites_list() != nullptr)
            {
                std::cout << "    DEBUG: Parsing capacites for role '" << new_role->role_name << "'" << std::endl;
                for (auto* const capacite : role_with_condition->capacites_list()->capacite())
                {
                    auto new_capacite = parseCapacite(capacite);
                    new_role->capacites.push_back(*new_capacite);
                    std::cout << "      DEBUG: Added capacite '" << new_capacite->name << "' (satisfies: ";
                    for (const auto& exp : new_capacite->can_satisfy_expectations) {
                        std::cout << exp << " ";
                    }
                    std::cout << ")" << std::endl;
                }
            }
            
            // Parse attentes if present
            if (role_with_condition->attentes_list() != nullptr)
            {
                std::cout << "    DEBUG: Parsing frame-level attentes for role '" << new_role->role_name << "'" << std::endl;
                for (auto* const attente : role_with_condition->attentes_list()->attente_extended())
                {
                    auto new_attente = parseAttenteExtended(attente);
                    new_role->attentes.push_back(new_attente);
                    std::cout << "      DEBUG: Added frame attente '" << new_attente.name << "' (type: " << new_attente.type << ")" << std::endl;
                }
            }
            
            std::cout << "    DEBUG: Role '" << new_role->role_name << "' complete with " 
                      << new_role->capacites.size() << " capacites and " 
                      << new_role->attentes.size() << " frame-level attentes" << std::endl;
            new_frame->roles.push_back(*new_role);
        }
    }
    
    // Legacy support for old roles_list structure (if needed)
    // Note: roles_list() method not available in new grammar
    // Removed for compatibility with new grammar structure
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

    // Parse execution bloc
    for (auto* exec_bloc: action->execution_bloc())
    {
        new_action.executions_bloc = parseExecutionBloc(exec_bloc);
    }

    // Parse description bloc
    for (auto* const description_bloc: action->description_bloc())
    {
        new_action.description = parseDescriptionBloc(description_bloc);
    }

    // Parse preconditions bloc (optional, single)
    if (action->preconditions_bloc() != nullptr)
    {
        new_action.preconditions = parseActionPreconditionsBloc(action->preconditions_bloc());
    }

    // Parse effects bloc (optional, single)
    if (action->effects_bloc() != nullptr)
    {
        new_action.effects = parseActionEffectsBloc(action->effects_bloc());
    }

    // Parse duration bloc (multiple possible)
    auto duration_blocs = action->duration_bloc();
    if (!duration_blocs.empty())
    {
        new_action.duration = parseActionDurationBloc(duration_blocs[0]);  // Take first one
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
Attente& FullParser::parseAttente(ExtentedHATPParser::AttenteContext* attente)
{
    const auto res = new Attente();
    res->name = attente->name()->getText();
    const auto condition = attente->conditions();
    for (auto* const query : condition->query())
    {
        res->query = query->getText();
    }
    for (auto* const triplet: condition->triplet())
    {
        Triplet_t new_triplet = parseTriplet(triplet);
        res->triplets.push_back(new_triplet.toRosMsg());
    }


    return *res;
}

Attente& FullParser::parseAttenteExtended(ExtentedHATPParser::Attente_extendedContext* attente)
{
    const auto res = new Attente();
    res->name = trim(attente->name()->getText());

    // Parse TYPE field if present
    if (!attente->attente_type().empty())
    {
        res->type = attente->attente_type()[0]->name()->getText();
        // Remove quotes from type string
        if (res->type.length() > 2 && res->type[0] == '"' && res->type.back() == '"')
        {
            res->type = res->type.substr(1, res->type.length() - 2);
        }
        // Trim whitespace
        res->type = trim(res->type);
    }
    
    // Parse EXPECTS_FROM field if present
    if (!attente->expects_from().empty() && attente->expects_from()[0]->role_list() != nullptr)
    {
        for (auto* const role_name : attente->expects_from()[0]->role_list()->name())
        {
            res->expects_from.push_back(role_name->getText());
        }
    }
    
    // Parse DESCRIPTION field if present
    if (!attente->description_attente().empty())
    {
        res->description = attente->description_attente()[0]->sentence()->getText();
        // Remove quotes from description string
        if (res->description.length() > 2 && res->description[0] == '"' && res->description.back() == '"')
        {
            res->description = res->description.substr(1, res->description.length() - 2);
        }
        // Trim whitespace
        res->description = trim(res->description);
    }
    
    // Parse conditions if present
    if (!attente->conditions().empty())
    {
        const auto condition = attente->conditions()[0];
        for (auto* const query : condition->query())
        {
            res->query = query->getText();
        }
        for (auto* const triplet: condition->triplet())
        {
            Triplet_t new_triplet = parseTriplet(triplet);
            res->triplets.push_back(new_triplet.toRosMsg());
        }
    }

    return *res;
}

Capacite* FullParser::parseCapacite(ExtentedHATPParser::CapaciteContext* ctx)
{
    const auto res = new Capacite();
    res->name = trim(ctx->name()->getText());
    
    // Parse CAN_SATISFY_EXPECTATIONS field if present
    if (!ctx->can_satisfy_expectations().empty())
    {
        auto can_satisfy = ctx->can_satisfy_expectations()[0];
        if (can_satisfy->expectation_type_list() != nullptr)
        {
            for (auto* const expectation_type : can_satisfy->expectation_type_list()->name())
            {
                res->can_satisfy_expectations.push_back(expectation_type->getText());
            }
        }
    }
    
    // Parse DESCRIPTION field if present
    if (!ctx->description_capacite().empty())
    {
        res->description = ctx->description_capacite()[0]->sentence()->getText();
        // Remove quotes from description string
        if (res->description.length() > 2 && res->description[0] == '"' && res->description.back() == '"')
        {
            res->description = res->description.substr(1, res->description.length() - 2);
        }
        // Trim whitespace
        res->description = trim(res->description);
    }
    
    // Parse conditions if present
    if (!ctx->conditions().empty())
    {
        const auto condition = ctx->conditions()[0];
        for (auto* const query : condition->query())
        {
            res->conditions_query = query->getText();
        }
        // Note: Capacite doesn't have triplets field in the message, but conditions_query covers it
    }

    return res;
}

Role* FullParser::parseRole(ExtentedHATPParser::RoleContext* ctx)
{
    const auto res = new Role();
    res->role_name = trim(ctx->name()->getText());
    for (auto *const query: ctx->conditions()->query())
    {
        res->conditions_query = query->getText();
    }
    for (auto* const attente: ctx->attente())
    {
        auto new_intent = parseAttente(attente);
        res->attentes.push_back(new_intent);
    }
    return res;

}

Priority* FullParser::parsePriority(ExtentedHATPParser::PriorityContext* ctx)
{
    const auto priority = new Priority();
    priority->name = ctx->name()->getText();
    
    // Parse priority level
    for (auto* const priority_level_ctx: ctx->priority_level())
    {
        priority->level = std::stoi(priority_level_ctx->numeric_value()->getText());
    }
    
    // Parse event (preconditions SPARQL)
    for (auto* const event_ctx: ctx->event())
    {
        for (auto* const query: event_ctx->query())
        {
            priority->preconditions.push_back(query->getText());
        }
    }
    
    // Parse objectives (new structure with STATE and TASK support)
    for (auto* const objectives_ctx: ctx->objectifs())
    {
        // Parse objectives_content items
        for (auto* const content_ctx: objectives_ctx->objectives_content())
        {
            // Check if it's a STATE block
            if (content_ctx->state_bloc() != nullptr)
            {
                auto* state_ctx = content_ctx->state_bloc();
                for (auto* const query: state_ctx->query())
                {
                    priority->state_objectives.push_back(query->getText());
                }
                for (auto* const triplet: state_ctx->triplet())
                {
                    // Convert triplet to SPARQL-like string representation
                    Triplet_t parsed_triplet = parseTriplet(triplet);
                    std::string triplet_str = parsed_triplet.subject.literal + " " +
                                            parsed_triplet.property + " " +
                                            parsed_triplet.object.literal;
                    priority->state_objectives.push_back(triplet_str);
                }
            }
            // Check if it's a TASK block
            else if (content_ctx->task_bloc() != nullptr)
            {
                auto* task_ctx = content_ctx->task_bloc();
                priority->task_name = task_ctx->name()->getText();

                // Parse task parameters
                for (auto* const task_arg: task_ctx->task_arg())
                {
                    priority->task_parameters.push_back(task_arg->getText());
                }
            }
            // Handle legacy query/triplet (for backward compatibility)
            else if (content_ctx->query() != nullptr)
            {
                priority->objectives.push_back(content_ctx->query()->getText());
            }
            else if (content_ctx->triplet() != nullptr)
            {
                // Convert triplet to SPARQL-like string representation
                Triplet_t parsed_triplet = parseTriplet(content_ctx->triplet());
                std::string triplet_str = parsed_triplet.subject.literal + " " +
                                        parsed_triplet.property + " " +
                                        parsed_triplet.object.literal;
                priority->objectives.push_back(triplet_str);
            }
        }

    }
    
    // Initialize with default values
    priority->activation_id = 0;
    priority->deactivation_id = 0;
    
    return priority;
}

std::vector<ActionPrecondition_t> FullParser::parseActionPreconditionsBloc(ExtentedHATPParser::Preconditions_blocContext* ctx)
{
    std::vector<ActionPrecondition_t> preconditions;
    if (ctx == nullptr) return preconditions;

    for (auto* const triplet_ctx: ctx->triplet())
    {
        ActionPrecondition_t precond;
        // Parse basic triplet structure: subject predicate object
        Triplet_t parsed_triplet = parseTriplet(triplet_ctx);
        precond.subject = parsed_triplet.subject.literal;
        precond.predicate = parsed_triplet.property;
        precond.object = parsed_triplet.object.literal;
        precond.is_negative = !parsed_triplet.add;  // NOT is represented as !add
        preconditions.push_back(precond);
    }
    return preconditions;
}

std::vector<ActionEffect_t> FullParser::parseActionEffectsBloc(ExtentedHATPParser::Effects_blocContext* ctx)
{
    std::vector<ActionEffect_t> effects;
    if (ctx == nullptr) return effects;

    for (auto* const triplet_ctx: ctx->triplet())
    {
        ActionEffect_t effect;
        // Parse basic triplet structure: subject predicate object
        Triplet_t parsed_triplet = parseTriplet(triplet_ctx);
        effect.subject = parsed_triplet.subject.literal;
        effect.predicate = parsed_triplet.property;
        effect.object = parsed_triplet.object.literal;
        effect.is_add = parsed_triplet.add;  // true for ADD, false for REMOVE
        effects.push_back(effect);
    }
    return effects;
}

double FullParser::parseActionDurationBloc(ExtentedHATPParser::Duration_blocContext* ctx)
{
    if (ctx == nullptr) return 0.0;

    auto* numeric = ctx->numeric_value();
    if (numeric != nullptr)
    {
        return std::stod(numeric->getText());
    }
    return 0.0;
}

void FullParser::parseTasksBloc(ExtentedHATPParser::Tasks_blocContext* ctx)
{
    for (auto* const task_ctx: ctx->task())
    {
        std::cout << "Parsing task: " << task_ctx->name()->getText() << std::endl;
        Abstract_task_t parsed_task = parseTask(task_ctx);
        tasks_.insert(std::make_pair(parsed_task.name, parsed_task));
    }
}

Abstract_task_t FullParser::parseTask(ExtentedHATPParser::TaskContext* ctx)
{
    Abstract_task_t task;
    task.name = ctx->name()->getText();

    // Parse arguments
    for (auto* const arg_ctx: ctx->arguments())
    {
        Arguments_t arg;
        arg.type = arg_ctx->type()->getText();
        arg.name = arg_ctx->varname()->getText();
        task.arguments.push_back(arg);
    }

    // Parse methods (optional, single)
    auto methods_bloc_ctx = ctx->methods_bloc();
    if (methods_bloc_ctx != nullptr)
    {
        for (auto* const method_ctx: methods_bloc_ctx->method())
        {
            Method_t method = parseMethod(method_ctx);
            task.methods_.push_back(method);
        }
    }

    // Parse goal as goals (optional, single)
    auto goal_ctx = ctx->goal();
    if (goal_ctx != nullptr)
    {
        for (auto* const triplet_ctx: goal_ctx->triplet())
        {
            Expression_t goal_expr;
            Triplet_t parsed_triplet = parseTriplet(triplet_ctx);
            // Convert triplet to expression format
            goal_expr.property = parsed_triplet.property;
            goal_expr.subject = parsed_triplet.subject.literal;
            goal_expr.object = parsed_triplet.object.literal;
            goal_expr.add = parsed_triplet.add;
            task.goals.push_back(goal_expr);
        }
    }

    return task;
}

Method_t FullParser::parseMethod(ExtentedHATPParser::MethodContext* ctx)
{
    Method_t method;

    // Parse preconditions (multiple possible)
    auto precond_blocs = ctx->preconditions_bloc();
    for (size_t i = 0; i < precond_blocs.size(); ++i)
    {
        auto* precond_ctx = precond_blocs[i];
        for (auto* const triplet_ctx: precond_ctx->triplet())
        {
            Expression_t precond_expr;
            Triplet_t parsed_triplet = parseTriplet(triplet_ctx);
            precond_expr.property = parsed_triplet.property;
            precond_expr.subject = parsed_triplet.subject.literal;
            precond_expr.object = parsed_triplet.object.literal;
            precond_expr.add = parsed_triplet.add;
            method.preconditions.push_back(precond_expr);
        }
    }

    // Parse subtasks as subtask (multiple possible)
    auto subtask_blocs = ctx->subtask_bloc();
    int action_id = 0;
    for (size_t i = 0; i < subtask_blocs.size(); ++i)
    {
        auto* subtask_ctx = subtask_blocs[i];
        for (auto* const subtask_line: subtask_ctx->subtask_line())
        {
            Ordered_Action_t ordered_action;
            ordered_action.id = action_id++;
            ordered_action.name = subtask_line->name()->getText();

            // Add arguments
            for (auto* const arg: subtask_line->arg())
            {
                ordered_action.arguments.push_back(arg->getText());
            }

            method.subtask.map_actions[ordered_action.id] = ordered_action;
        }
    }

    return method;
}

} // procedural
