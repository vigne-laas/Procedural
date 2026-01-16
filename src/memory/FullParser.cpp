    #include "procedural/memory/FullParser.h"

#include <ExtentedHATPParser.h>
#include <ExtentedHATPLexer.h>
#include <procedural_interfaces/action_t.h>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <regex>



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

    // Phase 1: Traitement des inclusions
    std::cout << "=== PHASE 1: PROCESSING INCLUDES ===" << std::endl;
    std::set<std::string> processing_stack;

    for (auto* const inclusion: ctx->include_bloc())
    {
        std::cout << "Processing include_bloc" << std::endl;
        processIncludes(inclusion);
    }

    // Traiter tous les fichiers dans la queue
    while (!files_to_process_.empty()) {
        FileContext file_context = files_to_process_.front();
        files_to_process_.pop();

        try {
            std::string resolved_path = resolveFilePath(file_context.filepath, file_context.package, file_context.base_directory);
            processIncludeFileRecursively(resolved_path, processing_stack);
        } catch (const std::exception& e) {
            std::cerr << "Error processing include file: " << e.what() << std::endl;
        }
    }

    // Phase 2: Parser le contenu du fichier courant et l'ajouter aux maps temporaires
    std::cout << "=== PHASE 2: PARSING CURRENT FILE CONTENT ===" << std::endl;

    // Parser les actions du fichier courant
    for (auto* const action_bloc: ctx->actions_bloc())
    {
        std::cout << "Parsing actions_bloc (current file)" << std::endl;
        Actions_t temp_actions = actions_; // Sauvegarder l'état actuel
        actions_.actions.clear(); // Vider temporairement

        parseActionBloc(action_bloc);

        // Ajouter aux maps temporaires
        for (const auto& action : actions_.actions) {
            std::string signature = getActionSignature(action);
            pending_actions_[signature].push_back(action);
        }

        actions_ = temp_actions; // Restaurer l'état
    }

    // Parser les frames du fichier courant
    for (auto* const frames: ctx->pratices_frames_bloc())
    {
        std::cout << "Parsing pratices_frames_bloc (current file)" << std::endl;
        for (auto* const frame: frames->practice_frame())
        {
            PracticeFrame* parsed_frame = parsePracticeFrame(frame);
            pending_frames_[parsed_frame->name].push_back(parsed_frame);
        }
    }

    // Parser les practices du fichier courant
    for (auto* const practices: ctx->practices_bloc())
    {
        std::cout << "Parsing practices_bloc (current file)" << std::endl;
        for (auto* const practice_item: practices->practice())
        {
            std::cout << "Parsing practice_action: " << practice_item->getText() << std::endl;
            Practice* parsed_practice = parsePractice(practice_item);
            pending_practices_[parsed_practice->name].push_back(parsed_practice);
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

    // Parser les priorities du fichier courant
    for (auto* const priorities_bloc: ctx->priorities_bloc())
    {
        std::cout << "Parsing priorities_bloc (current file)" << std::endl;
        for (auto* const priority: priorities_bloc->priority())
        {
            std::cout << "Parsing priority: " << priority->name()->getText() << std::endl;
            Priority* parsed_priority = parsePriority(priority);
            pending_priorities_[parsed_priority->name].push_back(parsed_priority);
        }
    }

    // Parser les tasks du fichier courant
    for (auto* const tasks_bloc: ctx->tasks_bloc())
    {
        std::cout << "Parsing tasks_bloc (current file)" << std::endl;
        std::map<std::string, Abstract_task_t> temp_tasks = tasks_; // Sauvegarder l'état actuel
        tasks_.clear(); // Vider temporairement

        parseTasksBloc(tasks_bloc);

        // Ajouter aux maps temporaires
        for (const auto& [name, task] : tasks_) {
            std::string signature = getTaskSignature(task);
            pending_tasks_[signature].push_back(task);
        }

        tasks_ = temp_tasks; // Restaurer l'état
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
    // Phase 3: Fusionner les éléments (après parsing du fichier courant)
    std::cout << "=== PHASE 3: MERGING ELEMENTS ===" << std::endl;
    mergeElements();

    // Phase 4: Liens et finalisation
    std::cout << "=== PHASE 4: LINKING ===" << std::endl;
    linkRolesToPractices();
    linkRolesToFrames();
    linkPracticesToFrames();

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

    // Parse recognition bloc (optional, single)
    for (auto* recognition_bloc : action->recognition_bloc())
    {
        new_action.recognition = parseRecognitionBloc(recognition_bloc);
    }

    // Parse commitments bloc (optional, single)
    if (action->commitments() != nullptr)
    {
        CommitmentBlock_t parsed_commitments = parseCommitmentBlock(action->commitments());

        // Create shared_ptr and assign directly
        new_action.commitments = std::make_shared<CommitmentBlock_t>(parsed_commitments);
        new_action.has_commitments = true;
        std::cout << "  ✓ Parsed commitments for action: " << new_action.name << std::endl;
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
            new_variable.literal = "??";
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
            new_variable.literal = "??";
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

Recognition_t FullParser::parseRecognitionBloc(ExtentedHATPParser::Recognition_blocContext* ctx)
{
    Recognition_t recognition;

    if (ctx == nullptr) return recognition;

    // Parse sequence bloc
    auto* sequence_bloc = ctx->sequence_bloc();
    if (sequence_bloc != nullptr)
    {
        for (auto* sequence : sequence_bloc->sequence())
        {
            RecognitionSequenceStep_t step;

            // Parse subject, predicate, object
            step.subject = trim(sequence->subject()->getText());
            step.predicate = trim(sequence->predicate()->getText());
            step.object = trim(sequence->object()->getText());

            // Check for NOT modifier
            step.is_negative = (sequence->NOT() != nullptr);

            // Check for REQUIRED modifier
            step.is_required = (sequence->REQUIRED() != nullptr);

            recognition.sequence.push_back(step);
        }
    }

    // Parse parameters bloc (optional)
    auto* parameters_bloc = ctx->parameters_bloc();
    if (parameters_bloc != nullptr)
    {
        for (auto* parameter : parameters_bloc->parameter())
        {
            std::string param_name = parameter->name()->getText();
            if (param_name == "ttl")
            {
                auto* value = parameter->value();
                if (value != nullptr)
                {
                    recognition.parameters.ttl = std::stod(value->getText());
                }
            }
        }
    }

    return recognition;
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

    // Parse method name/ID
    auto method_id_ctx = ctx->id_method();
    if (method_id_ctx != nullptr)
    {
        method.name = method_id_ctx->getText();
    }

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

// ===============================
// Système d'inclusion de fichiers
// ===============================

void FullParser::parseFileWithInclusions(const std::string& filepath) {
    std::cout << "=== STARTING INCLUSION-AWARE PARSING ===" << std::endl;

    // Phase 1: Initialisation
    current_directory_ = std::filesystem::path(filepath).parent_path();
    processed_files_.clear();
    parsed_files_.clear();
    pending_actions_.clear();
    pending_practices_.clear();
    pending_tasks_.clear();
    pending_frames_.clear();
    pending_roles_.clear();
    pending_priorities_.clear();

    std::cout << "Base directory: " << current_directory_ << std::endl;

    // Phase 2: Parser le fichier principal et tous les inclus récursivement
    std::set<std::string> processing_stack;
    processIncludeFileRecursively(filepath, processing_stack);

    // Phase 3: Fusionner tous les éléments
    mergeElements();

    // Phase 4: Appliquer les liaisons comme avant
    linkRolesToPractices();
    linkRolesToFrames();
    linkPracticesToFrames();

    std::cout << "=== INCLUSION-AWARE PARSING COMPLETE ===" << std::endl;
    displayResult();
}

void FullParser::processIncludeFileRecursively(const std::string& filepath, std::set<std::string>& processing_stack) {
    std::string resolved_path = std::filesystem::absolute(filepath);

    std::cout << "Processing file: " << resolved_path << std::endl;

    // Vérifier les dépendances circulaires
    if (processing_stack.find(resolved_path) != processing_stack.end()) {
        throw std::runtime_error("Circular dependency detected: " + resolved_path);
    }

    // Éviter de parser le même fichier plusieurs fois
    if (processed_files_.find(resolved_path) != processed_files_.end()) {
        std::cout << "File already processed, skipping: " << resolved_path << std::endl;
        return;
    }

    processing_stack.insert(resolved_path);
    processed_files_.insert(resolved_path);

    // Parser le fichier
    ParsedFileContent file_content = parseFile(resolved_path);

    // Stocker le contenu parsé dans les maps temporaires
    for (const auto& action : file_content.actions) {
        std::string signature = getActionSignature(action);
        pending_actions_[signature].push_back(action);
    }

    for (const auto& practice : file_content.practices) {
        pending_practices_[practice->name].push_back(practice);
    }

    for (const auto& task : file_content.tasks) {
        std::string signature = getTaskSignature(task);
        pending_tasks_[signature].push_back(task);
    }

    for (const auto& frame : file_content.frames) {
        pending_frames_[frame->name].push_back(frame);
    }

    for (const auto& role : file_content.roles) {
        pending_roles_[role->role_name].push_back(role);
    }

    for (const auto& priority : file_content.priorities) {
        pending_priorities_[priority->name].push_back(priority);
    }

    // Traiter les inclusions de ce fichier
    // (Ceci nécessiterait de parser le fichier pour extraire les includes)
    // Pour l'instant, nous allons implémenter une version simplifiée

    processing_stack.erase(resolved_path);
}

ParsedFileContent FullParser::parseFile(const std::string& filepath) {
    ParsedFileContent content(filepath);

    std::cout << "Parsing individual file: " << filepath << std::endl;

    // Créer un parser ANTLR pour ce fichier spécifique
    try {
        std::ifstream stream(filepath);
        if (!stream.is_open()) {
            throw std::runtime_error("Cannot open file: " + filepath);
        }

        antlr4::ANTLRInputStream input(stream);
        ExtentedHATPLexer lexer(&input);
        antlr4::CommonTokenStream tokens(&lexer);
        ExtentedHATPParser parser(&tokens);

        // Parser le fichier
        ExtentedHATPParser::RootContext* tree = parser.root();

        // Sauvegarder l'état actuel des variables membres
        Actions_t temp_actions = actions_;
        std::map<std::string, Practice*> temp_practices = practices_;
        std::map<std::string, Abstract_task_t> temp_tasks = tasks_;
        std::map<std::string, Priority*> temp_priorities = priorities_;
        std::vector<PracticeFrame*> temp_frames = practice_frames_;

        // Vider temporairement les variables membres
        actions_.actions.clear();
        practices_.clear();
        tasks_.clear();
        priorities_.clear();
        practice_frames_.clear();

        // D'abord traiter les includes de ce fichier
        std::string file_dir = std::filesystem::path(filepath).parent_path();
        std::set<std::string> local_processing_stack;

        for (auto* const include_bloc: tree->include_bloc()) {
            std::cout << "Processing include_bloc with " << include_bloc->inclusion().size() << " inclusions" << std::endl;
            for (auto* const inclusion_item: include_bloc->inclusion()) {
                std::cout << "Processing inclusion with " << inclusion_item->STRING().size() << " strings" << std::endl;
                for (auto* const string_node: inclusion_item->STRING()) {
                    std::string include_path = string_node->getText();
                    std::cout << "Raw include path: '" << include_path << "'" << std::endl;

                    // Enlever les guillemets si présents
                    if (include_path.length() >= 2 && include_path.front() == '"' && include_path.back() == '"') {
                        include_path = include_path.substr(1, include_path.length() - 2);
                    }

                    std::cout << "Cleaned include path: '" << include_path << "'" << std::endl;

                    if (!include_path.empty()) {
                        std::string resolved_include_path = resolveFilePath(include_path, "", file_dir);
                        std::cout << "Found include: " << resolved_include_path << std::endl;

                        // Parser récursivement le fichier inclus
                        processIncludeFileRecursively(resolved_include_path, local_processing_stack);
                    } else {
                        std::cout << "Warning: Empty include path found" << std::endl;
                    }
                }
            }
        }

        // Parser les différents blocs du fichier
        for (auto* const action_bloc: tree->actions_bloc()) {
            parseActionBloc(action_bloc);
            for (const auto& action : actions_.actions) {
                content.actions.push_back(action);
            }
            actions_.actions.clear();
        }

        for (auto* const frames: tree->pratices_frames_bloc()) {
            for (auto* const frame: frames->practice_frame()) {
                PracticeFrame* parsed_frame = parsePracticeFrame(frame);
                content.frames.push_back(parsed_frame);
            }
        }

        for (auto* const practices: tree->practices_bloc()) {
            for (auto* const practice_item: practices->practice()) {
                Practice* parsed_practice = parsePractice(practice_item);
                content.practices.push_back(parsed_practice);
            }
        }

        for (auto* const priorities_bloc: tree->priorities_bloc()) {
            for (auto* const priority: priorities_bloc->priority()) {
                Priority* parsed_priority = parsePriority(priority);
                content.priorities.push_back(parsed_priority);
            }
        }

        for (auto* const tasks_bloc: tree->tasks_bloc()) {
            parseTasksBloc(tasks_bloc);
            for (const auto& task_pair : tasks_) {
                content.tasks.push_back(task_pair.second);
            }
            tasks_.clear();
        }

        // Restaurer l'état original
        actions_ = temp_actions;
        practices_ = temp_practices;
        tasks_ = temp_tasks;
        priorities_ = temp_priorities;
        practice_frames_ = temp_frames;

        std::cout << "File parsing complete. Found: "
                  << content.actions.size() << " actions, "
                  << content.practices.size() << " practices, "
                  << content.tasks.size() << " tasks, "
                  << content.priorities.size() << " priorities, "
                  << content.frames.size() << " frames" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error parsing file " << filepath << ": " << e.what() << std::endl;
        throw;
    }

    return content;
}

std::string FullParser::resolveFilePath(const std::string& path, const std::string& package, const std::string& base_dir) {
    std::cout << "Resolving path: " << path << " (package: " << package << ", base: " << base_dir << ")" << std::endl;

    if (!package.empty()) {
        // Résolution via package ROS
        std::string pkg_path = getPackagePath(package);
        if (!pkg_path.empty()) {
            return std::filesystem::absolute(pkg_path + "/" + path);
        } else {
            throw std::runtime_error("Package not found: " + package);
        }
    }

    // Chemin absolu
    if (std::filesystem::path(path).is_absolute()) {
        return std::filesystem::absolute(path);
    }

    // Chemin relatif
    std::string base = base_dir.empty() ? current_directory_ : base_dir;
    return std::filesystem::absolute(base + "/" + path);
}

std::string FullParser::getPackagePath(const std::string& package_name) {
    std::cout << "Getting package path for: " << package_name << std::endl;

    // Utiliser rospack pour trouver le chemin du package
    std::string command = "rospack find " + package_name + " 2>/dev/null";

    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        std::cerr << "Failed to execute rospack command" << std::endl;
        return "";
    }

    char buffer[256];
    std::string result;

    if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result = buffer;
        // Enlever le newline final
        if (!result.empty() && result.back() == '\n') {
            result.pop_back();
        }
    }

    pclose(pipe);

    std::cout << "Package path: " << result << std::endl;
    return result;
}

void FullParser::mergeElements() {
    std::cout << "=== MERGING ELEMENTS ===" << std::endl;

    // Fusionner les actions
    for (const auto& [signature, action_list] : pending_actions_) {
        if (action_list.size() == 1) {
            actions_.actions.push_back(action_list[0]);
            std::cout << "Added single action: " << signature << std::endl;
        } else {
            Action_t merged = mergeActionDefinitions(action_list);
            actions_.actions.push_back(merged);
            std::cout << "Merged " << action_list.size() << " action definitions: " << signature << std::endl;
        }
    }

    // Fusionner les practices
    for (const auto& [name, practice_list] : pending_practices_) {
        if (practice_list.size() == 1) {
            practices_[name] = practice_list[0];
            std::cout << "Added single practice: " << name << std::endl;
        } else {
            Practice* merged = mergePracticeDefinitions(practice_list);
            practices_[name] = merged;
            std::cout << "Merged " << practice_list.size() << " practice definitions: " << name << std::endl;
        }
    }

    // Fusionner les tasks
    for (const auto& [signature, task_list] : pending_tasks_) {
        if (task_list.size() == 1) {
            tasks_[task_list[0].name] = task_list[0];
            std::cout << "Added single task: " << signature << std::endl;
        } else {
            Abstract_task_t merged = mergeTaskDefinitions(task_list);
            tasks_[merged.name] = merged;
            std::cout << "Merged " << task_list.size() << " task definitions: " << signature << std::endl;
        }
    }

    // Fusionner les frames
    for (const auto& [name, frame_list] : pending_frames_) {
        if (frame_list.size() == 1) {
            practice_frames_.push_back(frame_list[0]);
            std::cout << "Added single frame: " << name << std::endl;
        } else {
            PracticeFrame* merged = mergePracticeFrameDefinitions(frame_list);
            practice_frames_.push_back(merged);
            std::cout << "Merged " << frame_list.size() << " frame definitions: " << name << std::endl;
        }
    }

    // Fusionner les roles
    for (const auto& [name, role_list] : pending_roles_) {
        if (role_list.size() == 1) {
            roles_[name] = role_list[0];
            std::cout << "Added single role: " << name << std::endl;
        } else {
            // Pour l'instant, prendre le premier rôle (TODO: implémenter fusion des rôles)
            roles_[name] = role_list[0];
            std::cout << "Using first role definition (merge not implemented): " << name << std::endl;
        }
    }

    // Fusionner les priorities
    for (const auto& [name, priority_list] : pending_priorities_) {
        if (priority_list.size() == 1) {
            priorities_[name] = priority_list[0];
            std::cout << "Added single priority: " << name << std::endl;
        } else {
            // Pour l'instant, prendre la première priorité (TODO: implémenter fusion des priorités)
            priorities_[name] = priority_list[0];
            std::cout << "Using first priority definition (merge not implemented): " << name << std::endl;
        }
    }

    std::cout << "=== MERGING COMPLETE ===" << std::endl;
}

Action_t FullParser::mergeActionDefinitions(const std::vector<Action_t>& actions) {
    if (actions.empty()) {
        throw std::runtime_error("Cannot merge empty action list");
    }

    Action_t merged = actions[0];
    std::cout << "Merging " << actions.size() << " actions for: " << merged.name << std::endl;

    for (size_t i = 1; i < actions.size(); i++) {
        const Action_t& current = actions[i];

        // Fusionner les préconditions
        merged.preconditions.insert(merged.preconditions.end(),
                                   current.preconditions.begin(),
                                   current.preconditions.end());

        // Fusionner les effets
        merged.effects.insert(merged.effects.end(),
                             current.effects.begin(),
                             current.effects.end());

        // Fusionner l'exécution (prendre la première non-vide)
        if (merged.executions_bloc.empty() && !current.executions_bloc.empty()) {
            merged.executions_bloc = current.executions_bloc;
        }

        // Fusionner la description (prendre la première non-vide)
        if (merged.description.description.empty() && !current.description.description.empty()) {
            merged.description = current.description;
        }

        // Prendre la durée si non définie
        if (merged.duration == 0 && current.duration > 0) {
            merged.duration = current.duration;
        }
    }

    return merged;
}

Practice* FullParser::mergePracticeDefinitions(const std::vector<Practice*>& practices) {
    if (practices.empty()) {
        throw std::runtime_error("Cannot merge empty practice list");
    }

    Practice* merged = new Practice(*practices[0]);
    std::cout << "Merging " << practices.size() << " practices for: " << merged->name << std::endl;

    for (size_t i = 1; i < practices.size(); i++) {
        const Practice* current = practices[i];

        // Fusionner les compétences
        merged->competences.insert(merged->competences.end(),
                                  current->competences.begin(),
                                  current->competences.end());

        // Fusionner les objets
        merged->objects.insert(merged->objects.end(),
                              current->objects.begin(),
                              current->objects.end());

        // Fusionner les conditions d'activation
        merged->activation_conditions.insert(merged->activation_conditions.end(),
                                            current->activation_conditions.begin(),
                                            current->activation_conditions.end());

        // Fusionner les rôles
        merged->roles.insert(merged->roles.end(),
                            current->roles.begin(),
                            current->roles.end());

        // Fusionner les règles
        merged->rules.insert(merged->rules.end(),
                            current->rules.begin(),
                            current->rules.end());

        // Prendre la description si vide
        if (merged->description.empty() && !current->description.empty()) {
            merged->description = current->description;
        }
    }

    return merged;
}

Abstract_task_t FullParser::mergeTaskDefinitions(const std::vector<Abstract_task_t>& tasks) {
    if (tasks.empty()) {
        throw std::runtime_error("Cannot merge empty task list");
    }

    Abstract_task_t merged = tasks[0];
    std::cout << "Merging " << tasks.size() << " tasks for: " << merged.name << std::endl;

    for (size_t i = 1; i < tasks.size(); i++) {
        const Abstract_task_t& current = tasks[i];

        // Fusionner les méthodes
        merged.methods_.insert(merged.methods_.end(),
                              current.methods_.begin(),
                              current.methods_.end());
    }

    return merged;
}

PracticeFrame* FullParser::mergePracticeFrameDefinitions(const std::vector<PracticeFrame*>& frames) {
    if (frames.empty()) {
        throw std::runtime_error("Cannot merge empty frame list");
    }

    PracticeFrame* merged = new PracticeFrame(*frames[0]);
    std::cout << "Merging " << frames.size() << " frames for: " << merged->name << std::endl;

    for (size_t i = 1; i < frames.size(); i++) {
        const PracticeFrame* current = frames[i];

        // Fusionner les practices
        merged->practices.insert(merged->practices.end(),
                                current->practices.begin(),
                                current->practices.end());

        // Prendre la description si vide
        if (merged->description.empty() && !current->description.empty()) {
            merged->description = current->description;
        }
    }

    return merged;
}

std::string FullParser::getActionSignature(const Action_t& action) {
    std::string signature = action.name + "(";
    for (size_t i = 0; i < action.arguments.size(); i++) {
        if (i > 0) signature += ",";
        signature += action.arguments[i].type + " " + action.arguments[i].literal;
    }
    signature += ")";
    return signature;
}

std::string FullParser::getTaskSignature(const Abstract_task_t& task) {
    std::string signature = task.name + "(";
    for (size_t i = 0; i < task.arguments.size(); i++) {
        if (i > 0) signature += ",";
        signature += task.arguments[i].type + " " + task.arguments[i].name;
    }
    signature += ")";
    return signature;
}

void FullParser::processIncludes(ExtentedHATPParser::Include_blocContext* include_bloc) {
    std::cout << "Processing includes from include_bloc" << std::endl;

    for (auto* const include: include_bloc->inclusion()) {
        std::string filepath;
        std::string package;

        // Extraire le nom du fichier
        if (include->STRING().size() >= 2) {
            std::string first_string = include->STRING(0)->getText();
            std::string second_string = include->STRING(1)->getText();

            // Enlever les guillemets
            if (first_string.length() >= 2 && first_string.front() == '"' && first_string.back() == '"') {
                first_string = first_string.substr(1, first_string.length() - 2);
            }
            if (second_string.length() >= 2 && second_string.front() == '"' && second_string.back() == '"') {
                second_string = second_string.substr(1, second_string.length() - 2);
            }

            filepath = second_string; // Le fichier est généralement le deuxième string
        }

        // Vérifier s'il y a un package link
        if (include->package_link() != nullptr) {
            if (include->package_link()->link() != nullptr) {
                package = include->package_link()->link()->getText();
                std::cout << "Package Link detected: " << package << std::endl;
            }
        }
        // Vérifier s'il y a un link simple
        else if (include->link() != nullptr) {
            std::string link_text = include->link()->getText();
            std::cout << "Direct Link detected: " << link_text << std::endl;
            filepath = link_text;
        }

        if (!filepath.empty()) {
            std::cout << "Adding file to processing queue: " << filepath;
            if (!package.empty()) {
                std::cout << " (package: " << package << ")";
            }
            std::cout << std::endl;

            FileContext file_context(filepath, package, current_directory_);
            files_to_process_.push(file_context);
        } else {
            std::cerr << "Warning: Could not extract filepath from inclusion" << std::endl;
        }
    }
}

// ============================================================================
// Commitment Parsing Methods
// ============================================================================

CommitmentBlock_t FullParser::parseCommitmentBlock(
    ExtentedHATPParser::CommitmentsContext* ctx)
{
    CommitmentBlock_t commitment_block;
    commitment_block.has_commitments = true;

    if (ctx == nullptr) {
        return commitment_block;
    }

    // Get the raw text content
    std::string commitment_text = ctx->getText();

    std::cout << "  Parsing COMMITMENTS block..." << std::endl;

    // Parse INSTRUMENTAL conditions
    commitment_block.instrumental = parseConditionsWithFor(commitment_text, "INSTRUMENTAL");
    std::cout << "    Found " << commitment_block.instrumental.size() << " INSTRUMENTAL conditions" << std::endl;

    // Parse ENGAGEMENT conditions
    commitment_block.engagement = parseConditionsWithFor(commitment_text, "ENGAGEMENT");
    std::cout << "    Found " << commitment_block.engagement.size() << " ENGAGEMENT conditions" << std::endl;

    // Parse COMMON_GROUND conditions
    commitment_block.common_ground = parseConditionsWithFor(commitment_text, "COMMON_GROUND");
    std::cout << "    Found " << commitment_block.common_ground.size() << " COMMON_GROUND conditions" << std::endl;

    // Parse recovery actions
    commitment_block.on_instrumental_failure = parseRecoveryAction(ctx, "ON_INSTRUMENTAL_FAILURE");
    commitment_block.on_engagement_failure = parseRecoveryAction(ctx, "ON_ENGAGEMENT_FAILURE");
    commitment_block.on_common_ground_failure = parseRecoveryAction(ctx, "ON_COMMON_GROUND_FAILURE");

    // Parse recovery strategy directly from the commitment text
    size_t recovery_pos = commitment_text.find("RECOVERY_STRATEGY{");
    if (recovery_pos != std::string::npos) {
        // Helper to extract quoted values
        auto extractQuotedValue = [](const std::string& text, const std::string& key) -> std::string {
            size_t pos = text.find(key + ":");
            if (pos != std::string::npos) {
                size_t quote1 = text.find("\"", pos);
                if (quote1 != std::string::npos) {
                    size_t quote2 = text.find("\"", quote1 + 1);
                    if (quote2 != std::string::npos) {
                        return text.substr(quote1 + 1, quote2 - quote1 - 1);
                    }
                }
            }
            return "";
        };

        // Helper to extract numeric values
        auto extractNumericValue = [](const std::string& text, const std::string& key) -> std::string {
            size_t pos = text.find(key + ":");
            if (pos != std::string::npos) {
                size_t colon_pos = text.find(":", pos);
                size_t semicolon_pos = text.find(";", colon_pos);
                if (colon_pos != std::string::npos && semicolon_pos != std::string::npos) {
                    std::string num_str = text.substr(colon_pos + 1, semicolon_pos - colon_pos - 1);
                    // Trim whitespace
                    num_str.erase(0, num_str.find_first_not_of(" \t\n\r"));
                    num_str.erase(num_str.find_last_not_of(" \t\n\r") + 1);
                    return num_str;
                }
            }
            return "";
        };

        std::string mode = extractQuotedValue(commitment_text, "MODE");
        if (!mode.empty()) {
            commitment_block.recovery_strategy.mode = mode;
        }

        std::string attempts_str = extractNumericValue(commitment_text, "MAX_ATTEMPTS");
        if (!attempts_str.empty()) {
            try {
                commitment_block.recovery_strategy.max_attempts = std::stoi(attempts_str);
            } catch (...) {
                std::cerr << "Warning: Failed to parse MAX_ATTEMPTS value" << std::endl;
            }
        }

        std::string timeout_str = extractNumericValue(commitment_text, "TIMEOUT");
        if (!timeout_str.empty()) {
            try {
                commitment_block.recovery_strategy.timeout = std::stod(timeout_str);
            } catch (...) {
                std::cerr << "Warning: Failed to parse TIMEOUT value" << std::endl;
            }
        }
    }

    std::cout << "  ✓ Commitments parsed successfully" << std::endl;

    return commitment_block;
}

std::vector<CommitmentCondition_t> FullParser::parseConditionsWithFor(
    const std::string& text, const std::string& condition_type)
{
    std::vector<CommitmentCondition_t> conditions;

    // Find the block for this condition type
    std::string block_start = condition_type + "{";
    size_t block_pos = text.find(block_start);
    if (block_pos == std::string::npos) {
        return conditions;
    }

    // Find matching closing brace
    size_t start = text.find("{", block_pos) + 1;
    int depth = 1;
    size_t end = start;
    while (end < text.length() && depth > 0) {
        if (text[end] == '{') depth++;
        else if (text[end] == '}') depth--;
        if (depth > 0) end++;
    }

    if (end >= text.length()) {
        return conditions;
    }

    std::string block_text = text.substr(start, end - start);

    // Regex pattern to match: "description" [FOR for_clause] { SELECT ... };
    // Pattern explanation:
    // \"([^\"]+)\"          - Capture description in quotes
    // \\s*                 - Optional whitespace
    // (?:FOR\\s+([^{]+))?  - Optional FOR clause (non-capturing group with capturing inside)
    // \\s*\\{([^}]+)\\}    - SPARQL query in braces
    std::regex condition_regex(R"(\"([^\"]+)\"\s*(?:FOR\s+([^{]+))?\s*\{([^}]+)\})");

    std::sregex_iterator iter(block_text.begin(), block_text.end(), condition_regex);
    std::sregex_iterator end_iter;

    while (iter != end_iter) {
        std::smatch match = *iter;

        CommitmentCondition_t condition;

        // Capture group 1: description
        condition.description = trim(match[1].str());

        // Capture group 2: for_clause (optional)
        if (match[2].matched) {
            condition.for_clause = trim(match[2].str());
        } else {
            condition.for_clause = "";
        }

        // Capture group 3: SPARQL query
        condition.sparql_query = trim(match[3].str());

        conditions.push_back(condition);

        std::cout << "      Parsed: \"" << condition.description << "\"";
        if (!condition.for_clause.empty()) {
            std::cout << " FOR " << condition.for_clause;
        }
        std::cout << std::endl;

        ++iter;
    }

    return conditions;
}

std::string FullParser::parseRecoveryAction(
    ExtentedHATPParser::CommitmentsContext* ctx, const std::string& failure_type)
{
    if (ctx == nullptr) {
        return "";
    }

    std::string text = ctx->getText();
    std::string search_key = failure_type + ":";
    size_t pos = text.find(search_key);

    if (pos == std::string::npos) {
        return "";
    }

    // Extract the quoted value
    size_t quote1 = text.find("\"", pos);
    if (quote1 != std::string::npos) {
        size_t quote2 = text.find("\"", quote1 + 1);
        if (quote2 != std::string::npos) {
            return text.substr(quote1 + 1, quote2 - quote1 - 1);
        }
    }

    return "";
}

RecoveryStrategy_t FullParser::parseRecoveryStrategy(
    ExtentedHATPParser::Recovery_strategyContext* ctx)
{
    RecoveryStrategy_t strategy;

    // Since recovery_strategy() method doesn't exist in context, we parse from parent text
    // This parameter is kept for API compatibility but not used
    // Text parsing is done by parseCommitmentBlock which passes the full commitment text
    if (ctx == nullptr) {
        return strategy;  // Return default values
    }

    std::string text = ctx->getText();

    // Extract MODE
    auto extractQuotedValue = [](const std::string& text, const std::string& key) -> std::string {
        size_t pos = text.find(key + ":");
        if (pos != std::string::npos) {
            size_t quote1 = text.find("\"", pos);
            if (quote1 != std::string::npos) {
                size_t quote2 = text.find("\"", quote1 + 1);
                if (quote2 != std::string::npos) {
                    return text.substr(quote1 + 1, quote2 - quote1 - 1);
                }
            }
        }
        return "";
    };

    // Extract numeric value
    auto extractNumericValue = [](const std::string& text, const std::string& key) -> std::string {
        size_t pos = text.find(key + ":");
        if (pos != std::string::npos) {
            size_t colon_pos = text.find(":", pos);
            size_t semicolon_pos = text.find(";", colon_pos);
            if (colon_pos != std::string::npos && semicolon_pos != std::string::npos) {
                std::string num_str = text.substr(colon_pos + 1, semicolon_pos - colon_pos - 1);
                // Trim whitespace
                num_str.erase(0, num_str.find_first_not_of(" \t\n\r"));
                num_str.erase(num_str.find_last_not_of(" \t\n\r") + 1);
                return num_str;
            }
        }
        return "";
    };

    std::string mode = extractQuotedValue(text, "MODE");
    if (!mode.empty()) {
        strategy.mode = mode;
    }

    std::string attempts_str = extractNumericValue(text, "MAX_ATTEMPTS");
    if (!attempts_str.empty()) {
        try {
            strategy.max_attempts = std::stoi(attempts_str);
        } catch (...) {
            std::cerr << "Warning: Failed to parse MAX_ATTEMPTS value: " << attempts_str << std::endl;
        }
    }

    std::string timeout_str = extractNumericValue(text, "TIMEOUT");
    if (!timeout_str.empty()) {
        try {
            strategy.timeout = std::stod(timeout_str);
        } catch (...) {
            std::cerr << "Warning: Failed to parse TIMEOUT value: " << timeout_str << std::endl;
        }
    }

    return strategy;
}

} // procedural
