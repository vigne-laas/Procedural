#ifndef FULLPARSER_H
#define FULLPARSER_H
#include "ExtentedHATPParserBaseListener.h"
#include <procedural_interfaces/action_t.h>

#include "procedural_interfaces/Practice.h"
#include "procedural_interfaces/PracticeFrame.h"
#include "procedural_interfaces/Role.h"
#include "procedural_interfaces/Priority.h"
#include "procedural_interfaces/task_t.h"
#include "procedural/task_recognition/Reader/domainTypes/ParsedHTN.h"
#include <set>
#include <queue>
#include <filesystem>
using namespace procedural_interfaces;

namespace procedural {

// Structure pour gérer les contextes de fichiers inclus
struct FileContext {
    std::string filepath;
    std::string package;
    std::string base_directory;
    bool is_processed;

    FileContext(const std::string& path, const std::string& pkg = "", const std::string& base_dir = "")
        : filepath(path), package(pkg), base_directory(base_dir), is_processed(false) {}
};

// Structure pour stocker le contenu parsé d'un fichier
struct ParsedFileContent {
    std::vector<Action_t> actions;
    std::vector<Practice*> practices;
    std::vector<Abstract_task_t> tasks;
    std::vector<PracticeFrame*> frames;
    std::vector<Role*> roles;
    std::vector<Priority*> priorities;
    std::string source_file;

    ParsedFileContent(const std::string& file = "") : source_file(file) {}
};

class FullParser : public ExtentedHATPParserBaseListener
{
public:
    void enterRoot(ExtentedHATPParser::RootContext* ctx) override;
    Actions_t getActions() const { return actions_; };
    std::vector<PracticeFrame*> getPracticeFrames() const { return practice_frames_; };
    std::vector<Practice*> getPractices() const
    {
        std::vector<Practice*> practices;
        for (const auto& [name, practice]: practices_)
        {
            practices.push_back(practice);
        }
        return practices;
    }
    std::vector<Role*> getRoles() const
    {
        std::vector<Role*> roles;
        for (const auto& [name, role]: roles_)
        {
            roles.push_back(role);
        }
        return roles;
    }
    std::vector<Priority*> getPriorities() const
    {
        std::vector<Priority*> priorities;
        for (const auto& [name, priority]: priorities_)
        {
            priorities.push_back(priority);
        }
        return priorities;
    }
    std::vector<Abstract_task_t> getTasks() const
    {
        std::vector<Abstract_task_t> tasks;
        for (const auto& [name, task]: tasks_)
        {
            tasks.push_back(task);
        }
        return tasks;
    }

    // Nouvelle interface pour parser un fichier avec inclusions
    void parseFileWithInclusions(const std::string& filepath);

    // Méthodes pour le système d'inclusion
    void setCurrentDirectory(const std::string& directory) { current_directory_ = directory; }

private:
    void displayResult();
    void linkPracticesToFrames();
    void linkRolesToPractices();
    void linkRolesToFrames();
    void debugPrintRolesState(const std::string& phase);


    void parseActionBloc(ExtentedHATPParser::Actions_blocContext* ctx);

    std::string parseDescriptionPracticeBloc(
        const std::vector<ExtentedHATPParser::Description_practiceContext*>& vector);
    Practice* parsePractice(ExtentedHATPParser::PracticeContext* practice_context);
    PracticeFrame* parsePracticeFrame(ExtentedHATPParser::Practice_frameContext* frame);
    Action_t parseAction(ExtentedHATPParser::ActionContext* ctx);
    std::vector<Argument_t> parseArguments(std::vector<ExtentedHATPParser::ArgumentsContext*> ctx);
    std::vector<Execution_action_t> parseExecutionBloc(ExtentedHATPParser::Execution_blocContext* ctx);
    Execution_action_t parseExecutionAction(ExtentedHATPParser::Exec_actionContext* ctx);
    Execution_argument_t parseExecutionArgument(ExtentedHATPParser::Exec_action_argContext* ctx);
    Description_t parseDescriptionBloc(ExtentedHATPParser::Description_blocContext* ctx);
    Triplet_t parseTriplet(ExtentedHATPParser::TripletContext* ctx);
    TripletVariable_t parseVariable(ExtentedHATPParser::SubjectContext* ctx);
    TripletVariable_t parseVariable(ExtentedHATPParser::ObjectContext* ctx);
    Attente& parseAttente(ExtentedHATPParser::AttenteContext* attente);
    Attente& parseAttenteExtended(ExtentedHATPParser::Attente_extendedContext* attente);
    Capacite* parseCapacite(ExtentedHATPParser::CapaciteContext* ctx);
    Role* parseRole(ExtentedHATPParser::RoleContext* ctx);
    Priority* parsePriority(ExtentedHATPParser::PriorityContext* ctx);
    void parseTasksBloc(ExtentedHATPParser::Tasks_blocContext* ctx);
    Abstract_task_t parseTask(ExtentedHATPParser::TaskContext* ctx);
    Method_t parseMethod(ExtentedHATPParser::MethodContext* ctx);
    std::vector<ActionPrecondition_t> parseActionPreconditionsBloc(ExtentedHATPParser::Preconditions_blocContext* ctx);
    std::vector<ActionEffect_t> parseActionEffectsBloc(ExtentedHATPParser::Effects_blocContext* ctx);
    double parseActionDurationBloc(ExtentedHATPParser::Duration_blocContext* ctx);
    Recognition_t parseRecognitionBloc(ExtentedHATPParser::Recognition_blocContext* ctx);

    // Nouvelles méthodes pour le système d'inclusion
    void processIncludes(ExtentedHATPParser::Include_blocContext* include_bloc);
    void processIncludeFile(const FileContext& file_context);
    ParsedFileContent parseFile(const std::string& filepath);
    std::string resolveFilePath(const std::string& path, const std::string& package = "", const std::string& base_dir = "");
    void mergeElements();
    Action_t mergeActionDefinitions(const std::vector<Action_t>& actions);
    Practice* mergePracticeDefinitions(const std::vector<Practice*>& practices);
    Abstract_task_t mergeTaskDefinitions(const std::vector<Abstract_task_t>& tasks);
    PracticeFrame* mergePracticeFrameDefinitions(const std::vector<PracticeFrame*>& frames);
    std::string getPackagePath(const std::string& package_name);
    std::string getActionSignature(const Action_t& action);
    std::string getTaskSignature(const Abstract_task_t& task);
    void processIncludeFileRecursively(const std::string& filepath, std::set<std::string>& processing_stack);


    Actions_t actions_;
    std::vector<PracticeFrame*> practice_frames_;
    std::map<std::string, Practice*> practices_;
    std::map<std::string, Role*> roles_;
    std::map<std::string, Priority*> priorities_;
    std::map<std::string, Abstract_task_t> tasks_;

    // Variables pour le système d'inclusion
    std::string current_directory_;
    std::set<std::string> processed_files_;
    std::queue<FileContext> files_to_process_;
    std::vector<ParsedFileContent> parsed_files_;

    // Maps temporaires pour la fusion d'éléments
    std::map<std::string, std::vector<Action_t>> pending_actions_;
    std::map<std::string, std::vector<Practice*>> pending_practices_;
    std::map<std::string, std::vector<Abstract_task_t>> pending_tasks_;
    std::map<std::string, std::vector<PracticeFrame*>> pending_frames_;
    std::map<std::string, std::vector<Role*>> pending_roles_;
    std::map<std::string, std::vector<Priority*>> pending_priorities_;
};
} // procedural

#endif //FULLPARSER_H
