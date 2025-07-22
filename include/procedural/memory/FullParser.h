#ifndef FULLPARSER_H
#define FULLPARSER_H
#include "ExtentedHATPParserBaseListener.h"
#include <procedural_interfaces/action_t.h>

#include "procedural_interfaces/Practice.h"
#include "procedural_interfaces/PracticeFrame.h"
using namespace procedural_interfaces;
namespace procedural {

class FullParser : public ExtentedHATPParserBaseListener {
public:
    void enterRoot(ExtentedHATPParser::RootContext* ctx) override;
    Actions_t getActions() const { return actions_; };
    std::vector<PracticeFrame*> getPracticeFrames() const { return practice_frames_; };
    std::vector<Practice*> getPractices() const {
        std::vector<Practice*> practices;
        for (const auto& [name, practice]: practices_) {
            practices.push_back(practice);
        }
        return practices;
    }
private:

    void displayResult();
    void linkPracticesToFrames();


    void parseActionBloc(ExtentedHATPParser::Actions_blocContext* ctx);

    std::string parseDescriptionPracticeBloc(const std::vector<ExtentedHATPParser::Description_practiceContext*>& vector);
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

    Actions_t actions_;
    std::vector<PracticeFrame*> practice_frames_;
    std::map<std::string, Practice*> practices_;

};

} // procedural

#endif //FULLPARSER_H
