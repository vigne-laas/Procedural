
#include <fstream>
#include <filesystem>
#include "procedural/core/Types/Method.h"

namespace procedural {
Method::Method(const std::string& name, int id) : name_(name), id_(id), compt_SM_(0)
{
    factory_machine_ = new StateMachine(name, id);
}

void Method::addTransition(const HTNTransition_t& transition)
{
    factory_machine_->addTransition(transition);
}
void Method::close()
{
    factory_machine_->closeStateMachine();
}
std::string Method::getStrStructure()
{
    std::string res;
    res += "Structure of : " + factory_machine_->getName() + "\n\n";
    res += factory_machine_->getStrStructure() + "\n";
    return res;
}
EvolveResult_t Method::feed(StateMachine* state_machine_finished)
{
    EvolveResult_t res;
    bool evolve = false;
    for (auto& state_machine: current_state_machines_)
    {
        std::cout << "check if " << state_machine_finished->getName() << " can evolve method : " << name_;
        auto result = state_machine->evolve(state_machine_finished);
        switch (result.state)
        {

            case FeedResult::NO_EVOLUTION:
                break;
            case FeedResult::EVOLVE:
                std::cout << "\t succes of evolution  : " << state_machine->getName() << std::endl;
                evolve = true;
                if (res.state != FeedResult::FINISH)
                    res.state = FeedResult::EVOLVE;
                break;
            case FeedResult::FINISH:
                std::cout << "\t succes of evolution  : " << state_machine->getName() << "reach final state"
                          << std::endl;
                evolve = true;
                finished_state_machines_.insert(state_machine);
                res.state = FeedResult::FINISH;

                break;
        }
        if (result.update_available)
        {
            updated_states_machines.push_back(state_machine);
            res.update_available = true;
        }

    }
    if (evolve == false)
    {
//        std::cout << "Try to create new SM " << std::endl;
        StateMachine* new_net = factory_machine_->clone(-1);
        auto result = new_net->evolve(state_machine_finished);
        switch (result.state)
        {
            case FeedResult::NO_EVOLUTION:
                delete new_net;
                break;
            case FeedResult::EVOLVE:
                new_net->setId(getNextSMId());
//                    new_net->attach(this);
                current_state_machines_.insert(new_net);
                std::cout << "create new state machine " << new_net->getName() << std::endl;
                if (res.state != FeedResult::FINISH)
                    res.state = FeedResult::EVOLVE;
                break;
            case FeedResult::FINISH:
                new_net->setId(getNextSMId());
                std::cout << "Method finish" << name_ << std::endl;
                finished_state_machines_.insert(new_net);
                break;
        }

    }
    return res;
}
EvolveResult_t Method::feed(Task* task)
{
    EvolveResult_t res;
    bool evolve = false;
    for (auto& state_machine: current_state_machines_)
    {
        auto result = state_machine->evolve(task);
        switch (result.state)
        {

            case FeedResult::NO_EVOLUTION:
                break;
            case FeedResult::EVOLVE:
                std::cout << "\t succes of evolution  : " << state_machine->getName() << std::endl;
                evolve = true;
                if (res.state != FeedResult::FINISH)
                    res.state = FeedResult::EVOLVE;
                break;
            case FeedResult::FINISH:
                std::cout << "\t succes of evolution  : " << state_machine->getName() << "reach final state"
                          << std::endl;
                evolve = true;
                finished_state_machines_.insert(state_machine);
                res.state = FeedResult::FINISH;

                break;
        }
        if (result.update_available)
        {
            updated_states_machines.push_back(state_machine);
            res.update_available = true;
        }
    }

    if (evolve == false)
    {
        StateMachine* new_net = factory_machine_->clone(-1);
        auto result = new_net->evolve(task);
        switch (result.state)
        {
            case FeedResult::NO_EVOLUTION:
                delete new_net;
                break;
            case FeedResult::EVOLVE:
                new_net->setId(getNextSMId());
//                    new_net->attach(this);
                current_state_machines_.insert(new_net);
                std::cout << "create new state machine " << new_net->getName() << std::endl;
                if (res.state != FeedResult::FINISH)
                    res.state = FeedResult::EVOLVE;
                break;
            case FeedResult::FINISH:
                new_net->setId(getNextSMId());
                finished_state_machines_.insert(new_net);
                break;
        }

    }

    return res;
}
void Method::saveDot(int i, const std::string& suffix, bool partial, const std::string& path)
{
    std::ofstream dot_file;
    std::string folder_name = (path.empty()) ? "" : path + "/";
    folder_name += "dot/" + name_ + "/" + std::to_string(id_);
    std::filesystem::path full_folder_path = std::filesystem::absolute(folder_name);
    std::cout << "essaie creation : " << full_folder_path << std::endl;
    if (!std::filesystem::exists(full_folder_path))
    {
        // Crée le dossier s'il n'existe pas
        if (std::filesystem::create_directories(full_folder_path))
        {
            std::cout << "Dossier créé avec succès." << std::endl;
        } else
        {
            std::cerr << "Erreur lors de la création du dossier." << std::endl;
        }
    } else
    {
        std::cout << "Le dossier existe déjà." << std::endl;
    }


    if (partial)
    {

        std::string filename = folder_name + "/" + name_ + "_partial_step_" + std::to_string(i) +
                               (suffix.empty() ? "_" + suffix + "_" : "") + ".dot";
        std::filesystem::path full_path = std::filesystem::absolute(filename);

        dot_file.open(full_path);
        if (!dot_file.is_open())
        {
            std::cerr << "Error opening DOT file." << std::endl;
            return;
        }
        dot_file << "digraph G {" << std::endl;
        factory_machine_->saveDot(dot_file, partial);
        dot_file << "}" << std::endl;
        dot_file.close();
        std::cout << "DOT file generated successfully: " << full_path << std::endl;
    } else
    {
        std::string filename =
                folder_name + "/" + name_ + "_full_" + (suffix.empty() ? "_" + suffix + "_" : "") + ".dot";
        std::filesystem::path full_path = std::filesystem::absolute(filename);
        dot_file.open(full_path);
        if (!dot_file.is_open())
        {
            std::cerr << "Error opening DOT file." << std::endl;
            return;
        }
        dot_file << "digraph G {" << std::endl;
        factory_machine_->saveDot(dot_file, partial);

        dot_file << factory_machine_->getInitialState()->getFullName() << " [shape=Mdiamond];" << std::endl;
        dot_file << factory_machine_->getFinalState()->getFullName() << " [shape=Msquare];" << std::endl;
        dot_file << "}" << std::endl;
        dot_file.close();
        std::cout << "DOT file generated successfully: " << full_path << std::endl;

    }

}
} // procedural