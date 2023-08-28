#ifndef PROCEDURAL_PARAMETERS_H
#define PROCEDURAL_PARAMETERS_H

#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>

namespace procedural
{

class Parameter
{
public:
  std::string name_;
  std::vector<std::string> options_;
  std::vector<std::string> values_;
  std::vector<std::string> default_values_;

  Parameter(const std::string& name, const std::vector<std::string>& options, const std::vector<std::string>& default_values = {}) : name_(name),
                                                                                                                                     options_(options),
                                                                                                                                     default_values_(default_values)
  {}

  Parameter(const Parameter& other) : name_(other.name_),
                                      options_(other.options_),
                                      values_(other.values_),
                                      default_values_(other.default_values_)
  {}

  void insert(const std::string& value) { values_.push_back(value); }

  std::string getFirst()
  {
    if(values_.size() == 0)
    {
      if(default_values_.size())
        return default_values_[0];
      else
        throw std::invalid_argument("No value for argument " + name_);
    }
    else
      return (values_.size() ? values_[0] : "");
  }

  std::vector<std::string> get()
  {
    if(values_.size() == 0)
      return default_values_;
    else
      return values_;
  }

  bool testOption(const std::string& option)
  {
    return std::any_of(options_.begin(), options_.end(), [option](auto op){ return option == op; });
  }

  void display()
  {
    std::cout << name_ << ":" << std::endl;

    if(values_.size())
    {
      for(auto value : values_)
        std::cout << "\t- " << value << std::endl;
    }
    else
    {
      for(auto value : default_values_)
        std::cout << "\t- " << value << std::endl;
    }
  }
};

class Parameters
{
private:
  std::map<std::string, Parameter> parameters_;
  std::string default_param_name_;
  std::string process_name_;

public:

  /// @brief Register a new parameter model in the parameter set
  /// @param param is the parameter object to insert
  void insert(const Parameter& param)
  {
    parameters_.insert(std::pair<std::string, Parameter>(param.name_,param));
    if(param.options_.size() == 0)
      default_param_name_ = param.name_;
  }

  /// @brief Returns the parameter object related to the name provided in argument
  /// @param parameter is the name of the parameter to get
  /// @return A copy of the parameter object
  Parameter at(const std::string& parameter)
  {
    return parameters_.at(parameter);
  }

  /// @brief Sets/Reads the values of the parameters
  /// @param argc the number of strings pointed to by argv
  /// @param argv is the array of arguments
  void set(int argc, char** argv)
  {
    process_name_ = std::string(argv[0]);
    size_t pose;
    while ((pose = process_name_.find("/")) != std::string::npos) {
      process_name_ = process_name_.substr(pose+1);
    }
    process_name_ = " " + process_name_ + " ";

    for(size_t i = 1; i < (size_t)argc; i++)
    {
      if(argv[i][0] == '-')
      {
        std::string param_name = "";
        for(auto param : parameters_)
          if(param.second.testOption(std::string(argv[i])))
          {
            param_name = param.second.name_;
            break;
          }

        if(param_name == "")
          std::cout << "unknow option " << std::string(argv[i]) << std::endl;
        else
        {
          if(i+1 < (size_t)argc)
          {
            i++;
            parameters_.at(param_name).insert(std::string(argv[i]));
          }
        }
      }
      else
      {
        if(default_param_name_ != "")
          parameters_.at(default_param_name_).insert(std::string(argv[i]));
        else
          std::cout << "No default parameter" << std::endl;;
      }
    }
  }

  /// @brief Displays the parameters names and setted values
  void display()
  {
    std::string delim = "****************";
    std::string delim_gap;
    for(size_t i = 0; i < process_name_.size(); i++)
      delim_gap += "*";
    std::cout << delim << process_name_ << delim << std::endl;
    for(auto param : parameters_)
      param.second.display();
    std::cout << delim << delim_gap << delim << std::endl;
  }
};

}

#endif // PROCEDURAL_PARAMETERS_H
