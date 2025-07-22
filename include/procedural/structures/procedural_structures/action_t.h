// #ifndef PROCEDURAL_ACTION_T_H
// #define PROCEDURAL_ACTION_T_H
// #include <string>
// #include <vector>
// #include <map>
// namespace procedural {
//
// struct Argument_t {
//   std::string type;
//   std::string varname;
//
//   friend std::ostream& operator<<(std::ostream& os, const Argument_t& arg) {
//     return os << arg.to_string();
//   }
//
//   std::string to_string() const {
//     return "{" + type + ", " + varname + "}";
//   }
// };
//
// struct Execution_argument_t {
//   std::string type;
//   std::string value;
//   std::map<std::string, std::string> json;
//
//   friend std::ostream& operator<<(std::ostream& os, const Execution_argument_t& exec_arg) {
//     return os << exec_arg.to_string();
//   }
//
//   std::string to_string() const {
//     std::string json_str = "{";
//     for (const auto& pair : json) {
//       json_str += pair.first + ": " + pair.second + ", ";
//     }
//     if (!json.empty()) json_str.pop_back(), json_str.pop_back();
//     json_str += "}";
//     return "{" + type + ", " + value + ", " + json_str + "}";
//   }
// };
//
// struct Execution_action_t {
//   std::string name;
//   std::vector<Execution_argument_t> arguments;
//
//   friend std::ostream& operator<<(std::ostream& os, const Execution_action_t& exec_action) {
//     return os << exec_action.to_string();
//   }
//
//   std::string to_string() const {
//     std::string args_str = "[";
//     for (const auto& arg : arguments) {
//       args_str += arg.to_string() + ", ";
//     }
//     if (!arguments.empty()) args_str.pop_back(), args_str.pop_back();
//     args_str += "]";
//     return "{" + name + ", " + args_str + "}";
//   }
// };
//
// struct Action_t {
//   std::string name;
//   std::vector<Argument_t> arguments;
//   std::vector<Execution_action_t> executions_bloc;
//
//   friend std::ostream& operator<<(std::ostream& os, const Action_t& action) {
//     return os << action.to_string();
//   }
//
//   std::string to_string() const {
//     std::string args_str = "[";
//     for (const auto& arg : arguments) {
//       args_str += arg.to_string() + ", ";
//     }
//     if (!arguments.empty()) args_str.pop_back(), args_str.pop_back();
//     args_str += "]";
//     std::string execs_str = "[";
//     for (const auto& exec : executions_bloc) {
//       execs_str += exec.to_string() + ", ";
//     }
//     if (!executions_bloc.empty()) execs_str.pop_back(), execs_str.pop_back();
//     execs_str += "]";
//     return "{" + name + ", " + args_str + ", " + execs_str + "}";
//   }
// };
//
// struct Actions_t {
//   std::vector<Action_t> actions;
//
//   friend std::ostream& operator<<(std::ostream& os, const Actions_t& actions) {
//     return os << actions.to_string();
//   }
//
//   std::string to_string() const {
//     std::string actions_str = "[";
//     for (const auto& action : actions) {
//       actions_str += action.to_string() + ", ";
//     }
//     if (!actions.empty()) actions_str.pop_back(), actions_str.pop_back();
//     actions_str += "]";
//     return "{" + actions_str + "}";
//   }
// };
//
//
// } // procedural
//
//
// #endif //PROCEDURAL_ACTION_T_H
