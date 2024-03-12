import random
from typing import List

from graphPublisher.dataStructure import GraphTask,Action,Sequences
from graphPublisher.reader import SequencesLoader, GraphReader


class ActionExecutor:
    def __init__(self, sequence_path, dot_folder):
        sequence_loader = SequencesLoader(sequence_path)
        self.graph_analyzer = GraphReader(dot_folder)
        self.sequences = Sequences(sequence_loader.get_args(), sequence_loader.get_sequence())
        self.__check_actions()
        self.callback = self.__default_callback

    def __default_callback(self, args):
        print(f"default callback : {args}")

    def __check_actions(self):
        for action in self.graph_analyzer.actions_non_graph_keys:
            methode = getattr(Action, action, None)
            if methode is None:
                print(f" action not found : {action}")
                raise IndexError(f"Action not found {action}")

    print("Check Actions Ok")

    def __link_variables(self, var_set, sequence_args, local_call=None):
        var_dict = {}
        remap = {}
        # print(local_call,sequence_args)
        for i in range(len(local_call)):
            remap[local_call[i]] = list(sequence_args.keys())[i]
        for var in var_set:
            if var in sequence_args.keys():
                var_dict[var] = sequence_args[var]
            elif var in self.sequences.variables.keys():
                var_dict[var] = self.sequences.variables[var]
            elif var in remap:
                sub_var = remap[var]
                if sub_var in sequence_args.keys():
                    var_dict[var] = sequence_args[sub_var]
                elif sub_var in self.sequences.variables.keys():
                    var_dict[var] = self.sequences.variables[sub_var]
                else:
                    raise IndexError(f"{var} not found in any dict local or global")
            else:
                raise IndexError(f"{var} not found in any dict local or global")
        return var_dict

    def __excute_graph(self, name: str, list_graph: List[GraphTask], sequence_args, id_graph=None, id_chemin=None):
        if name == "Get":
            id_graph = 1
        if id_graph is not None:
            graph = list_graph[id_graph]
        else:
            graph = random.choice(list_graph)
        set_var = graph.set_vars_globales
        dict_var = self.__link_variables(set_var, sequence_args, graph.args)
        # print(f"dictionnaire pour ce graph : {dict_var}")
        paths = graph.actions_par_chemin
        selected_path = random.choice(paths)
        # print(f"selected_path : {selected_path}")
        self.__play_path(selected_path, dict_var)

    def __play_path(self, path, dict_var):
        for action, var in path:
            # print(f"execute {action} with param : {local_var} ")
            if action in self.graph_analyzer.actions_graph_keys:
                self.__excute_sub_graph(action, var, dict_var)
            else:
                local_var = [dict_var[v] for v in var]
                self.__execute_action(action, local_var)

    def __excute_sub_graph(self, name, param, dict_var):
        # print(f"play inside subgraph : {name},{param}, {dict_var}")
        graphs = self.graph_analyzer.graphs[name]
        args = {arg: dict_var[arg] for arg in param}
        self.__excute_graph(name, graphs, args)
        pass

    def __execute_action(self, name, list_arg):
        # print(f"execute action {name} ({','.join(list_arg)})")
        method = getattr(Action, name)
        if method is not None:
            res = method(*list_arg)
            self.callback(res)
            # print(f"result of call method {name} : {res}")
        else:
            raise IndexError(f"action not found : {name}")

    def execute_actions(self):
        for action_name, action_args in self.sequences.actions:
            if action_name in self.graph_analyzer.graphs:
                print(f"Exécution du graphe {action_name}, with params : {action_args}")
                graphs = self.graph_analyzer.graphs[action_name]
                self.__excute_graph(action_name, graphs, action_args)
            else:
                print("Action must be execute")
                param = [self.graph_analyzer.variables[arg] for arg in action_args]
                self.__execute_action(action_name, param)

    def set_callback(self, callback):
        self.callback = callback


if __name__ == "__main__":
    folder_path = '/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot'
    yaml_file = '/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/src/tests/kitchen_domain/publisher.yaml'
    A = ActionExecutor(dot_folder=folder_path, sequence_path=yaml_file)
    A.execute_actions()
    pass