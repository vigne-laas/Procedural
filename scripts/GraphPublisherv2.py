import networkx as nx
import re
import random
import yaml
from pathlib import Path
import matplotlib.pyplot as plt
import pydot


class Action:
    @staticmethod
    def release(A, S, O):
        sequence = [f"[ADD]{A}|hasHandMovingToward|{S}", f"[DEL]{A}|isHolding|{O}"]
        return sequence

    @staticmethod
    def grasp(A, C):
        sequence = [f"[ADD]{A}|hasHandMovingToward|{C}", f"[ADD]{A}|isHolding|{C}"]
        return sequence

    @staticmethod
    def Pick_In(A, O, C):
        grasp_sequence = Action.grasp(A, O)
        grasp_sequence.append(f"[DEL]{O}|inIn|{C}")
        return grasp_sequence

    @staticmethod
    def Pick_Over(A, O, S):
        grasp_sequence = Action.grasp(A, O)
        grasp_sequence.append(f"[DEL]{O}|isOnTopOf|{S}")
        return grasp_sequence

    @staticmethod
    def Place_In(A, O, S):
        release_sequence = Action.release(A, S, O)
        release_sequence.append(f"[ADD]{O}|isInContainer|{S}")
        return release_sequence

    @staticmethod
    def Place_Over(A, O, S):
        release_sequence = Action.release(A, S, O)
        release_sequence.append(f"[ADD]{O}|isOnTopOf|{S}")
        return release_sequence

    @staticmethod
    def Give(A, A2, C):
        grasp_sequence = Action.grasp(A, C)
        grasp_sequence.append(f"[DEL]{A2}|isHolding|{C}")
        return grasp_sequence

    @staticmethod
    def SplitEggs(Y, W, EY, EW, E, A):
        instructions = [
            f"[ADD]{EY}|isIn|{Y}",
            f"[ADD]{EW}|isIn|{W}",
            f"[ADD]{A}|isHolding|{E}"
        ]
        return instructions

    @staticmethod
    def Dispose(A, B, S):
        instructions = [
            f"[ADD]{A}|hasInHand|{B}",
            f"[ADD]{S}|hasSpreadOver|{B}",
            f"[DEL]{B}|hasIn|~"
        ]
        return instructions

    @staticmethod
    def PoorBatter(A, M, B):
        instructions = [
            f"[ADD]{A}|hasInHand|{M}",
            f"[ADD]{B}|hasIn|{M}",
            f"[DEL]{M}|hasIn|~"
        ]
        return instructions

    @staticmethod
    def PoorMold(A, P, M):
        instructions = [
            f"[ADD]{A}|hasInHand|{P}",
            f"[ADD]{M}|hasIn|{P}",
            f"[DEL]{M}|hasIn|~"
        ]
        return instructions

    @staticmethod
    def Cut(A, K, I):
        instructions = [
            f"[ADD]{A}|hasInHand|{K}",
            f"[ADD]{I}|isUnder|{K}",
            f"[ADD]{I}|isCutBy|{K}"
        ]
        return instructions

    @staticmethod
    def Twist(A, F, C):
        instructions = [
            f"[ADD]{A}|hasInHand|{F}",
            f"[ADD]{F}|isHoldIn|{C}",
            f"[ADD]{A}|CircularMove|{F}"
        ]
        return instructions

# Importation des bibliothèques nécessaires


# Définition de la classe GraphTask pour manipuler les tâches de graph
class GraphTask:
    def __init__(self, graph, name, graph_names):
        self.graph = graph
        self.name = name
        self.graph.graph['name'] = name
        self.initial_node = self._find_initial_node()
        self.final_node = self._find_final_node()
        self.paths = self._find_all_paths()
        self.actions_per_path = [self._get_actions_and_vars(path) for path in self.paths]
        self.global_vars_set, self.global_actions_set = self._get_global_sets()
        self.graph_related_actions = set()
        self.non_related_actions = set()
        self._separate_actions(graph_names)

    def _find_initial_node(self):
        for node in self.graph.nodes():
            if self.graph.in_degree(node) == 0:
                return node
        return None

    def _find_final_node(self):
        for node in self.graph.nodes():
            if self.graph.out_degree(node) == 0:
                return node
        return None

    def _find_all_paths(self):
        if self.initial_node and self.final_node:
            return list(nx.all_simple_paths(self.graph, self.initial_node, self.final_node))
        return []

    def _get_actions_and_vars(self, path):
        actions_and_vars = []
        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            data = self.graph.get_edge_data(u, v)
            edge_data = data[next(iter(data))]
            label = edge_data.get('label', '').strip('"')
            name, vars = self._parse_label(label)
            actions_and_vars.append((name, vars))
        return actions_and_vars

    @staticmethod
    def _parse_label(label):
        name_match = re.search(r'\((.*?)\)', label)
        name = name_match.group(1) if name_match else "Unknown"
        vars_match = re.search(r'Vars : (.*)', label)
        vars = vars_match.group(1).split(',') if vars_match else []
        vars = [var.strip().strip('"') for var in vars]
        return name, vars

    def _get_global_sets(self):
        global_vars = set()
        global_actions = set()
        for path_actions in self.actions_per_path:
            for action, vars in path_actions:
                global_actions.add(action)
                global_vars.update(vars)
        return global_vars, global_actions

    def _separate_actions(self, graph_names):
        for action in self.global_actions_set:
            if action in graph_names:
                self.graph_related_actions.add(action)
            else:
                self.non_related_actions.add(action)

    def display_info(self):
        print(f"Graph name: {self.name}")
        print(f"Global variables: {self.global_vars_set}")
        print(f"Global actions/tasks: {self.global_actions_set}")
        print(f"Graph-related tasks: {self.graph_related_actions}")
        print(f"Non-graph actions: {self.non_related_actions}\n")
        print("Possible paths:")
        for i, path in enumerate(self.paths, 1):
            print(f"  Path {i}: {path}")
        print("Actions and variables per path:")
        for i, actions_vars in enumerate(self.actions_per_path, 1):
            print(f"  Path {i}: {actions_vars}")
        print("----")

    def execute_graph(self, param, graph_analyzer, path_index=None):
        if not self.paths:
            print("No path available for execution.")
            return
        path = self.paths[path_index] if path_index is not None and path_index < len(self.paths) else random.choice(self.paths)
        actions_vars = self.actions_per_path[self.paths.index(path)]
        for action_name, _ in actions_vars:
            action_method = getattr(Action, action_name, None)
            if action_method:
                print(f"Executing {action_name} with parameters: {param}")
                result = action_method(*param)
                print("Sequence:", result)
                return True
            else:
                print(f"Action {action_name} does not exist in Action class.")
                graph_analyzer.execute_graph(action_name, param)
                return False

# Cette refactorisation inclut la simplification des noms de méthodes, l'optimisation de la recherche de nœuds et de chemins,
# et la centralisation du traitement des labels. Les détails d'implémentation spécifiques à l'exécution des actions et à l'affichage
# des informations restent inchangés pour préserver le fonctionnement attendu. Des modifications supplémentaires seront appliquées
# pour améliorer d'autres parties du code selon les principes énoncés précédemment.

# Refactorisation de GraphAnalyzer et ActionExecutor

class GraphAnalyzer:
    def __init__(self, folder_path):
        self.folder_path = folder_path
        self.graphs = {}
        self.graphs = self._load_graphs()
        self.global_vars_set, self.global_actions_set = self._compute_global_sets()

    def _load_graphs(self):
        graphs = {}
        graph_names = self._get_graph_names_initial()  # Utilisez une nouvelle méthode pour obtenir les noms initiaux
        for file_path in Path(self.folder_path).rglob('*.dot'):
            graph_nx = self._dot_to_networkx(file_path)
            graph_task = GraphTask(graph_nx, file_path.stem, graph_names)  # Utilisez les noms obtenus précédemment
            key = file_path.parent.name
            if key not in graphs:
                graphs[key] = []
            graphs[key].append(graph_task)
        return graphs

    def _dot_to_networkx(self, file_path):
        pydot_graphs = pydot.graph_from_dot_file(str(file_path))
        return nx.nx_pydot.from_pydot(pydot_graphs[0])

    def _get_graph_names(self):
        return set(graph.name for graph_list in self.graphs.values() for graph in graph_list)

    def _compute_global_sets(self):
        vars_set = set()
        actions_set = set()
        for graph_list in self.graphs.values():
            for graph in graph_list:
                vars_set.update(graph.global_vars_set)
                actions_set.update(graph.global_actions_set)
        return vars_set, actions_set

    def display_global_info(self):
        print(f"Global variables: {self.global_vars_set}")
        print(f"Global actions/tasks: {self.global_actions_set}")

    def execute_graph(self, graph_name, params, from_action=False):
        for graph_list in self.graphs.values():
            for graph in graph_list:
                if graph.name == graph_name:
                    if not graph.execute_graph(params, self) and from_action:
                        print(f"Graph {graph_name} called another graph. Executing...")
                    return
        if from_action:
            print(f"Executing action {graph_name} as it's not a graph.")
            action_method = getattr(Action, graph_name, None)
            if action_method:
                action_method(*params)
            else:
                print(f"Action {graph_name} not found.")


class ActionExecutor:
    def __init__(self, yaml_file, graph_analyzer):
        self.variables = {}
        self.actions = []
        self.graph_analyzer = graph_analyzer
        self._load_yaml(yaml_file)

    def _load_yaml(self, yaml_file):
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
            self._parse_args(data.get('Args', []))
            self._parse_sequence(data.get('Sequence', []))

    def _parse_args(self, args):
        self.variables = {arg['name']: arg['value'] for arg in args}

    def _parse_sequence(self, sequence):
        self.actions = [(action['name'], [self.variables[param] for param in action['params']]) for action in sequence]

    def display_actions_and_variables(self):
        print("Variables:")
        for var, value in self.variables.items():
            print(f"  {var}: {value}")
        print("\nActions:")
        for action_name, params in self.actions:
            print(f"  Action: {action_name}, Params: {params}")

    def execute_actions(self):
        for action_name, params in self.actions:
            if action_name in self.graph_analyzer.graphs or action_name in self.graph_analyzer.global_actions_set:
                print(f"Executing graph or action {action_name} with params: {params}")
                self.graph_analyzer.execute_graph(action_name, params, from_action=True)
            else:
                print(f"Action or Graph {action_name} not defined or not found.")


def main():
    folder_path = '/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot'  # À modifier selon le chemin réel
    yaml_file = '/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/src/tests/kitchen_domain/publisher.yaml'  # À modifier selon le chemin réel

    analyzer = GraphAnalyzer(folder_path)
    analyzer.display_global_info()

    executor = ActionExecutor(yaml_file, analyzer)
    executor.display_actions_and_variables()
    executor.execute_actions()

if __name__ == "__main__":
    main()