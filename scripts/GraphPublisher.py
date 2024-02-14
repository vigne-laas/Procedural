import re
import time
from pathlib import Path
from typing import Union, Dict, Tuple, List, Callable

import networkx as nx
import pydot
import matplotlib.pyplot as plt


# import rospy
# from ontologenius.msg import OntologeniusStampedString
# from ontologenius import Ontoros


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
    def Pick_In(A, C, O):
        grasp_sequence = Action.grasp(A, O)
        grasp_sequence.append(f"[DEL]{O}|isIn|{C}")
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
    def Twist(A, C, F):
        instructions = [
            f"[ADD]{A}|hasInHand|{F}",
            f"[ADD]{F}|isHoldIn|{C}",
            f"[ADD]{A}|CircularMove|{F}"
        ]
        return instructions


class GraphTask:
    def __init__(self, graph, name, graph_names):
        self.graph = graph
        self.name = name
        self.graph.graph['name'] = name
        self.initial_node = self.trouver_noeud_initial()
        self.final_node = self.trouver_noeud_final()
        self.paths = self.trouver_tous_les_chemins()
        self.actions_par_chemin = [self.obtenir_actions_et_vars(chemin) for chemin in self.paths]
        self.set_vars_globales, self.set_actions_globales = self.obtenir_sets_globaux()
        self.actions_correspondant_aux_graphes = set()
        self.actions_non_correspondantes = set()
        self.separer_actions(graph_names)
        self.output = None
        self.graph_analyzer = None

    def setGraphAnalyzer(self, graph_analyzer):
        self.graph_analyzer = graph_analyzer

    def trouver_noeud_initial(self):
        """Trouve le nœud initial dans le graphe."""
        if not isinstance(self.graph, nx.DiGraph):
            raise ValueError("Le graphe doit être dirigé pour utiliser cette méthode.")

        for node in self.graph.nodes():
            if self.graph.in_degree(node) == 0:
                # print("initial node :", node)
                return node

        return None  # Aucun nœud initial trouvé

    def trouver_noeud_final(self):
        """Trouve le nœud final dans le graphe."""
        if not isinstance(self.graph, nx.DiGraph):
            raise ValueError("Le graphe doit être dirigé pour utiliser cette méthode.")

        for node in self.graph.nodes():
            if self.graph.out_degree(node) == 0:
                # print("final node :", node)
                return node

        return None  # Aucun nœud final trouvé

    def trouver_tous_les_chemins(self):
        """Trouve tous les chemins simples entre le nœud initial et final."""
        if self.initial_node and self.final_node:
            return list(nx.all_simple_paths(self.graph, self.initial_node, self.final_node))
        else:
            return []

    def obtenir_infos_label(self, label):
        # Extraire le nom entre parenthèses
        nom_match = re.search(r'\((.*?)\)', label)
        nom = nom_match.group(1) if nom_match else "Inconnu"

        # Extraire les variables après "Vars : "
        vars_match = re.search(r'Vars : (.*)', label)
        vars = vars_match.group(1).split(',') if vars_match else []

        # Nettoyer les espaces et les guillemets supplémentaires
        vars = [var.strip().strip('"') for var in vars]

        return nom, vars

    def obtenir_actions_et_vars(self, chemin):
        """Extrait les actions et les variables pour un chemin donné."""
        actions_et_vars = []
        for i in range(len(chemin) - 1):
            u, v = chemin[i], chemin[i + 1]
            if self.graph.has_edge(u, v):
                data = self.graph.get_edge_data(u, v)
                edge_data = data[next(iter(data))]
                label = edge_data.get('label', '').strip('"')
                nom, vars = self.obtenir_infos_label(label)
                actions_et_vars.append((nom, vars))
            else:
                raise ValueError(f"Pas d'arête entre {u} et {v} - chemin invalide")
        return actions_et_vars

    def obtenir_sets_globaux(self):
        """Obtient les ensembles globaux des actions et des variables pour le graphe entier."""
        set_vars = set()
        set_actions = set()
        for chemin in self.actions_par_chemin:
            for action, vars in chemin:
                set_actions.add(action)
                set_vars.update(vars)
        return set_vars, set_actions

    def afficher_informations(self):
        """Affiche les informations du graphe, y compris son nom, les ensembles globaux et les chemins possibles."""
        print(f"Nom du graphe: {self.name}")
        print(f"Variables globales: {self.set_vars_globales}")
        print(f"Actions/Task globales: {self.set_actions_globales}")
        print(f"Task : {self.actions_correspondant_aux_graphes}")
        print(f"Actions : {self.actions_non_correspondantes}")
        print()
        print("Chemins possibles:")
        for i, chemin in enumerate(self.paths, 1):
            print(f"  Chemin {i}: {chemin}")
        print("Actions et variables par chemin:")
        for i, actions_vars in enumerate(self.actions_par_chemin, 1):
            print(f"  Chemin {i}: {actions_vars}")
        print("----")

    def separer_actions(self, graph_names):
        """Sépare les actions en deux sets basés sur leur correspondance avec les noms des graphes."""
        for action in self.set_actions_globales:
            if action in graph_names:
                self.actions_correspondant_aux_graphes.add(action)
            else:
                self.actions_non_correspondantes.add(action)

    def executer_graph(self, param, chemin_index=None, level=0):
        """Exécute les actions d'un chemin aléatoire ou sélectionné du graphe."""
        print("\t" + "\t" * level + f"Try to execute {self.name} with param: {param} , level : {level}")
        if not self.paths:
            print("Aucun chemin disponible pour l'exécution.")
            return

        # Sélectionner un chemin aléatoirement si aucun index n'est fourni, sinon utiliser l'index fourni
        if chemin_index is None:
            chemin = random.choice(self.paths)
        else:
            chemin = self.paths[chemin_index] if chemin_index < len(self.paths) else None

        if chemin is None:
            print("Index de chemin invalide.")
            return

        actions_vars = self.actions_par_chemin[self.paths.index(chemin)]
        print("actions_vars:", actions_vars)
        # Exécuter les actions du chemin sélectionné
        for nom_action, local_var in actions_vars:
            print("--------------------------------")
            print(f"\t sub part : {nom_action} : {local_var}")
            action_method = getattr(Action, nom_action, None)
            # local_param = [self.graph_analyzer.variables[arg] for arg in param]
            if action_method:
                local_param = list(param.values())
                if len(local_param) != len(local_var):
                    nb_values = len(local_param)
                    for i in range(nb_values, len(local_var)):
                        print(f"missing param : {local_var[i]}")
                        local_param.append(self.graph_analyzer.variables[local_var[i]])
                print(
                    f"Exécution de {nom_action} avec les paramètres :{param} local var : {local_var} local param : {local_param}")
                # Appeler la méthode avec des paramètres de test (ajustez selon vos besoins)
                result = action_method(*local_param)  # Exemple avec des paramètres arbitraires
                # print("sequence :", result)
                self.output(result)
                print("-------------------------")
            else:
                sub_executive_dict = self.get_exective_param(local_var, param)

                print(
                    f"{nom_action} est une tâche donc décomposition. {local_var},param {param}, local_param {sub_executive_dict}")
                if nom_action == "Get":
                    self.graph_analyzer.execute_graph(nom_action, sub_executive_dict, 0, level=level + 1)
                self.graph_analyzer.execute_graph(nom_action, sub_executive_dict, level=level + 1)

    def set_output_callback(self, callback):
        self.output = callback

    def get_exective_param(self, local_var, arg):
        res = {}
        for var in local_var:
            print("local var : ", var)
            if var in arg.keys():
                print(f"Add {var} from local_executive context  value :{arg[var]}")
                res[var] = arg[var]
            else:
                print(f"Add {var} from global context value : {self.graph_analyzer.variables[var]}")
                res[var] = self.graph_analyzer.variables[var]
        return res


import random


class GraphAnalyzer:
    def __init__(self, folder_path):
        self.variables = {}
        self.folder_path = folder_path
        self.graphs: Dict[str, List[GraphTask]] = {}
        self.global_vars_set = set()
        self.global_actions_set = set()
        self.actions_non_graph_keys = set()  # Actions qui ne sont pas des clés du dictionnaire graph
        self.actions_graph_keys = set()  # Actions qui sont des clés du dictionnaire graph

    def run(self):
        """Exécute l'analyse des fichiers .dot dans le dossier spécifié."""
        self.graphs = self.read_dot_files_recursively(self.folder_path)
        self.calculer_ensembles_globaux()

    def obtenir_noms_graphs(self):
        """Retourne les noms de tous les graphes."""
        return {file_path.parent.parent.name for file_path in Path(self.folder_path).rglob('*.dot')}

    def dot_to_networkx(self, file_path):
        """Convertit un fichier .dot en un graphe NetworkX."""
        pydot_graphs = pydot.graph_from_dot_file(file_path)
        if pydot_graphs and isinstance(pydot_graphs, list):
            pydot_graph = pydot_graphs[0]
        else:
            raise ValueError(f"Aucun graphe trouvé dans le fichier {file_path}")
        return nx.nx_pydot.from_pydot(pydot_graph)

    def display_multigraph(self, graph):
        """Affiche un graphe multi-arêtes."""
        plt.figure(figsize=(12, 8))
        pos = nx.spring_layout(graph)

        # Dessiner les nœuds et les arêtes
        nx.draw(graph, pos, with_labels=True, node_color='lightblue', node_size=500, font_size=8, font_weight='bold',
                edge_color='gray')

        # Préparer les étiquettes pour les arêtes multiples
        edge_labels = {}
        for u, v, data in graph.edges(data=True):
            # Créer une clé unique pour chaque paire de nœuds
            edge_key = (u, v)
            label = data.get('label', '')  # Remplacer 'label' par l'attribut clé de vos arêtes

            # Ajouter ou concaténer les étiquettes
            if edge_key in edge_labels:
                edge_labels[edge_key] += ', ' + label
            else:
                edge_labels[edge_key] = label

        # Dessiner les étiquettes des arêtes
        nx.draw_networkx_edge_labels(graph, pos, edge_labels=edge_labels, font_size=8)

        plt.title("Graph Visualisation")
        plt.show(block=True)

    def read_dot_files_recursively(self, folder_path):
        """Lit récursivement les fichiers .dot dans un dossier et sous-dossiers."""
        graphs = {}
        graph_names = self.obtenir_noms_graphs()  # Assurez-vous d'implémenter cette méthode

        for file_path in Path(folder_path).rglob('*.dot'):
            graph_nx = self.dot_to_networkx(file_path)

            parent_folder_name = file_path.parent.parent.name
            containing_folder_name = file_path.parent.name
            name = f"{parent_folder_name}_{containing_folder_name}"
            key = parent_folder_name
            graph_info = GraphTask(graph_nx, name, graph_names)
            if key not in graphs:
                graphs[key] = []
            graphs[key].append(graph_info)

            # Optionnel: Afficher le graphe
            # self.display_multigraph(graph_nx)

        # Trier chaque liste de GraphInfo dans le dictionnaire
        for key in graphs:
            graphs[key] = sorted(graphs[key], key=lambda g: g.name)

        return graphs

    def calculer_ensembles_globaux(self):
        """Calcule les ensembles globaux de variables et d'actions pour tous les graphes."""
        for graph_list in self.graphs.values():
            for graph_task in graph_list:
                self.global_vars_set.update(graph_task.set_vars_globales)
                self.global_actions_set.update(graph_task.set_actions_globales)

        # Séparer les actions en deux sets : ceux qui sont des clés et ceux qui ne le sont pas
        for action in self.global_actions_set:
            if action in self.graphs:
                self.actions_graph_keys.add(action)
            else:
                self.actions_non_graph_keys.add(action)

    def afficher_ensembles_globaux(self):
        """Affiche les ensembles globaux de variables et d'actions."""
        print("Variables globales utilisées dans tous les graphes:", self.global_vars_set)
        print("Actions/Taches globales utilisées dans tous les graphes:", self.global_actions_set)
        print("Actions :", self.actions_non_graph_keys)
        print("Task :", self.actions_graph_keys)

    def execute_graph(self, graph_name, params, index=None, level=0):
        # Trouver et exécuter un graphe spécifique par son nom
        graph_task = self.find_graph_by_name(graph_name, index)
        if graph_task:
            graph_task.executer_graph(params, level=level)
        else:
            print(f"Graphe {graph_name} non trouvé.")

    def find_graph_by_name(self, name, graph_id=None) -> Union[GraphTask, None]:
        """Retourne aléatoirement ou par ID spécifique un GraphTask sous le nom donné."""
        if name in self.graphs:
            graph_list = self.graphs[name]

            if graph_id is not None:
                return graph_list[graph_id]
            else:
                # Sélection aléatoire
                return random.choice(graph_list)

        print(f"Aucun GraphTask trouvé sous le nom {name}.")
        return None

    def set_variables(self, variables: dict):
        self.variables = variables

    def set_callback(self, callback: Callable):
        for graphs in self.graphs.values():
            for graph in graphs:
                graph.set_output_callback(callback)
        pass


import yaml


class ActionExecutor:
    def __init__(self, yaml_file, dot_folder):
        self.debug = True
        self.variables = {}
        self.actions: List[Tuple[str, Dict[str, str]]] = []
        self.graph_analyzer = GraphAnalyzer(dot_folder)
        self.graph_analyzer.run()
        self.loadYaml(yaml_file)
        self.checkActions()
        self.link()

        # self.execute_actions()

    def displayAll(self):
        print("\n\n\n-------Variables load and sequence---------\n\n\n")
        self.displayVariablesAndAction()
        print("\n\n\n-------Display details---------\n\n\n")
        self.displayGraphDetails()
        print("\n\n\n--------Display Sets--------\n\n\n")
        self.displayAnalyserSets()
        print("\n\n\n----------------\n\n\n")

    def displayAnalyserSets(self):
        self.graph_analyzer.afficher_ensembles_globaux()

    def displayGraphDetails(self):
        for k, values in self.graph_analyzer.graphs.items():
            print("key :", k)
            for val in values:
                val.afficher_informations()

    def checkActions(self):
        for action in self.graph_analyzer.actions_non_graph_keys:
            methode = getattr(Action, action, None)
            if methode is None:
                print(f" action not found : {action}")
                raise IndexError(f"Action not found {action}")
            elif self.debug:
                print(f"{action} found")
        print("Check Actions Ok")

    def link(self):
        for graphs in self.graph_analyzer.graphs.values():
            for graph in graphs:
                graph.setGraphAnalyzer(self.graph_analyzer)

    def loadYaml(self, yaml_file):
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
            self.parseArgs(data.get('Args', []))
            self.parseSequence(data.get('Sequence', []))

    def parseArgs(self, args):
        for arg in args:
            for key, value in arg.items():
                self.variables[key] = value

    def parseSequence(self, sequence):
        for action in sequence:
            for action_name, params in action.items():
                action_params = {}
                for param in params:
                    for literal, value in param.items():
                        action_params[literal] = self.variables[value]
                self.actions.append((action_name, action_params))

    def displayVariablesAndAction(self):
        """Affiche les actions et les variables traitées."""
        print("Variables:")
        for var, value in self.variables.items():
            print(f"  {var}: {value}")
        print("\nActions:")
        for action_name, params in self.actions:
            print(f"  Action: {action_name}, Params: {params}")

    def executeActions(self):
        self.graph_analyzer.set_variables(self.variables)
        for action_name, args in self.actions:
            # executive_dict = {arg: self.variables[arg] for arg in args}
            # print("executive_dict: ", executive_dict)
            if self.graph_analyzer and action_name in self.graph_analyzer.graphs:
                # Si l'action correspond au nom d'un graphe, exécute ce graphe
                if self.debug:
                    print(f"Exécution du graphe {action_name}, with params : {args}")
                self.graph_analyzer.execute_graph(action_name, args)
            else:
                # Sinon, exécute l'action comme avant
                param = [self.variables[arg] for arg in args]
                if self.debug:
                    print(f"Execute directly : {action_name} with args : {args} so params : {param}")
                action_method = getattr(Action, action_name, None)
                if action_method:
                    result = action_method(*param)
                    print(result)
                else:
                    print(f"Action {action_name} not defined.")

    def setCallback(self, callback):
        self.graph_analyzer.set_callback(callback=callback)
        pass


list_msg = []


def print_result(msg):
    print(f"\t\t\t result : {msg}")
    list_msg.append(msg)


class OntologyPublisher:
    def __init__(self, name=None):
        # rospy.init_node("OntologyTaskPublisher", anonymous=False)
        # if name is not None:
        #     self.pub = rospy.Publisher("/ontologenius/insert_stamped/" + name, OntologeniusStampedString, queue_size=1)
        # else:
        #     self.pub = rospy.Publisher("/ontologenius/insert_stamped", OntologeniusStampedString, queue_size=1)
        pass

    def sendSequence(self, sequence):
        for s in sequence:
            print(f'publish : {s}')
            # stamp = Ontoros.getRosTime()
            # msg = OntologeniusStampedString(data=s, stamp=stamp)
            # self.pub.publish(msg)
            # time.sleep(0.1)
            pass


# Utilisation de la classe
if __name__ == '__main__':
    publisher = OntologyPublisher("pr2")
    folder_path = '/home/adrien/Robots/Procedural/catkin_ws/src/Procedural/dot'
    yaml_file = '/home/adrien/Robots/Procedural/catkin_ws/src/Procedural/src/tests/kitchen_domain/publisher.yaml'
    executor = ActionExecutor(yaml_file, folder_path)
    executor.displayAll()
    executor.setCallback(publisher.sendSequence)
    executor.executeActions()
