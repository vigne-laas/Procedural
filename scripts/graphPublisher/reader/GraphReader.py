from pathlib import Path
from typing import Dict, List

import pydot
from matplotlib import pyplot as plt
import networkx as nx
from scripts.graphPublisher.dataStructure import GraphTask

class GraphReader:
    def __init__(self, folder_path):
        self.variables = {}
        self.folder_path = folder_path
        self.graphs: Dict[str, List[GraphTask]] = {}
        self.global_vars_set = set()
        self.global_actions_set = set()
        self.actions_non_graph_keys = set()  # Actions qui ne sont pas des clés du dictionnaire graph
        self.actions_graph_keys = set()  # Actions qui sont des clés du dictionnaire graph
        self.run()

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

