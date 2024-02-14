import re

import networkx as nx


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
        self.args = self.parse_args()
        self.actions_correspondant_aux_graphes = set()
        self.actions_non_correspondantes = set()
        self.separer_actions(graph_names)

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

    def parse_args(self):
        if "args" in self.graph.nodes:
            node_label = self.graph.nodes["args"]['label']
            literals = [part.split(":")[0].strip() for part in node_label.split(",")]
            # print(f"literals {literals}")
            return literals
    pass
