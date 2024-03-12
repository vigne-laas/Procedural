import os
import re
from pathlib import Path
import networkx as nx
import pydot

import matplotlib.pyplot as plt


def dot_to_networkx(file_path):
    pydot_graphs = pydot.graph_from_dot_file(file_path)

    # Pydot peut retourner une liste de graphes, nous traitons le premier graphe seulement
    if pydot_graphs and isinstance(pydot_graphs, list):
        pydot_graph = pydot_graphs[0]
    else:
        raise ValueError(f"Aucun graphe trouvé dans le fichier {file_path}")

    networkx_graph = nx.nx_pydot.from_pydot(pydot_graph)
    return networkx_graph


def display_graph(graph):
    plt.figure(figsize=(12, 8))
    pos = nx.spring_layout(graph)
    nx.draw(graph, pos, with_labels=True, node_color='lightblue', node_size=500, font_size=8, font_weight='bold',
            edge_color='gray')
    edge_labels = nx.get_edge_attributes(graph, 'label')
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=edge_labels, font_size=8)
    plt.title("Graph Visualisation")
    plt.show()


def display_multigraph(graph):
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


def parcourir_transitions(graph):
    for u, v, data in graph.edges(data=True):
        # Imprimer les nœuds de départ (u) et d'arrivée (v)
        print(f"Transition de {u} à {v}")
        # Imprimer les attributs de l'arête, s'il y en a
        if data:
            print("Attributs de l'arête :", data['label'])


def trouver_noeud_final(graph):
    if not isinstance(graph, nx.DiGraph):
        raise ValueError("Le graphe doit être dirigé pour utiliser cette méthode.")

    for node in graph.nodes():
        if graph.out_degree(node) == 0:
            print("final node :", node)
            return node

    return None  # Aucun nœud final trouvé


def trouver_tous_les_chemins(graph, source, cible):
    try:
        paths = list(nx.all_simple_paths(graph, source, cible))
        # print("exemple de path :", paths[0])
        return paths
    except nx.NetworkXNoPath:
        print(f"Aucun chemin trouvé entre {source} et {cible}.")
        return []
    except KeyError as e:
        print(f"Le nœud spécifié n'existe pas dans le graphe : {e}")
        return []


def trouver_noeud_initial(graph):
    if not isinstance(graph, nx.DiGraph):
        raise ValueError("Le graphe doit être dirigé pour utiliser cette méthode.")

    for node in graph.nodes():
        if graph.in_degree(node) == 0:
            print("initial node :", node)
            return node

    return None  # Aucun nœud initial trouvé


def obtenir_infos_label(label):
    # Extraire le nom entre parenthèses
    nom_match = re.search(r'\((.*?)\)', label)
    nom = nom_match.group(1) if nom_match else "Inconnu"

    # Extraire les variables après "Vars : "
    vars_match = re.search(r'Vars : (.*)', label)
    vars = vars_match.group(1).split(',') if vars_match else []

    # Nettoyer les espaces et les guillemets supplémentaires
    vars = [var.strip().strip('"') for var in vars]

    return nom, vars


def obtenir_transitions_avec_infos(graph, chemin):
    transitions = []
    for i in range(len(chemin) - 1):
        u = chemin[i]
        v = chemin[i + 1]
        if graph.has_edge(u, v):
            data = graph.get_edge_data(u, v)
            edge_data = data[next(iter(data))]
            label = edge_data.get('label', '').strip('"')
            color = edge_data.get('color', 'Aucune couleur').strip('"')

            # Extraire les informations du label
            nom, vars = obtenir_infos_label(label)
            transitions.append((u, v, nom, vars, color))
        else:
            raise ValueError(f"Pas d'arête entre {u} et {v} - chemin invalide")
    return transitions


def obtenir_ensembles_variables_et_noms(transitions):
    ensemble_variables = set()
    ensemble_noms = set()
    for transition in transitions:
        _, _, nom, vars, _ = transition
        ensemble_noms.add(nom)
        ensemble_variables.update(vars)
    return ensemble_variables, ensemble_noms


def read_dot_files_recursively(folder_path):
    graphs = {}
    for file_path in Path(folder_path).rglob('*.dot'):
        graph = dot_to_networkx(file_path)

        # Construire la clé en utilisant le nom du dossier parent et le nom du dossier contenant le fichier
        parent_folder_name = file_path.parent.parent.name
        containing_folder_name = file_path.parent.name
        name = f"{parent_folder_name}_{containing_folder_name}"
        key = f"{parent_folder_name}"

        graph.graph["name"] = name
        initial_node = trouver_noeud_initial(graph)
        final_node = trouver_noeud_final(graph)
        paths = trouver_tous_les_chemins(graph, initial_node, final_node)
        t = obtenir_transitions_avec_infos(graph, paths[0])
        # print(t[0][2])
        set_var, set_nom = obtenir_ensembles_variables_et_noms(t)
        print(set_var, set_nom)
        if key not in graphs:
            graphs[key] = []
        graphs[key].append(graph)

        # display_multigraph(graph)
    # Trier chaque liste de graphes dans le dictionnaire
    for key in graphs:
        graphs[key] = sorted(graphs[key], key=lambda g: g.graph['name'])

    return graphs


if __name__ == '__main__':
    # Le reste de votre code d'application Qt...

    # Utiliser la fonction sur un dossier spécifique
    folder_path = '/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot'
    graphs_dict = read_dot_files_recursively(folder_path)
    print(graphs_dict)
    # for folder_name, graphs in graphs_dict.items():
    #     print(f"Graphes du dossier {folder_name}:")
    # for graph in graphs:
    #     print(graph.graph["name"])
    # parcourir_transitions(graph)
    # print("---")

    # input()
