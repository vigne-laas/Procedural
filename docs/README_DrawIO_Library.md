# Guide d'utilisation de la Librairie DrawIO Procedural

## Installation de la Librairie

1. Ouvrez Draw.io dans votre navigateur ou l'application desktop
2. Cliquez sur **File** → **Open Library from** → **Device**
3. Sélectionnez le fichier `procedural_drawio_library.xml`
4. La librairie apparaîtra dans le panneau de gauche sous "Procedural Library"

## Éléments Disponibles

### Classes Core

#### Fact Class
- **Usage** : Représentation des faits RDF dans le système
- **Couleur** : Bleu clair (#e1f5fe)
- **Propriétés principales** :
  - `subject_` et `object_` : Paires string/Variable_t
  - `id_property_` : Identifiant de propriété 
  - `timestamp_` : Horodatage du fait

#### Variable_t Class  
- **Usage** : Variables avec valeurs liées dynamiquement
- **Couleur** : Violet clair (#f3e5f5)
- **Propriétés** : 
  - `value_` : Valeur numérique
  - `is_set_` : État de liaison

#### Observation Class
- **Usage** : Observations des capteurs avec contexte
- **Couleur** : Orange clair (#fff3e0)
- **Propriétés** :
  - `table_variables_` : Table des variables liées
  - `constraints_` : Contraintes HTN

### Structures Graphiques

#### Graph Class
- **Usage** : Container pour les nœuds d'états
- **Couleur** : Vert clair (#e8f5e8)
- **Relations** : Contient 1..* Node

#### Node Class
- **Usage** : Nœud dans le graphe d'états
- **Couleur** : Rose clair (#fce4ec)
- **Relations** : Contient 0..* Transition

#### Transition Class
- **Usage** : Transition entre nœuds  
- **Couleur** : Vert lime (#f9fbe7)
- **Propriétés** : Observation, contraintes

### Actions et États

#### Action Class
- **Usage** : Actions robotiques complètes
- **Couleur** : Jaune clair (#ffecb3)
- **Propriétés** : Machine à états, paramètres

#### State Class
- **Usage** : États dans machines d'actions
- **Couleur** : Vert teal (#e0f2f1)
- **Relations** : Contient ActionTransition

## Conventions de Modélisation

### Relations UML
- **Composition** : Flèche noire avec losange rempli
- **Agrégation** : Flèche noire avec losange vide  
- **Association** : Flèche simple
- **Cardinalité** : `1`, `0..1`, `1..*`, `0..*`

### Code Couleur
- **Bleu** : Données fondamentales (Fact, Practice)
- **Violet** : Variables et types
- **Orange** : Observations et contextes
- **Vert** : Structures de contrôle (Graph, State)
- **Rose** : Nœuds et éléments discrets
- **Jaune** : Actions et comportements

### Bonnes Pratiques

1. **Disposition** : Placer les classes parent en haut, enfants en bas
2. **Espacement** : Garder ~50px entre les éléments
3. **Alignement** : Aligner les connexions sur les centres des boîtes
4. **Nommage** : Utiliser les noms exacts des classes C++
5. **Groupement** : Regrouper par modules fonctionnels

## Exemples d'Utilisation

### Diagramme de Reconnaissance d'Action

```
ActionRecognition
       ↓ 1..*
     Action  
       ↓ 1..*
     State
       ↓ 0..*
ActionTransition
```

### Graphe d'États HTN

```
Graph ─1..* → Node ─0..* → Transition
  ↓                          ↓
 name                   Observation
```

### Structure de Fait

```
Fact ──→ Variable_t (subject)
  ↓
  └──→ Variable_t (object)  
  ↓
TimeStamp_t
```

## Personnalisation

Pour modifier la librairie :

1. Éditez le fichier XML avec un éditeur de texte
2. Modifiez les propriétés `fillColor` pour changer les couleurs
3. Ajustez `width` et `height` pour redimensionner
4. Rechargez la librairie dans Draw.io

## Export et Partage

- **Export SVG** : Pour intégration dans documentation
- **Export PNG** : Pour présentations  
- **Export PDF** : Pour impression
- **Partage XML** : Distribuer la librairie complète

La librairie est conçue pour maintenir la cohérence visuelle à travers tous les diagrammes du projet Procedural.