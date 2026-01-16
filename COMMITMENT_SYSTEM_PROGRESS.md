# SYSTÈME DE COMMITMENTS UNIFIÉ - ÉTAT D'AVANCEMENT

**Date de dernière mise à jour**: 2025-11-07
**Phase actuelle**: 2/5 complétées (40%)
**Statut**: ✅ En cours - Parsing et conversion ROS fonctionnels

---

## 📊 RÉSUMÉ EXÉCUTIF

### Architecture choisie
- **Approche unifiée**: Tout le parsing dans FullParser.cpp (ANTLR4)
- **Suppression des parsers redondants**: CommitmentParser.py sera retiré (Task 3)
- **Conversion centralisée**: CommitmentConverter pour ROS messages

### Progrès global
- ✅ **Task 1**: FullParser parsing des commitments avec FOR clause
- ✅ **Task 2**: Conversion ROS messages pour IPC
- ⏳ **Task 3**: Suppression Python parser + mise à jour hri_planning
- ⏳ **Task 4**: Implémentation recovery action execution
- ⏳ **Task 5**: Infrastructure de tests et validation end-to-end

---

## ✅ TÂCHES COMPLÉTÉES

### Task 1: Extension FullParser pour parsing commitments (TERMINÉ)

#### Implémentation
**Fichiers créés/modifiés:**
- `src/memory/FullParser.cpp` (lignes 734-741, 1856-2070)
  - `parseCommitmentBlock()`: Parse COMMITMENTS blocks complets
  - `parseConditionsWithFor()`: Extraction FOR clause avec regex
  - `parseRecoveryAction()`: Mapping ON_*_FAILURE reactions
  - `parseRecoveryStrategy()`: Parse MODE, MAX_ATTEMPTS, TIMEOUT

- `include/procedural/memory/FullParser.h` (lignes 128-133)
  - Déclarations des nouvelles méthodes de parsing

- `include/procedural/memory/grammar/ExtentedHATPParser.g4` (ligne 17)
  - Ajout `commitments?` à la règle action
  - Règles commitments (lignes 57-67)

- `include/procedural/memory/grammar/ExtentedHATPLexer.g4` (lignes 58-70)
  - 13 nouveaux tokens: COMMITMENTS, INSTRUMENTAL, ENGAGEMENT, etc.

**Tests créés:**
- `test/test_fullparser_commitments.cpp`
- `test/test_commitment_with_for.dom`

**Patterns FOR clause supportés:**
```
FOR robot           → Responsabilité du robot
FOR ?C              → Variable d'argument
FOR environment     → Responsabilité environnement
FOR both(robot, ?C) → Responsabilité multiple
```

**Regex FOR clause:**
```regex
"([^"]+)"\s*(?:FOR\s+([^{]+))?\s*\{([^}]+)\}
```
Capture: (1) description, (2) FOR clause optionnelle, (3) requête SPARQL

#### Problèmes résolus
1. **ANTLR grammar missing**: Ajout des règles commitments au parser
2. **CommitmentsContext manquant**: Génération après modification .g4
3. **recovery_strategy() inexistant**: Parsing texte brut avec regex
4. **Namespace ambiguité**: Utilisation de `CommitmentBlock_t` directement (pas `task_recognition::`)

---

### Task 2: Conversion ROS Messages (TERMINÉ)

#### Implémentation
**Fichiers créés:**
- `include/procedural/memory/CommitmentConverter.h`
  - `convertToRosMessage()`: CommitmentBlock_t → CommitmentInfo
  - `convertCondition()`: CommitmentCondition_t → CommitmentCondition
  - Préserve FOR clause pour attribution de responsabilité

- `include/procedural/memory/ActionRosConverter.h`
  - `convertActionToRos()`: Wrapper qui étend toRosMsg()
  - Résout dépendance circulaire action_t.h ↔ ParsedHTN.h

**Fichiers modifiés:**
- `src/memory/ROS_Interfaces/MemoryRosInterface.cpp`
  - `getRobotActions()`: ligne 22 → `convertActionToRos()`
  - `getActions()`: ligne 121 → `convertActionToRos()`
  - `getActionDetails()`: ligne 159 → `convertActionToRos()`

- `include/procedural_interfaces/action_t.h`
  - Forward declaration: `namespace procedural { struct CommitmentBlock_t; }`
  - Champ: `std::shared_ptr<procedural::CommitmentBlock_t> commitments`
  - Note dans toRosMsg(): conversion déléguée à ActionRosConverter

#### Architecture de conversion
```
FullParser.cpp
    ↓ parse
CommitmentBlock_t (procedural)
    ↓
ActionRosConverter::convertActionToRos()
    ↓ uses
CommitmentConverter::convertToRosMessage()
    ↓
CommitmentInfo (ROS message)
    ↓
MemoryRosInterface → ROS services
```

#### Problèmes résolus
1. **Type incomplet dans action_t.h**: Forward declaration mal placée
2. **Namespace confusion**: `procedural::task_recognition` → `procedural`
3. **reinterpret_pointer_cast échec**: Assignment direct shared_ptr
4. **Circular dependency**: ActionRosConverter comme pont

#### Vérification compilation
```bash
cd /home/avigne/Projets/ArchiThese/catkin_ws
source devel/setup.bash
catkin_make --pkg procedural
```
✅ Compilation réussie sans erreurs

---

## ⏳ TÂCHES RESTANTES

### Task 3: Suppression Python Parser + Mise à jour hri_planning

**Objectif**: Éliminer CommitmentParser.py et utiliser uniquement FullParser

**Actions requises:**
1. **Identifier usages CommitmentParser.py**
   ```bash
   grep -r "CommitmentParser" src/hri_planning/
   grep -r "import.*commitment" src/hri_planning/
   ```

2. **Modifier hri_planning**
   - Remplacer appels CommitmentParser.py par services ROS
   - Utiliser `/getActionDetails` pour récupérer commitments
   - Parser commitment_info depuis message ROS

3. **Supprimer fichiers obsolètes**
   - `src/hri_planning/commitment_parser/CommitmentParser.py`
   - Tout autre fichier lié au parser Python

4. **Tests**
   - Vérifier que hri_planning fonctionne avec ROS services
   - Valider que commitments sont correctement reçus

**Fichiers à modifier:**
- `src/hri_planning/` (à identifier précisément)
- Possiblement `src/hri_planning/planner_node.py`

---

### Task 4: Implémentation Recovery Action Execution

**Objectif**: Système complet d'exécution des actions de récupération

**Composants à implémenter:**

#### 4.1 ResponsibilityAnalyzer
**Fichier**: `src/commitment_monitoring/ResponsibilityAnalyzer.cpp/h`

```cpp
class ResponsibilityAnalyzer {
    // Analyse FOR clause et détermine responsable
    enum Responsibility { SELF, PARTNER, ENVIRONMENT, BOTH };
    Responsibility analyzeForClause(const std::string& for_clause,
                                     const std::string& executor_id);
};
```

#### 4.2 RecoveryActionExecutor
**Fichier**: `src/commitment_monitoring/RecoveryActionExecutor.cpp/h`

```cpp
class RecoveryActionExecutor {
    // Execute recovery action selon stratégie
    bool executeRecoveryAction(const std::string& action_name,
                                const RecoveryStrategy& strategy);
    // Gère retry logic, timeout, max_attempts
};
```

#### 4.3 Integration dans CommitmentMonitor
**Fichier**: `src/commitment_monitoring/commitment_monitor_node.cpp`

- Lien avec ResponsibilityAnalyzer
- Appel RecoveryActionExecutor sur violation
- Respect de la stratégie (mode, max_attempts, timeout)

**Tests requis:**
- Test avec FOR SELF → exécution immédiate
- Test avec FOR PARTNER → skip si partner responsable
- Test retry strategy avec max_attempts
- Test timeout recovery

---

### Task 5: Infrastructure Tests et Validation End-to-End

**Objectif**: Tests automatisés + manuels complets

#### 5.1 Tests unitaires C++ (GTest)
**Fichiers à créer:**
- `test/test_commitment_conversion.cpp`
  - Vérifier conversion CommitmentBlock_t → CommitmentInfo
  - Tester préservation FOR clause
  - Valider recovery strategy

- `test/test_responsibility_analyzer.cpp`
  - Tester tous les patterns FOR
  - Cas edge: FOR clause vide, malformée

- `test/test_recovery_executor.cpp`
  - Tester retry logic
  - Tester timeout
  - Tester max_attempts

#### 5.2 Tests d'intégration ROS
**Scénarios:**
1. **Test parsing → ROS → monitoring**
   - Domaine avec commitments → FullParser
   - Récupération via `/getActionDetails`
   - Vérification du contenu ROS message

2. **Test recovery execution**
   - Simuler violation commitment
   - Vérifier appel recovery action
   - Valider respect stratégie

3. **Test multi-agents**
   - Commitments avec FOR both(robot, partner)
   - Vérifier attribution correcte

#### 5.3 Tests manuels
**Checklist:**
- [ ] Lancer procedural_memory_node
- [ ] Charger domain3.dom avec commitments
- [ ] Appeler `/getActionDetails GoToArea`
- [ ] Vérifier commitment_info présent et complet
- [ ] Lancer commitment_monitor_node
- [ ] Simuler violation → observer recovery action

---

## 📁 FICHIERS CLÉS

### Fichiers créés (Task 1-2)
```
/home/avigne/Projets/ArchiThese/catkin_ws/src/Procedural/
├── include/procedural/memory/
│   ├── CommitmentConverter.h              [NEW - Task 2]
│   ├── ActionRosConverter.h                [NEW - Task 2]
│   ├── FullParser.h                        [MODIFIED - Task 1]
│   └── grammar/
│       ├── ExtentedHATPParser.g4           [MODIFIED - Task 1]
│       └── ExtentedHATPLexer.g4            [MODIFIED - Task 1]
├── src/memory/
│   ├── FullParser.cpp                      [MODIFIED - Task 1-2]
│   └── ROS_Interfaces/
│       └── MemoryRosInterface.cpp          [MODIFIED - Task 2]
└── test/
    ├── test_fullparser_commitments.cpp     [NEW - Task 1]
    └── test_commitment_with_for.dom        [NEW - Task 1]

/home/avigne/Projets/ArchiThese/catkin_ws/src/procedural_interfaces/
└── include/procedural_interfaces/
    └── action_t.h                          [MODIFIED - Task 2]
```

### Fichiers à créer/modifier (Task 3-5)
```
À CRÉER:
- src/commitment_monitoring/ResponsibilityAnalyzer.cpp/h
- src/commitment_monitoring/RecoveryActionExecutor.cpp/h
- test/test_commitment_conversion.cpp
- test/test_responsibility_analyzer.cpp
- test/test_recovery_executor.cpp

À MODIFIER:
- src/hri_planning/ (fichiers Python utilisant CommitmentParser.py)
- src/commitment_monitoring/commitment_monitor_node.cpp

À SUPPRIMER:
- src/hri_planning/commitment_parser/CommitmentParser.py
```

---

## 🔧 COMMANDES UTILES

### Compilation
```bash
cd /home/avigne/Projets/ArchiThese/catkin_ws
source devel/setup.bash

# Package spécifique
catkin_make --pkg procedural

# Tous les packages
catkin_make
```

### Tests
```bash
# Tests unitaires C++
cd build/Procedural
./test_fullparser_commitments

# Test ROS services
rosservice call /getActionDetails "action_name: 'GoToArea'"
```

### Vérification parsing
```bash
# Logs FullParser
rosrun procedural procedural_memory_node -f domain3.dom
# Chercher: "✓ Parsed commitments for action"
```

---

## ⚠️ POINTS D'ATTENTION

### Problèmes résolus à ne pas réintroduire

#### 1. Namespace ambiguïté ANTLR
**Symptôme**: `error: reference to 'procedural' is ambiguous`
**Cause**: RobotActionParser.h et ExtentedHATPParser.h définissent tous deux `namespace procedural`
**Solution**: Utiliser `::procedural::` (global namespace) dans les fichiers source

**Fichier concerné**: `src/nodes/memory_node.cpp:13-19`

#### 2. Forward declaration incorrecte
**Symptôme**: `procedural_interfaces::procedural::task_recognition::CommitmentBlock_t` not found
**Cause**: Forward declaration dans mauvais namespace
**Solution**:
```cpp
// CORRECT
namespace procedural {
    struct CommitmentBlock_t;
}

// INCORRECT
namespace procedural { namespace task_recognition {
    struct CommitmentBlock_t;
}}
```

#### 3. Type incomplet dans conversion
**Symptôme**: `invalid use of incomplete type`
**Cause**: action_t.h ne peut pas inclure ParsedHTN.h (circular dependency)
**Solution**: ActionRosConverter comme pont, conversion où types sont complets

#### 4. SharedPlan API change
**Symptôme**: `SharedPlan has no member named 'nodes'`
**Cause**: API changée de `.nodes` à `.all_nodes`
**Solution**: Mise à jour dans RosMissionManager.cpp:1532,1533,1558

### Variables d'environnement
```bash
# Toujours sourcer avant compilation
source /home/avigne/Projets/ArchiThese/catkin_ws/devel/setup.bash
```

### CHANGELOG
Toujours mettre à jour le CHANGELOG.md après chaque tâche:
```
/home/avigne/Projets/ArchiThese/catkin_ws/src/Procedural/CHANGELOG.md
```

---

## 🎯 PROCHAINES ÉTAPES IMMÉDIATES

### Pour reprendre le travail:

1. **Lire ce document** pour se remettre en contexte

2. **Vérifier que la compilation fonctionne**
   ```bash
   cd /home/avigne/Projets/ArchiThese/catkin_ws
   source devel/setup.bash
   catkin_make --pkg procedural
   ```

3. **Commencer Task 3** - Suppression Python Parser
   - Identifier où CommitmentParser.py est utilisé
   - Planifier remplacement par ROS services
   - Créer plan avec ExitPlanMode
   - Exécuter après approbation utilisateur

4. **Tests après chaque tâche**
   - Compilation sans erreurs
   - Tests unitaires passent
   - Tests manuels ROS

---

## 📚 RÉFÉRENCES

### Spécifications originales
- Phase 7 Commitment System (2025-10-30)
- Architecture unifiée proposée (2025-11-07)

### Messages ROS pertinents
```
procedural_interfaces/CommitmentInfo.msg
procedural_interfaces/CommitmentCondition.msg
procedural_interfaces/Action.msg (champ: commitment_info)
```

### Services ROS utilisés
```
/getRobotActions         → Liste actions avec commitments
/getActions              → Liste actions filtrées
/getActionDetails        → Détails action incluant commitments
```

### Domain files de test
```
/home/avigne/Projets/ArchiThese/catkin_ws/src/Procedural/test/test_commitment_with_for.dom
```

---

## 📊 MÉTRIQUES

- **Lignes de code ajoutées**: ~500
- **Fichiers créés**: 4
- **Fichiers modifiés**: 8
- **Tests créés**: 1 suite
- **Temps estimé restant**: 3-4 tâches × 2-3h = 6-12h

---

**Dernière validation**: 2025-11-07
**Prochaine revue prévue**: Après Task 3
**Responsable**: Claude Code + Utilisateur
