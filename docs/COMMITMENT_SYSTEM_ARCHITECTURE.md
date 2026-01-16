# Architecture du Système de Commitment
*Mise à jour: 2026-01-16*

## Vue d'Ensemble

Le système de commitment permet de définir des conditions que le robot doit surveiller pendant l'exécution d'actions, avec attribution automatique de responsabilité en cas de violation.

---

## 🏗️ Architecture Complète

```
┌─────────────────────────────────────────────────────────────────┐
│                    DOMAIN FILE (.dom)                          │
│                                                                  │
│  ACTION MyAction(Client C) {                                    │
│    COMMITMENTS {                                                │
│      INSTRUMENTAL {                                             │
│        "Battery OK" FOR robot { SELECT ... };                   │
│      };                                                         │
│      ENGAGEMENT {                                               │
│        "Client follows" FOR ?C { SELECT ... };                  │
│      };                                                         │
│    };                                                           │
│  };                                                             │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│               PARSER LAYER (FullParser + ANTLR4)               │
│                                                                  │
│  1. ExtentedHATPLexer.g4  → Tokenize                           │
│  2. ExtentedHATPParser.g4 → Parse structure                    │
│  3. FullParser.cpp        → Extract commitments                │
│     - parseCommitmentBlock()                                    │
│     - parseConditionsWithFor()  ← Parse FOR clause             │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│             C++ STRUCTURES (ParsedHTN.h)                       │
│                                                                  │
│  CommitmentCondition_t {                                        │
│    std::string description;                                     │
│    std::string sparql_query;                                    │
│    std::string for_clause;     ← Stored here                   │
│  }                                                              │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│          ROS CONVERSION (CommitmentConverter.h)                │
│                                                                  │
│  convertCondition() {                                           │
│    msg.for_clause = condition.for_clause;  ← Preserved         │
│  }                                                              │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│         ROS MESSAGE (CommitmentCondition.msg)                  │
│                                                                  │
│  string for_clause  # Who/what to monitor                      │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│    RESPONSIBILITY ANALYZER (ResponsibilityAnalyzer.h)          │
│                                                                  │
│  analyzeForClause(for_clause) {                                │
│    if (for_clause == "robot") return SELF;                     │
│    if (isParameter(for_clause)) return PARTNER;                │
│    if (for_clause == "environment") return ENVIRONMENT;        │
│    return UNCLEAR;                                             │
│  }                                                              │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📝 Syntaxe des Commitments

### Structure Générale

```
ACTION ActionName(Type Param) {
    PRECONDITIONS { ... };
    EFFECTS { ... };
    EXECUTION { ... };

    COMMITMENTS {
        INSTRUMENTAL {
            "Description" FOR target {
                SELECT * WHERE { ... }
            };
        };

        ENGAGEMENT {
            "Description" FOR target {
                SELECT * WHERE { ... }
            };
        };

        COMMON_GROUND {
            "Description" FOR target {
                SELECT * WHERE { ... }
            };
        };

        ON_INSTRUMENTAL_FAILURE: "recovery_action";
        ON_ENGAGEMENT_FAILURE: "recovery_action";
        ON_COMMON_GROUND_FAILURE: "recovery_action";

        RECOVERY_STRATEGY {
            MODE: "retry";
            MAX_ATTEMPTS: 3;
            TIMEOUT: 30.0;
        };
    };
};
```

### Cibles FOR (Attribution de Responsabilité)

| Syntaxe FOR | Attribution | Confidence | Description |
|-------------|-------------|------------|-------------|
| `FOR robot` | **SELF** | 0.9 | Robot responsable |
| `FOR ?C` (paramètre) | **PARTNER** | 0.9 | Partenaire/client responsable |
| `FOR environment` | **ENVIRONMENT** | 0.9 | Environnement responsable |
| `FOR both(robot, ?C)` | **UNCLEAR** | 0.5 | Responsabilité partagée |
| *(absent)* | **UNCLEAR** | 0.3 | Non spécifié |

---

## 🔧 Composants Clés

### 1. FullParser (C++)

**Fichier**: `src/memory/FullParser.cpp`

**Méthodes importantes**:
```cpp
// Parse le bloc COMMITMENTS complet
CommitmentBlock_t parseCommitmentBlock(ExtentedHATPParser::CommitmentsContext* ctx);

// Parse les conditions avec clause FOR
std::vector<CommitmentCondition_t> parseConditionsWithFor(
    const std::string& text,
    const std::string& condition_type  // "INSTRUMENTAL", "ENGAGEMENT", "COMMON_GROUND"
);
```

**Regex utilisé**:
```cpp
// Capture: "description" [FOR for_clause] { SPARQL query }
std::regex condition_regex(R"(\"([^\"]+)\"\s*(?:FOR\s+([^{]+))?\s*\{([^}]+)\})");
```

### 2. CommitmentConverter (C++)

**Fichier**: `include/procedural/memory/CommitmentConverter.h`

**Rôle**: Convertit les structures C++ en messages ROS

```cpp
static procedural_interfaces::CommitmentInfo convertToRosMessage(
    const CommitmentBlock_t& commitment_block,
    const std::string& action_name
);

static procedural_interfaces::CommitmentCondition convertCondition(
    const CommitmentCondition_t& condition,
    const std::string& type
);
```

### 3. ResponsibilityAnalyzer (C++)

**Fichier**: `include/procedural/ResponsibilityAnalyzer.h`

**Rôle**: Analyse la clause FOR pour attribuer la responsabilité

```cpp
struct AttributionResult {
    std::string attribution;     // "SELF", "PARTNER", "ENVIRONMENT", "UNCLEAR"
    float confidence;            // 0.0 to 1.0
    std::string failure_details; // Explication
};

AttributionResult analyzeForClause(
    const std::string& for_clause,
    const std::vector<std::string>& action_parameters = {}
) const;
```

**Logique d'attribution**:
1. Normalise l'identifiant (enlève `?` des paramètres)
2. Compare avec "robot" → SELF
3. Vérifie si c'est un paramètre d'action → PARTNER
4. Compare avec "environment" → ENVIRONMENT
5. Sinon → UNCLEAR

---

## 🚀 Utilisation

### Exemple 1: Commitment Simple

```
ACTION GuideClient(Client C) {
    PRECONDITIONS { robot isAvailable true; };
    EFFECTS { robot isGuiding C; };
    EXECUTION { /* ... */ };

    COMMITMENTS {
        INSTRUMENTAL {
            "Battery sufficient" FOR robot {
                SELECT * WHERE {
                    robot hasBatteryLevel ?level
                    FILTER(?level > 20)
                }
            };
        };

        ON_INSTRUMENTAL_FAILURE: "go_recharge";
    };
};
```

**Résultat**:
- Condition surveillée : niveau batterie > 20%
- Responsabilité : SELF (robot)
- En cas de violation : exécute action `go_recharge`

### Exemple 2: Commitment Multi-Agent

```
ACTION ServeFood(Client C, Dish D) {
    /* ... */
    COMMITMENTS {
        ENGAGEMENT {
            "Client stays at table" FOR ?C {
                SELECT * WHERE {
                    ?C isAt ?table
                }
            };
        };

        COMMON_GROUND {
            "Both agree on order" FOR both(robot, ?C) {
                SELECT * WHERE {
                    robot hasOrder ?o .
                    ?C hasExpected ?o
                }
            };
        };

        ON_ENGAGEMENT_FAILURE: "find_client";
        ON_COMMON_GROUND_FAILURE: "clarify_order";
    };
};
```

**Résultat**:
- Engagement : responsabilité CLIENT (paramètre ?C)
- Common ground : responsabilité PARTAGÉE
- Actions de récupération spécifiques selon le type

### Exemple 3: Commitment Environnement

```
ACTION NavigateToKitchen() {
    /* ... */
    COMMITMENTS {
        INSTRUMENTAL {
            "Path is clear" FOR environment {
                SELECT * WHERE {
                    NOT EXISTS { ?obs blockingPath true }
                }
            };
        };

        ON_INSTRUMENTAL_FAILURE: "wait_and_retry";

        RECOVERY_STRATEGY {
            MODE: "retry";
            MAX_ATTEMPTS: 5;
            TIMEOUT: 60.0;
        };
    };
};
```

**Résultat**:
- Responsabilité : ENVIRONMENT
- Stratégie : retry avec 5 tentatives max
- Timeout : 60 secondes

---

## 🧪 Tests

### Test Unitaire: ForClauseParsing_test

**Fichier**: `test/test_for_clause_parsing.cpp`
**Parser**: DomainReader (HATPListener)
**Status**: ✅ PASSÉ

```bash
cd /home/avigne/Projets/ArchiThese/catkin_ws
source devel/setup.bash
rosrun procedural ForClauseParsing_test
```

**Résultat attendu**:
```
✓ FOR clause: robot
✓ FOR clause: environment
✓ FOR clause: ?C
✓ FOR clause: both
[  PASSED  ] 1 test.
```

### Test Unitaire: FullParserCommitments_test

**Fichier**: `test/test_fullparser_commitments.cpp`
**Parser**: ProceduralFullReader (FullParser)
**Status**: ⚠️ EN COURS (problème syntaxe fichier test)

```bash
rosrun procedural FullParserCommitments_test
```

---

## 🐛 Debugging

### Activer les Logs de Parsing

Dans `FullParser.cpp`, les logs sont déjà actifs :

```cpp
std::cout << "  Parsing COMMITMENTS block..." << std::endl;
std::cout << "    Found " << commitment_block.instrumental.size() << " INSTRUMENTAL conditions" << std::endl;
std::cout << "      Parsed: \"" << condition.description << "\"";
if (!condition.for_clause.empty()) {
    std::cout << " FOR " << condition.for_clause;
}
```

### Vérifier la Propagation

1. **Parser** : Logs dans `parseConditionsWithFor()`
2. **Converter** : Vérifier `CommitmentConverter::convertCondition()`
3. **ROS Message** : `rostopic echo /procedural/actions`
4. **Analyzer** : Logs dans `analyzeForClause()`

### Syntaxe ANTLR4 Stricte

**Important** : Attention aux espaces !

❌ **Incorrect**:
```
COMMITMENTS {    // espace après COMMITMENTS
    INSTRUMENTAL {    // espace après INSTRUMENTAL
```

✅ **Correct**:
```
COMMITMENTS{    // pas d'espace
    INSTRUMENTAL{    // pas d'espace
```

**Exception** : `EXECUTION{ };` (espace entre braces pour éviter token `OpenCloseCurly`)

---

## 📚 Références

### Fichiers Clés

| Fichier | Rôle |
|---------|------|
| `include/procedural/memory/grammar/ExtentedHATPParser.g4` | Grammaire parser |
| `include/procedural/memory/grammar/ExtentedHATPLexer.g4` | Grammaire lexer |
| `src/memory/FullParser.cpp` | Parsing C++ |
| `include/procedural/memory/CommitmentConverter.h` | Conversion ROS |
| `include/procedural/ResponsibilityAnalyzer.h` | Attribution |
| `msg/CommitmentCondition.msg` | Interface ROS |
| `test/test_for_clause_parsing.cpp` | Test unitaire |

### Documentation Externe

- ANTLR4: https://www.antlr.org/
- SPARQL: https://www.w3.org/TR/sparql11-query/
- ROS Messages: http://wiki.ros.org/msg

---

## 🔄 Workflow Complet

```
1. Édition fichier .dom
   ↓
2. Compilation: catkin_make --pkg procedural
   ↓
3. Parsing au démarrage: FullParser charge le domaine
   ↓
4. Publication ROS: Actions avec commitments disponibles
   ↓
5. Monitoring runtime: Yggdrasil surveille les SPARQL queries
   ↓
6. Détection violation: Condition non satisfaite
   ↓
7. Attribution: ResponsibilityAnalyzer analyse FOR clause
   ↓
8. Récupération: Action ON_*_FAILURE exécutée
   ↓
9. Retry: RECOVERY_STRATEGY appliquée si configurée
```

---

## ✅ Checklist de Validation

Avant de déployer un nouveau fichier de domaine avec commitments :

- [ ] Syntaxe respectée (pas d'espaces entre mots-clés et accolades)
- [ ] Clause FOR spécifiée pour attribution claire
- [ ] SPARQL query valide
- [ ] Actions de récupération définies
- [ ] Test avec `ForClauseParsing_test` ou équivalent
- [ ] Vérification logs parsing (0 conditions trouvées = problème syntaxe)
- [ ] Test en conditions réelles avec robot

---

*Documentation générée le 2026-01-16*
*Pour questions: consulter COMMITMENT_SYSTEM_VALIDATION_2026-01-16.md*