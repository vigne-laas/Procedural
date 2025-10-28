# Changelog - Procedural Package

All notable changes to the Procedural package are documented in this file.

## [v0.1.0] - 2025-01-28

### Summary
This release consolidates the multi-agent action recognition system, parser inclusion capabilities, and enhanced procedural reasoning features developed over recent months.

### Key Features
- **Parser inclusion system**: Modular domain definitions with file imports
- **Multi-agent action recognition**: Complete refactoring for dedicated ontologies/timelines per agent
- **Memory-based readers**: Integration with procedural memory for dynamic domain loading
- **Enhanced state machine execution**: Extended graph structures and transitions
- **Complete launch configurations**: Ready-to-use multi-agent system setups

## 2025-10-15

### Added
- **`/recognition/library` topic publication** dans `action_recognition_multi_node.cpp`
  - Nouveau publisher `library_pub_` pour publier la bibliothèque des graphes de reconnaissance
  - Méthode `publishAggregatedLibrary()` qui agrège les bibliothèques de tous les agents
  - Publication automatique après la création de chaque agent
  - Utilise `action->getFactory()` pour obtenir le graphe factory de chaque action
  - Permet à l'UI Web de recevoir la liste complète des graphes disponibles

- **`getActionBuilder()` dans ActionRecognitionNode.h**
  - Nouvelle méthode publique pour accéder à l'ActionBuilder
  - Permet au MultiNodeManager d'accéder aux actions de chaque agent
  - Nécessaire pour l'agrégation de la bibliothèque multi-agents

### Fixed
- Correction du problème d'affichage vide dans l'onglet Recognition de l'UI Web
- L'UI peut maintenant recevoir et afficher les graphes de reconnaissance via `/recognition/library`
- Correction du type de conversion dans `publishAggregatedLibrary()` : utilise `action->getFactory()` au lieu de passer directement l'Action* à `convertGraphToMsg()`

## [Multi-Agent Architecture] - 2025-10-14

### Major Changes: Multi-Agent Action Recognition

Complete refactoring of the action recognition system to support multi-agent architectures with dedicated ontologies and timelines per agent.

#### Added

- **ActionRecognitionNode class** (`include/procedural/ActionRecognitionNode.h`, `src/ActionRecognitionNode.cpp`)
  - Reusable action recognition node for individual agents
  - Each instance operates on dedicated ontology and timeline
  - Callback-based architecture for centralized event publication
  - Subscribes to agent-specific Mementar topics (`/mementar/echo/{agent_name}`)
  - Public utility methods for graph/variable conversion

- **MultiNodeManager** (`src/nodes/action_recognition_multi_node.cpp`)
  - Central manager for multi-agent action recognition
  - Automatic agent detection via `/overworld/getAgents` service
  - Dynamic node creation for new agents via `/overworld/new_assessor` topic
  - Manages dedicated ontology and timeline per agent using `OntologiesManipulator` and `TimelinesManipulator`
  - Centralized publication of all recognition events
  - Thread-based execution (one thread per agent)
  - Aggregates state from all timelines for complete system view

- **Launch files**
  - `launch/action_recognition_multi.launch` - Multi-agent recognition node with configurable parameters
  - `launch/complete_action_recognition_multi.launch` - Complete system including all dependencies

- **Centralized ROS Topics** (published by MultiNodeManager)
  - `/recognition/state_changes` - State changes from all timelines
  - `/recognition/actions` - Recognized actions from all agents
  - `/recognition/descriptions` - Action descriptions
  - `/recognition/descriptions_str` - String format descriptions
  - `/recognition/active_graphs_state` - Aggregated state from all timelines

#### Modified

- **Message Interfaces** (requires `procedural_interfaces` recompilation)
  - Added `string timeline_id` field to `RecognitionStateChange.msg`
  - Added `string timeline_id` field to `RecognizedAction.msg`
  - Added `string timeline_id` field to `StateMachineGraph.msg`
  - Added `string[] timeline_ids` field to `ActiveGraphsState.msg`

- **CMakeLists.txt**
  - Added `overworld` dependency for agent management
  - Added `procedural_action_recognition_node_lib` library target
  - Added `action_recognition_multi_node` executable target
  - Proper linking of ontologenius and mementar libraries

- **package.xml**
  - Added `overworld` as build, build_export, and exec dependency

#### Fixed

- **Temporal precision**: Now uses `msg->stamp` from StampedFact instead of `ros::Time::now()` for accurate temporal tracking
- **Timeline isolation**: Each agent's facts are processed independently on dedicated timelines

#### Robustness Improvements

- **Exception handling in agent callbacks** (`onNewAgent`)
  - Added try-catch to prevent callback crashes from malformed messages
  - Logs full context including received message on exception

- **Exception handling in node creation** (`createNodeForAgent`)
  - Wrapped node creation in try-catch to handle ontology/timeline failures
  - System continues operating even if one agent fails to initialize
  - Clear error messages indicate which agent failed and why

- **Invalid message detection**
  - Logs warning when `/overworld/new_assessor` receives invalid format
  - Helps debug communication issues with overworld
  - Expected format: "ADD|agent_name"

#### Technical Details

**Architecture Pattern:**
- One `ActionRecognitionNode` instance per agent
- Each node runs in its own thread with dedicated resources
- Callback-based communication with `MultiNodeManager`
- Centralized state aggregation and publication

**Agent Lifecycle:**
1. Query existing agents from `/overworld/getAgents` on startup
2. Listen for new agents on `/overworld/new_assessor`
3. Create dedicated ontology via `OntologiesManipulator::add(agent_name)`
4. Create dedicated timeline via `TimelinesManipulator::add(agent_name)`
5. Subscribe to `/mementar/echo/{agent_name}` for agent-specific facts
6. Launch recognition thread for the agent

**Parameters** (ROS private namespace):
- `memory_service_namespace` (default: "procedural_memory") - Procedural memory service namespace
- `action_filter` (default: "") - Filter for specific actions
- `fact_time_to_live` (default: 10.0) - Fact TTL in seconds
- `buffer_max_size` (default: 1000) - Maximum fact buffer size
- `debug_mode` (default: false) - Enable debug logging

### Migration Notes

**Breaking Changes:**
- Old single-agent `action_recognition_node` still available for backward compatibility
- New multi-agent system requires `overworld` for agent management
- New multi-agent system requires Ontologenius and Mementar in multi mode

**Compatibility:**
- Shared `ActionBuilder` library unchanged (fully compatible)
- All existing domain files and action definitions remain valid
- Recognition patterns (SEQUENCE, PRECONDITIONS, EFFECTS) unchanged

### Dependencies

- `ontologenius` - Multi-agent ontology management
- `mementar` - Multi-agent timeline management
- `overworld` - Agent detection and management (NEW)
- `procedural_interfaces` - Updated message definitions

---

## Previous Versions

See git history for changes prior to multi-agent architecture refactoring.
