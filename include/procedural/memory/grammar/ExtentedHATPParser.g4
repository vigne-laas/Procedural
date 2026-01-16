parser grammar ExtentedHATPParser;
options {
	tokenVocab = ExtentedHATPLexer;
}


root: (include_bloc|comment|actions_bloc|attentes_bloc|tasks_bloc|priorities_bloc|pratices_frames_bloc|practices_bloc)+ EOF;


include_bloc: INCLUDE OpenCurly inclusion* CloseCurly SEMICOLON;
inclusion: STRING (package_link | link) STRING SEMICOLON;
package_link: PACKAGE COLON COLON link;
link: SLASH? IDENTIFIER (SLASH IDENTIFIER)* POINT IDENTIFIER;


actions_bloc: ACTIONS OpenCurly action* CloseCurly SEMICOLON;
action: ACTION name OpenPar arguments* (Comma arguments)* ClosePar OpenCurly preconditions_bloc? effects_bloc? (recognition_bloc|execution_bloc|description_bloc|cost_bloc|duration_bloc)+ commitments? CloseCurly SEMICOLON;

preconditions_bloc: PRECONDITIONS OpenCurly (query | triplet)* CloseCurly SEMICOLON;
query: SELECT (variable|TIMES|MINUS) (Comma variable)* WHERE OpenCurly where_clause CloseCurly;

where_clause: where_statement+;

where_statement: NOT OpenCurly triplet_with_dot+ CloseCurly POINT
               | triplet_with_dot;

triplet_with_dot: triplet_pattern POINT SPACE*;


triplet_pattern: subject source COLON predicate object;

triplet_query: (NOT)? subject source COLON predicate object  SEMICOLON;
source: IDENTIFIER;
effects_bloc: EFFECTS OpenCurly triplet* CloseCurly SEMICOLON;
cost_bloc: COST OpenCurly numeric_value SEMICOLON CloseCurly SEMICOLON;
duration_bloc: DURATION OpenCurly numeric_value SEMICOLON CloseCurly SEMICOLON;

description_bloc: DESCRIPTION OpenCurly triplet* CloseCurly SEMICOLON;
triplet: (NOT)? subject predicate object (REQUIRED)? SEMICOLON;

execution_bloc: EXECUTION OpenCurly exec_action* CloseCurly SEMICOLON;
exec_action: name (OpenPar exec_action_arg (Comma exec_action_arg)* ClosePar) SEMICOLON |OpenClosePar SEMICOLON;
exec_action_arg: arg |topic_name| json_struct | OpenCloseCurly;
arg: IDENTIFIER;
name: IDENTIFIER;
topic_name : STRING? (SLASH IDENTIFIER (SLASH IDENTIFIER)*) STRING?;
json_struct: OpenCurly json_pair* (Comma json_pair)* CloseCurly;
json_pair: STRING? varname STRING? COLON STRING? value STRING?;
value: IDENTIFIER | NUMBER;

recognition_bloc: RECOGNITION OpenCurly sequence_bloc parameters_bloc? CloseCurly SEMICOLON;
sequence_bloc: SEQUENCE OpenCurly sequence* CloseCurly SEMICOLON;
sequence: (NOT)? subject predicate object (REQUIRED)? SEMICOLON;
parameters_bloc: PARAMETERS OpenCurly parameter* CloseCurly SEMICOLON;
parameter: name OpenPar value ClosePar SEMICOLON;

// Commitment rules (simple pass-through approach - actual parsing done in FullParser.cpp)
commitments: COMMITMENTS OpenCurly commitment_content CloseCurly SEMICOLON;
commitment_content: (commitment_token | commitment_block | recovery_strategy)*;
commitment_block: OpenCurly commitment_content CloseCurly;
recovery_strategy: RECOVERY_STRATEGY OpenCurly commitment_content CloseCurly;
commitment_token: INSTRUMENTAL | ENGAGEMENT | COMMON_GROUND
                | ON_INSTRUMENTAL_FAILURE | ON_ENGAGEMENT_FAILURE | ON_COMMON_GROUND_FAILURE
                | MODE | MAX_ATTEMPTS | TIMEOUT
                | SELECT | WHERE | FOR | NOT | EXISTS
                | STRING | IDENTIFIER | NUMBER | COLON | POINT | SEMICOLON | Comma
                | QUESTIONMARK | MINUS | PLUS | TIMES | SLASH | OpenPar | ClosePar | OpenSquare | CloseSquare;


attentes_bloc: ATTENTES OpenCurly role* CloseCurly SEMICOLON;
role: ROLE name OpenCurly conditions attente* CloseCurly SEMICOLON;
attente: ATTENTE name OpenCurly conditions CloseCurly SEMICOLON;
conditions: CONDITIONS OpenCurly (query|triplet)* CloseCurly SEMICOLON;


tasks_bloc: TASKS OpenCurly task* CloseCurly SEMICOLON;
task: TASK name OpenPar arguments* (Comma arguments)* ClosePar OpenCurly entry_state? goal? methods_bloc? cost_bloc? duration_bloc? parameters_bloc? CloseCurly SEMICOLON;
entry_state: ENTRYSTATE OpenCurly (triplet|query)* CloseCurly SEMICOLON;
goal: GOAL OpenCurly triplet* CloseCurly SEMICOLON;
methods_bloc: METHODS OpenCurly method (method)* CloseCurly SEMICOLON;
method:  id_method COLON OpenCurly preconditions_bloc* subtask_bloc* CloseCurly ;
subtask_bloc: SUBTASKS OpenCurly subtask_line* CloseCurly SEMICOLON;
subtask_line: id COLON name OpenPar arg* (Comma arg)* ClosePar (operator id)*  SEMICOLON;

event: EVENT OpenCurly query* CloseCurly SEMICOLON;
priority_level: PRIORITY_LEVEL OpenCurly numeric_value SEMICOLON CloseCurly SEMICOLON;
objectifs: OBJECTIVES OpenCurly objectives_content* CloseCurly SEMICOLON;
objectives_content: state_bloc | task_bloc | query | triplet;
state_bloc: STATE OpenCurly (query|triplet)* CloseCurly SEMICOLON;
task_bloc: TASK COLON name OpenPar task_arg* (Comma task_arg)* ClosePar SEMICOLON;
task_arg: variable | name;
priorities_bloc: PRIORITIES OpenCurly priority* CloseCurly SEMICOLON;
priority: PRIORITY name OpenCurly (event|priority_level|objectifs)* CloseCurly SEMICOLON;


pratices_frames_bloc: PRACTICE_FRAMES OpenCurly practice_frame* CloseCurly SEMICOLON;
practice_frame: PRACTICE_FRAME name OpenCurly (description_practice|conditions_practices|practices_list|roles_with_conditions|objects_bloc|rules_bloc)* CloseCurly SEMICOLON;
practices_list: PRACTICES_LIST OpenCurly practice_name* CloseCurly SEMICOLON;
roles_list: ROLES OpenCurly role_name* CloseCurly SEMICOLON;
role_name: MINUS name SEMICOLON;
practice_name: MINUS name SEMICOLON;
description_practice: DESCRIPTION OpenCurly STRING sentence STRING CloseCurly SEMICOLON;
objects_bloc: OBJECTS OpenCurly object_item* CloseCurly SEMICOLON;
object_item: MINUS object SEMICOLON;
rules_bloc: RULES OpenCurly rule_item+ CloseCurly SEMICOLON;
rule_item: MINUS STRING sentence STRING;

// New rules for roles with conditions (at frame level)
roles_with_conditions: ROLES OpenCurly role_with_condition* CloseCurly SEMICOLON;
role_with_condition: MINUS name (OpenCurly conditions capacites_list? attentes_list? CloseCurly)? SEMICOLON;

// New rules for roles with attentes (at practice level)
roles_with_attentes: ROLES OpenCurly role_with_attente* CloseCurly SEMICOLON;
role_with_attente: MINUS name (OpenCurly capacites_list? attentes_list? CloseCurly)? SEMICOLON;
attentes_list: ATTENTES COLON OpenCurly attente_extended* CloseCurly;

// New rules for capacites
capacites_list: CAPACITES COLON OpenCurly capacite* CloseCurly SEMICOLON;
capacite: CAPACITE name OpenCurly (can_satisfy_expectations|description_capacite|conditions)* CloseCurly SEMICOLON;
can_satisfy_expectations: CAN_SATISFY_EXPECTATIONS COLON OpenSquare expectation_type_list CloseSquare SEMICOLON;
expectation_type_list: name (Comma name)*;
description_capacite: DESCRIPTION COLON STRING sentence STRING SEMICOLON;

// Enhanced attente with new fields
attente_extended: ATTENTE name OpenCurly (attente_type|expects_from|description_attente|conditions)* CloseCurly SEMICOLON;
attente_type: TYPE_KW COLON name SEMICOLON;
expects_from: EXPECTS_FROM COLON OpenSquare role_list CloseSquare SEMICOLON;
role_list: name (Comma name)*;
description_attente: DESCRIPTION COLON STRING sentence STRING SEMICOLON;

practices_bloc: PRACTICES OpenCurly practice* CloseCurly SEMICOLON;
practice: PRACTICE name OpenCurly (description_practice|conditions_practices|roles_with_attentes|competences|objects_bloc|rules_bloc)* CloseCurly SEMICOLON;
conditions_practices: CONDITIONS OpenCurly query CloseCurly SEMICOLON;
competences: COMPETENCES OpenCurly competence* CloseCurly SEMICOLON;
competence: MINUS STRING sentence STRING;

id_method: IDENTIFIER | NUMBER;
id: NUMBER;
numeric_value: NUMBER;
subject: (variable | literal | my_self_var);
variable: QUESTIONMARK IDENTIFIER;
literal: IDENTIFIER;
my_self_var: QUESTIONMARK QUESTIONMARK;
predicate: IDENTIFIER;
object: (variable | literal | my_self_var);
arguments: type  varname;
type : IDENTIFIER;
varname : IDENTIFIER;
comment : COMMENT | LINE_COMMENT;
ignore: .*? SEMICOLON;
operator : ADD_IN_SET
          |REMOVE_FROM_SET
          |EQUAL
          |TEST_EQUAL
          |TEST_DIFF
          |TEST_SET_IN
          |TEST_SET_NOT_IN
          |SUP
          |SUP_EQUAL
          |INF
          |INF_EQUAL
          |SUP_TILD;

sentence: (word|SPACE)*;
//sentence: (.)*? SEMICOLON ;
word: IDENTIFIER | Comma | POINT | COLON | QUESTIONMARK | NUMBER | SLASH | PLUS | MINUS | TIMES | SLASH | OpenPar | ClosePar | OpenCurly | CloseCurly;