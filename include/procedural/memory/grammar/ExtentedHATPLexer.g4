lexer grammar ExtentedHATPLexer;

STRING: '"';



HTN: 'HTN';
ACTIONS: 'ACTIONS';
ACTION: 'ACTION';
PRECONDITIONS:  'Preconditions' | 'PRECONDITIONS';
RECOGNITION: 'RECOGNITION';
DESCRIPTION: 'DESCRIPTION' ;
SEQUENCE: 'SEQUENCE';
NOT: 'NOT';
REQUIRED: 'REQUIRED';
EXECUTION: 'EXECUTION';
EFFECTS: 'EFFECTS';
TASKS:  'Tasks' | 'TASKS';
TASK:  'Task' | 'TASK';
GOAL:  'Goal' | 'GOAL';
SUBTASKS: 'SUBTASKS';
COST: 'COST';
DURATION: 'DURATION';
SELECT: 'SELECT';
PACKAGE: 'package';
INCLUDE: 'include';
WHERE: 'WHERE';
ATTENTES:  'Attentes' | 'ATTENTES';
ATTENTE:  'Attente' | 'ATTENTE';
CONDITIONS: 'Conditions' | 'CONDITIONS';
ROLE: 'Role' | 'ROLE';
ENTRYSTATE:  'EntryState' | 'ENTRYSTATE' | 'Entry_state' | 'Entry_State' | 'ENTRY_STATE' | 'entry_state';
METHODS:  'Methods' | 'METHODS';
PARAMETERS:  'Parameters' | 'PARAMETERS';
PRIORITIES: 'Priorities' | 'PRIORITIES';
PRIORITY:  'Priority' | 'PRIORITY';
EVENT:  'Event' | 'EVENT';
OBJECTIVES: 'Objectives' | 'OBJECTIVES';
PRIORITY_LEVEL:  'Priority_level' | 'PRIORITY_LEVEL';

ROLES: 'ROLES';
PRACTICE_FRAMES: 'PRACTICE_FRAMES';
PRACTICE_FRAME: 'PRACTICE_FRAME';
PRACTICES: 'PRACTICES';
PRACTICE: 'PRACTICE';
PRACTICES_LIST: 'PRACTICES_LIST';
OBJECTS: 'OBJECTS';
RULES: 'RULES';
COMPETENCES: 'COMPETENCES';



SPACE : [ ] ;
WS: [ \t\r\n]+ -> skip;
ADD_IN_SET: '<<=';
REMOVE_FROM_SET: '=>>';
EQUAL: '=';
TEST_EQUAL: '==';
TEST_DIFF: '!=';
TEST_SET_IN: '>>';
TEST_SET_NOT_IN: '!>>';
SUP: '>';
SUP_EQUAL: '>=';
INF: '<';
INF_EQUAL: '<=';
SUP_TILD: '~>';
PLUS: '+';
MINUS: '-';
TIMES: '*';
SLASH: '/';
SEMICOLON: ';';
POINT: '.';
COLON: ':';
QUESTIONMARK: '?';
COMMENT
: '/*' .*? '*/' -> skip
;
LINE_COMMENT
: '//' ~[\r\n]* -> skip
;

OpenPar: '(';
ClosePar: ')';
OpenClosePar: '()';
OpenCurly: '{' ;
CloseCurly: '}';
OpenCloseCurly: '{}';
Comma: ',';

IDENTIFIER: [a-zA-Z_][a-zA-Z0-9_]*;
TYPE: [a-zA-Z_][a-zA-Z0-9_]*;
VARNAME: [a-zA-Z_][a-zA-Z0-9_]*;
ATTRIBUT: [a-zA-Z_][a-zA-Z0-9_]*;
NUMBER: [0-9]+;
