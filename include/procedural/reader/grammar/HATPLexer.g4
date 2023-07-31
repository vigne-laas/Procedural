lexer grammar HATPLexer;

HTN: 'HTN';
ACTION: 'action';
PRECONDITIONS: 'preconditions';
EFFECTS: 'effects';
METHOD: 'method';
GOAL: 'goal';
SUBTASK: 'subtasks';
TIMEPART: 'timePart';
FACTDATABASE: 'factdatabase';
FORALL: 'FORALL';
COST: 'cost';
DURATION: 'duration';
SELECT: 'SELECT';

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
STRING: '"';
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
Comma: ',';

IDENTIFIER: [a-zA-Z_][a-zA-Z0-9_]*;
TYPE: [a-zA-Z_][a-zA-Z0-9_]*;
VARNAME: [a-zA-Z_][a-zA-Z0-9_]*;
ATTRIBUT: [a-zA-Z_][a-zA-Z0-9_]*;
NUMBER: [0-9]+;
