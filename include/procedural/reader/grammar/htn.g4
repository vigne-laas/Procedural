grammar htn;
// parser section
hatp: comment* factbase* comment* htn comment* timepart comment* EOF;

timepart: TIMEPART'{'ignore* '}';
factbase: FACTDATABASE'{'ignore*'}';
htn: HTN '{'actions*  methods* '}';
actions: ACTION  IDENTIFIER'('arguments (',' arguments)*')' '{'preconditions+ effects+ cost? duration?'}';
preconditions: PRECONDITIONS '{' expression* '}'SEMICOLON;
effects: EFFECTS '{'(forall|expression)*'}'SEMICOLON;
arguments: type  varname;
type : IDENTIFIER;
varname : IDENTIFIER;
value : IDENTIFIER;
methods: METHOD  IDENTIFIER '('arguments (',' arguments)*')' '{' goal ('{'preconditions subtask '}')+'}';
subtask: SUBTASK '{' (subselection|order)* '}' SEMICOLON;
goal: GOAL'{'expression+'}'SEMICOLON;
comment : COMMENT | LINE_COMMENT;
ignore: .*? SEMICOLON;
attribut : varname (POINT value)?;
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
expression : attribut operator attribut SEMICOLON;
subselection : attribut operator selectcase SEMICOLON;
selectcase: SELECT '(' IDENTIFIER',' '{' expression? '}' ')' ;
order : NUMBER COLON function test? SEMICOLON;
function : IDENTIFIER'('varname (','varname)* ')';
test: operator NUMBER (',' test)*;
forall : FORALL'('arguments ',' ('{'expression?'}') (',' '{'expression'}')*')'SEMICOLON;
cost : COST '{' IDENTIFIER'()' '}' SEMICOLON;
duration: DURATION '{' IDENTIFIER'()' '}' SEMICOLON;
//lexer section

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

IDENTIFIER: [a-zA-Z_][a-zA-Z0-9_]*;
TYPE: [a-zA-Z_][a-zA-Z0-9_]*;
VARNAME: [a-zA-Z_][a-zA-Z0-9_]*;
ATTRIBUT: [a-zA-Z_][a-zA-Z0-9_]*;
NUMBER: [0-9]+;
