parser grammar HATPParser;
options {
	tokenVocab = HATPLexer;
}
// parser section
hatp: comment* factbase* comment* htn comment* timepart comment* EOF;

timepart: TIMEPART OpenCurly ignore*  CloseCurly ;
factbase: FACTDATABASE OpenCurly ignore* CloseCurly ;
htn: HTN  OpenCurly actions*  methods*  CloseCurly ;
actions: ACTION  action_name OpenPar arguments (Comma  arguments)* ClosePar   OpenCurly preconditions+ effects+ cost? duration? CloseCurly ;
action_name: IDENTIFIER;
preconditions: PRECONDITIONS  OpenCurly  expression*  CloseCurly SEMICOLON;
effects: EFFECTS  OpenCurly (forall|expression)* CloseCurly SEMICOLON;
arguments: type  varname;
type : IDENTIFIER;
varname : IDENTIFIER;
value : IDENTIFIER;
methods: METHOD  IDENTIFIER  OpenPar arguments ( Comma  arguments)* ClosePar   OpenCurly  goal (decomposition)+ CloseCurly ;
decomposition : OpenCurly preconditions subtask  CloseCurly;
subtask: SUBTASK  OpenCurly  (subselection|list)*  CloseCurly  SEMICOLON;
goal: GOAL OpenCurly expression+ CloseCurly SEMICOLON;
comment : COMMENT | LINE_COMMENT;
ignore: .*? SEMICOLON;
attribut : varname (POINT value)?;
subject : attribut;
object : attribut;
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
expression : subject operator object SEMICOLON;
subselection : attribut operator selectcase SEMICOLON;
selectcase: SELECT  OpenPar  IDENTIFIER Comma   OpenCurly  expression?  CloseCurly   ClosePar  ;
list : NUMBER COLON function (order|Comma order)* SEMICOLON;
function : IDENTIFIER OpenPar varname ( Comma varname)*  ClosePar ;
order: operator NUMBER;
forall : FORALL OpenPar arguments  Comma  ( OpenCurly expression? CloseCurly ) ( Comma   OpenCurly expression CloseCurly )* ClosePar SEMICOLON;
cost : COST  OpenCurly  IDENTIFIER OpenClosePar CloseCurly  SEMICOLON;
duration: DURATION  OpenCurly  IDENTIFIER OpenClosePar CloseCurly  SEMICOLON;
