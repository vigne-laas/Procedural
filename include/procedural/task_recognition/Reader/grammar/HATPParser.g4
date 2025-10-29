parser grammar HATPParser;
options {
	tokenVocab = HATPLexer;
}
// parser section
hatp: comment* factbase* comment* htn comment* timepart comment* EOF;

timepart: TIMEPART OpenCurly ignore*  CloseCurly ;
factbase: FACTDATABASE OpenCurly ignore* CloseCurly ;
htn: HTN  OpenCurly actions*  tasks*  CloseCurly ;
actions: ACTION  action_name (OpenPar (arguments (Comma  arguments)*)? ClosePar | OpenClosePar)   OpenCurly preconditions+ effects+ commitments? cost? duration? CloseCurly SEMICOLON?;
action_name: IDENTIFIER;
preconditions: PRECONDITIONS  OpenCurly  expression*  CloseCurly SEMICOLON;
effects: EFFECTS  OpenCurly (forall|expression)* CloseCurly SEMICOLON;
arguments: type  varname;
type : IDENTIFIER;
varname : IDENTIFIER;
value : IDENTIFIER;
tasks: TASK  IDENTIFIER  OpenPar arguments ( Comma  arguments)* ClosePar   OpenCurly  goal (decomposition)+ CloseCurly ;
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

// Commitment system rules - Accept anything as raw text
// This rule matches everything between COMMITMENTS { and };
// The HATPListener will manually parse the text content
commitments: COMMITMENTS OpenCurly commitment_content CloseCurly SEMICOLON;

// Match everything until we find }; at the end
// Using a simple approach: match any sequence that doesn't end with };
commitment_content: (commitment_token | commitment_block)*;

// A block is { ... } which can be nested
commitment_block: OpenCurly commitment_content CloseCurly;

// Any token except OpenCurly and CloseCurly (which are handled by commitment_block)
commitment_token: INSTRUMENTAL | ENGAGEMENT | COMMON_GROUND
                | ON_INSTRUMENTAL_FAILURE | ON_ENGAGEMENT_FAILURE | ON_COMMON_GROUND_FAILURE
                | RECOVERY_STRATEGY | MODE | MAX_ATTEMPTS | TIMEOUT
                | SELECT | COLON | SEMICOLON | STRING | NUMBER | POINT
                | IDENTIFIER | OpenPar | ClosePar | Comma | FORALL
                | ~(OpenCurly | CloseCurly);
