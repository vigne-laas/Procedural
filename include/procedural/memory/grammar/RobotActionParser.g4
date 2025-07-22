parser grammar RobotActionParser;
options {
	tokenVocab = RobotActionLexer;
}

root: comment* actions_bloc* comment* EOF;

actions_bloc: ACTIONS OpenCurly action* CloseCurly SEMICOLON;
action: ACTION name OpenPar arguments* (Comma arguments)* ClosePar OpenCurly recognition_bloc? execution_bloc? description_bloc? CloseCurly SEMICOLON;

description_bloc: DESCRIPTION OpenCurly triplet* CloseCurly SEMICOLON;
triplet: (NOT)? subject predicate object (REQUIRED)? SEMICOLON;


execution_bloc: EXECUTION OpenCurly exec_action* CloseCurly SEMICOLON;
exec_action: name (OpenPar exec_action_arg (Comma exec_action_arg)* ClosePar) SEMICOLON |OpenClosePar SEMICOLON;
exec_action_arg: arg |topic_name| json_struct | OpenCloseCurly;
arg: IDENTIFIER;
name: IDENTIFIER;
topic_name : (SLASH IDENTIFIER (SLASH IDENTIFIER)*);
json_struct: OpenCurly json_pair* (Comma json_pair)* CloseCurly;
json_pair: varname COLON value;
value: IDENTIFIER ;


recognition_bloc: RECOGNITION OpenCurly sequence_bloc parameters_bloc? CloseCurly SEMICOLON;
sequence_bloc: SEQUENCE OpenCurly sequence* CloseCurly SEMICOLON;
sequence: (NOT)? subject predicate object (REQUIRED)? SEMICOLON;
parameters_bloc: PARAMETERS OpenCurly parameter* CloseCurly SEMICOLON;
parameter: IDENTIFIER OpenPar NUMBER ClosePar SEMICOLON;

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

