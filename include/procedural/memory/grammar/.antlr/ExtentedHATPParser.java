// Generated from /home/avigne/Projets/ArchiThese/catkin_ws/src/Procedural/include/procedural/memory/grammar/ExtentedHATPParser.g4 by ANTLR 4.13.1
import org.antlr.v4.runtime.atn.*;
import org.antlr.v4.runtime.dfa.DFA;
import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.misc.*;
import org.antlr.v4.runtime.tree.*;
import java.util.List;
import java.util.Iterator;
import java.util.ArrayList;

@SuppressWarnings({"all", "warnings", "unchecked", "unused", "cast", "CheckReturnValue"})
public class ExtentedHATPParser extends Parser {
	static { RuntimeMetaData.checkVersion("4.13.1", RuntimeMetaData.VERSION); }

	protected static final DFA[] _decisionToDFA;
	protected static final PredictionContextCache _sharedContextCache =
		new PredictionContextCache();
	public static final int
		STRING=1, HTN=2, ACTIONS=3, ACTION=4, PRECONDITIONS=5, RECOGNITION=6, 
		DESCRIPTION=7, SEQUENCE=8, NOT=9, REQUIRED=10, EXECUTION=11, EFFECTS=12, 
		TASKS=13, TASK=14, GOAL=15, SUBTASKS=16, COST=17, DURATION=18, SELECT=19, 
		PACKAGE=20, INCLUDE=21, WHERE=22, ATTENTES=23, ATTENTE=24, CONDITIONS=25, 
		ROLE=26, ENTRYSTATE=27, METHODS=28, PARAMETERS=29, PRIORITIES=30, PRIORITY=31, 
		EVENT=32, OBJECTIVES=33, PRIORITY_LEVEL=34, WS=35, ADD_IN_SET=36, REMOVE_FROM_SET=37, 
		EQUAL=38, TEST_EQUAL=39, TEST_DIFF=40, TEST_SET_IN=41, TEST_SET_NOT_IN=42, 
		SUP=43, SUP_EQUAL=44, INF=45, INF_EQUAL=46, SUP_TILD=47, PLUS=48, MINUS=49, 
		TIMES=50, SLASH=51, SEMICOLON=52, POINT=53, COLON=54, QUESTIONMARK=55, 
		COMMENT=56, LINE_COMMENT=57, OpenPar=58, ClosePar=59, OpenClosePar=60, 
		OpenCurly=61, CloseCurly=62, OpenCloseCurly=63, Comma=64, IDENTIFIER=65, 
		TYPE=66, VARNAME=67, ATTRIBUT=68, NUMBER=69, SPACE=70, PRACTICE_FRAMES=71, 
		PRACTICE_FRAME=72, PRACTICES_LIST=73, ROLES=74, OBJECTS=75, RULES=76, 
		CAPACITES=77, CAPACITE=78, CAN_SATISFY_EXPECTATIONS=79, OpenSquare=80, 
		CloseSquare=81, TYPE_KW=82, EXPECTS_FROM=83, PRACTICES=84, PRACTICE=85, 
		COMPETENCES=86;
	public static final int
		RULE_root = 0, RULE_include_bloc = 1, RULE_inclusion = 2, RULE_package_link = 3, 
		RULE_link = 4, RULE_actions_bloc = 5, RULE_action = 6, RULE_preconditions_bloc = 7, 
		RULE_query = 8, RULE_where_clause = 9, RULE_where_statement = 10, RULE_triplet_with_dot = 11, 
		RULE_triplet_pattern = 12, RULE_triplet_query = 13, RULE_source = 14, 
		RULE_effects_bloc = 15, RULE_cost_bloc = 16, RULE_duration_bloc = 17, 
		RULE_description_bloc = 18, RULE_triplet = 19, RULE_execution_bloc = 20, 
		RULE_exec_action = 21, RULE_exec_action_arg = 22, RULE_arg = 23, RULE_name = 24, 
		RULE_topic_name = 25, RULE_json_struct = 26, RULE_json_pair = 27, RULE_value = 28, 
		RULE_recognition_bloc = 29, RULE_sequence_bloc = 30, RULE_sequence = 31, 
		RULE_parameters_bloc = 32, RULE_parameter = 33, RULE_attentes_bloc = 34, 
		RULE_role = 35, RULE_attente = 36, RULE_conditions = 37, RULE_tasks_bloc = 38, 
		RULE_task = 39, RULE_entry_state = 40, RULE_goal = 41, RULE_methods_bloc = 42, 
		RULE_method = 43, RULE_subtask_bloc = 44, RULE_subtask_line = 45, RULE_event = 46, 
		RULE_priority_level = 47, RULE_objectifs = 48, RULE_priorities_bloc = 49, 
		RULE_priority = 50, RULE_pratices_frames_bloc = 51, RULE_practice_frame = 52, 
		RULE_practices_list = 53, RULE_roles_list = 54, RULE_role_name = 55, RULE_practice_name = 56, 
		RULE_description_practice = 57, RULE_objects_bloc = 58, RULE_object_item = 59, 
		RULE_rules_bloc = 60, RULE_rule_item = 61, RULE_roles_with_conditions = 62, 
		RULE_role_with_condition = 63, RULE_roles_with_attentes = 64, RULE_role_with_attente = 65, 
		RULE_attentes_list = 66, RULE_capacites_list = 67, RULE_capacite = 68, 
		RULE_can_satisfy_expectations = 69, RULE_expectation_type_list = 70, RULE_description_capacite = 71, 
		RULE_attente_extended = 72, RULE_attente_type = 73, RULE_expects_from = 74, 
		RULE_role_list = 75, RULE_description_attente = 76, RULE_practices_bloc = 77, 
		RULE_practice = 78, RULE_conditions_practices = 79, RULE_competences = 80, 
		RULE_competence = 81, RULE_id_method = 82, RULE_id = 83, RULE_numeric_value = 84, 
		RULE_subject = 85, RULE_variable = 86, RULE_literal = 87, RULE_my_self_var = 88, 
		RULE_predicate = 89, RULE_object = 90, RULE_arguments = 91, RULE_type = 92, 
		RULE_varname = 93, RULE_comment = 94, RULE_ignore = 95, RULE_operator = 96, 
		RULE_sentence = 97, RULE_word = 98;
	private static String[] makeRuleNames() {
		return new String[] {
			"root", "include_bloc", "inclusion", "package_link", "link", "actions_bloc", 
			"action", "preconditions_bloc", "query", "where_clause", "where_statement", 
			"triplet_with_dot", "triplet_pattern", "triplet_query", "source", "effects_bloc", 
			"cost_bloc", "duration_bloc", "description_bloc", "triplet", "execution_bloc", 
			"exec_action", "exec_action_arg", "arg", "name", "topic_name", "json_struct", 
			"json_pair", "value", "recognition_bloc", "sequence_bloc", "sequence", 
			"parameters_bloc", "parameter", "attentes_bloc", "role", "attente", "conditions", 
			"tasks_bloc", "task", "entry_state", "goal", "methods_bloc", "method", 
			"subtask_bloc", "subtask_line", "event", "priority_level", "objectifs", 
			"priorities_bloc", "priority", "pratices_frames_bloc", "practice_frame", 
			"practices_list", "roles_list", "role_name", "practice_name", "description_practice", 
			"objects_bloc", "object_item", "rules_bloc", "rule_item", "roles_with_conditions", 
			"role_with_condition", "roles_with_attentes", "role_with_attente", "attentes_list", 
			"capacites_list", "capacite", "can_satisfy_expectations", "expectation_type_list", 
			"description_capacite", "attente_extended", "attente_type", "expects_from", 
			"role_list", "description_attente", "practices_bloc", "practice", "conditions_practices", 
			"competences", "competence", "id_method", "id", "numeric_value", "subject", 
			"variable", "literal", "my_self_var", "predicate", "object", "arguments", 
			"type", "varname", "comment", "ignore", "operator", "sentence", "word"
		};
	}
	public static final String[] ruleNames = makeRuleNames();

	private static String[] makeLiteralNames() {
		return new String[] {
			null, "'\"'", "'HTN'", "'ACTIONS'", "'ACTION'", null, "'RECOGNITION'", 
			"'DESCRIPTION'", "'SEQUENCE'", "'NOT'", "'REQUIRED'", "'EXECUTION'", 
			"'EFFECTS'", null, null, null, "'SUBTASKS'", "'COST'", "'DURATION'", 
			"'SELECT'", "'package'", "'include'", "'WHERE'", null, null, null, null, 
			null, null, null, null, null, null, null, null, null, "'<<='", "'=>>'", 
			"'='", "'=='", "'!='", "'>>'", "'!>>'", "'>'", "'>='", "'<'", "'<='", 
			"'~>'", "'+'", "'-'", "'*'", "'/'", "';'", "'.'", "':'", "'?'", null, 
			null, "'('", "')'", "'()'", "'{'", "'}'", "'{}'", "','"
		};
	}
	private static final String[] _LITERAL_NAMES = makeLiteralNames();
	private static String[] makeSymbolicNames() {
		return new String[] {
			null, "STRING", "HTN", "ACTIONS", "ACTION", "PRECONDITIONS", "RECOGNITION", 
			"DESCRIPTION", "SEQUENCE", "NOT", "REQUIRED", "EXECUTION", "EFFECTS", 
			"TASKS", "TASK", "GOAL", "SUBTASKS", "COST", "DURATION", "SELECT", "PACKAGE", 
			"INCLUDE", "WHERE", "ATTENTES", "ATTENTE", "CONDITIONS", "ROLE", "ENTRYSTATE", 
			"METHODS", "PARAMETERS", "PRIORITIES", "PRIORITY", "EVENT", "OBJECTIVES", 
			"PRIORITY_LEVEL", "WS", "ADD_IN_SET", "REMOVE_FROM_SET", "EQUAL", "TEST_EQUAL", 
			"TEST_DIFF", "TEST_SET_IN", "TEST_SET_NOT_IN", "SUP", "SUP_EQUAL", "INF", 
			"INF_EQUAL", "SUP_TILD", "PLUS", "MINUS", "TIMES", "SLASH", "SEMICOLON", 
			"POINT", "COLON", "QUESTIONMARK", "COMMENT", "LINE_COMMENT", "OpenPar", 
			"ClosePar", "OpenClosePar", "OpenCurly", "CloseCurly", "OpenCloseCurly", 
			"Comma", "IDENTIFIER", "TYPE", "VARNAME", "ATTRIBUT", "NUMBER", "SPACE", 
			"PRACTICE_FRAMES", "PRACTICE_FRAME", "PRACTICES_LIST", "ROLES", "OBJECTS", 
			"RULES", "CAPACITES", "CAPACITE", "CAN_SATISFY_EXPECTATIONS", "OpenSquare", 
			"CloseSquare", "TYPE_KW", "EXPECTS_FROM", "PRACTICES", "PRACTICE", "COMPETENCES"
		};
	}
	private static final String[] _SYMBOLIC_NAMES = makeSymbolicNames();
	public static final Vocabulary VOCABULARY = new VocabularyImpl(_LITERAL_NAMES, _SYMBOLIC_NAMES);

	/**
	 * @deprecated Use {@link #VOCABULARY} instead.
	 */
	@Deprecated
	public static final String[] tokenNames;
	static {
		tokenNames = new String[_SYMBOLIC_NAMES.length];
		for (int i = 0; i < tokenNames.length; i++) {
			tokenNames[i] = VOCABULARY.getLiteralName(i);
			if (tokenNames[i] == null) {
				tokenNames[i] = VOCABULARY.getSymbolicName(i);
			}

			if (tokenNames[i] == null) {
				tokenNames[i] = "<INVALID>";
			}
		}
	}

	@Override
	@Deprecated
	public String[] getTokenNames() {
		return tokenNames;
	}

	@Override

	public Vocabulary getVocabulary() {
		return VOCABULARY;
	}

	@Override
	public String getGrammarFileName() { return "ExtentedHATPParser.g4"; }

	@Override
	public String[] getRuleNames() { return ruleNames; }

	@Override
	public String getSerializedATN() { return _serializedATN; }

	@Override
	public ATN getATN() { return _ATN; }

	public ExtentedHATPParser(TokenStream input) {
		super(input);
		_interp = new ParserATNSimulator(this,_ATN,_decisionToDFA,_sharedContextCache);
	}

	@SuppressWarnings("CheckReturnValue")
	public static class RootContext extends ParserRuleContext {
		public TerminalNode EOF() { return getToken(ExtentedHATPParser.EOF, 0); }
		public List<Include_blocContext> include_bloc() {
			return getRuleContexts(Include_blocContext.class);
		}
		public Include_blocContext include_bloc(int i) {
			return getRuleContext(Include_blocContext.class,i);
		}
		public List<CommentContext> comment() {
			return getRuleContexts(CommentContext.class);
		}
		public CommentContext comment(int i) {
			return getRuleContext(CommentContext.class,i);
		}
		public List<Actions_blocContext> actions_bloc() {
			return getRuleContexts(Actions_blocContext.class);
		}
		public Actions_blocContext actions_bloc(int i) {
			return getRuleContext(Actions_blocContext.class,i);
		}
		public List<Attentes_blocContext> attentes_bloc() {
			return getRuleContexts(Attentes_blocContext.class);
		}
		public Attentes_blocContext attentes_bloc(int i) {
			return getRuleContext(Attentes_blocContext.class,i);
		}
		public List<Tasks_blocContext> tasks_bloc() {
			return getRuleContexts(Tasks_blocContext.class);
		}
		public Tasks_blocContext tasks_bloc(int i) {
			return getRuleContext(Tasks_blocContext.class,i);
		}
		public List<Priorities_blocContext> priorities_bloc() {
			return getRuleContexts(Priorities_blocContext.class);
		}
		public Priorities_blocContext priorities_bloc(int i) {
			return getRuleContext(Priorities_blocContext.class,i);
		}
		public List<Pratices_frames_blocContext> pratices_frames_bloc() {
			return getRuleContexts(Pratices_frames_blocContext.class);
		}
		public Pratices_frames_blocContext pratices_frames_bloc(int i) {
			return getRuleContext(Pratices_frames_blocContext.class,i);
		}
		public List<Practices_blocContext> practices_bloc() {
			return getRuleContexts(Practices_blocContext.class);
		}
		public Practices_blocContext practices_bloc(int i) {
			return getRuleContext(Practices_blocContext.class,i);
		}
		public RootContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_root; }
	}

	public final RootContext root() throws RecognitionException {
		RootContext _localctx = new RootContext(_ctx, getState());
		enterRule(_localctx, 0, RULE_root);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(206); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				setState(206);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case INCLUDE:
					{
					setState(198);
					include_bloc();
					}
					break;
				case COMMENT:
				case LINE_COMMENT:
					{
					setState(199);
					comment();
					}
					break;
				case ACTIONS:
					{
					setState(200);
					actions_bloc();
					}
					break;
				case ATTENTES:
					{
					setState(201);
					attentes_bloc();
					}
					break;
				case TASKS:
					{
					setState(202);
					tasks_bloc();
					}
					break;
				case PRIORITIES:
					{
					setState(203);
					priorities_bloc();
					}
					break;
				case PRACTICE_FRAMES:
					{
					setState(204);
					pratices_frames_bloc();
					}
					break;
				case PRACTICES:
					{
					setState(205);
					practices_bloc();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(208); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( (((_la) & ~0x3f) == 0 && ((1L << _la) & 216172783198019592L) != 0) || _la==PRACTICE_FRAMES || _la==PRACTICES );
			setState(210);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Include_blocContext extends ParserRuleContext {
		public TerminalNode INCLUDE() { return getToken(ExtentedHATPParser.INCLUDE, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<InclusionContext> inclusion() {
			return getRuleContexts(InclusionContext.class);
		}
		public InclusionContext inclusion(int i) {
			return getRuleContext(InclusionContext.class,i);
		}
		public Include_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_include_bloc; }
	}

	public final Include_blocContext include_bloc() throws RecognitionException {
		Include_blocContext _localctx = new Include_blocContext(_ctx, getState());
		enterRule(_localctx, 2, RULE_include_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(212);
			match(INCLUDE);
			setState(213);
			match(OpenCurly);
			setState(217);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==STRING) {
				{
				{
				setState(214);
				inclusion();
				}
				}
				setState(219);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(220);
			match(CloseCurly);
			setState(221);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class InclusionContext extends ParserRuleContext {
		public List<TerminalNode> STRING() { return getTokens(ExtentedHATPParser.STRING); }
		public TerminalNode STRING(int i) {
			return getToken(ExtentedHATPParser.STRING, i);
		}
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public Package_linkContext package_link() {
			return getRuleContext(Package_linkContext.class,0);
		}
		public LinkContext link() {
			return getRuleContext(LinkContext.class,0);
		}
		public InclusionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_inclusion; }
	}

	public final InclusionContext inclusion() throws RecognitionException {
		InclusionContext _localctx = new InclusionContext(_ctx, getState());
		enterRule(_localctx, 4, RULE_inclusion);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(223);
			match(STRING);
			setState(226);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case PACKAGE:
				{
				setState(224);
				package_link();
				}
				break;
			case SLASH:
			case IDENTIFIER:
				{
				setState(225);
				link();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			setState(228);
			match(STRING);
			setState(229);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Package_linkContext extends ParserRuleContext {
		public TerminalNode PACKAGE() { return getToken(ExtentedHATPParser.PACKAGE, 0); }
		public List<TerminalNode> COLON() { return getTokens(ExtentedHATPParser.COLON); }
		public TerminalNode COLON(int i) {
			return getToken(ExtentedHATPParser.COLON, i);
		}
		public LinkContext link() {
			return getRuleContext(LinkContext.class,0);
		}
		public Package_linkContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_package_link; }
	}

	public final Package_linkContext package_link() throws RecognitionException {
		Package_linkContext _localctx = new Package_linkContext(_ctx, getState());
		enterRule(_localctx, 6, RULE_package_link);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(231);
			match(PACKAGE);
			setState(232);
			match(COLON);
			setState(233);
			match(COLON);
			setState(234);
			link();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class LinkContext extends ParserRuleContext {
		public List<TerminalNode> IDENTIFIER() { return getTokens(ExtentedHATPParser.IDENTIFIER); }
		public TerminalNode IDENTIFIER(int i) {
			return getToken(ExtentedHATPParser.IDENTIFIER, i);
		}
		public TerminalNode POINT() { return getToken(ExtentedHATPParser.POINT, 0); }
		public List<TerminalNode> SLASH() { return getTokens(ExtentedHATPParser.SLASH); }
		public TerminalNode SLASH(int i) {
			return getToken(ExtentedHATPParser.SLASH, i);
		}
		public LinkContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_link; }
	}

	public final LinkContext link() throws RecognitionException {
		LinkContext _localctx = new LinkContext(_ctx, getState());
		enterRule(_localctx, 8, RULE_link);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(237);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==SLASH) {
				{
				setState(236);
				match(SLASH);
				}
			}

			setState(239);
			match(IDENTIFIER);
			setState(244);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==SLASH) {
				{
				{
				setState(240);
				match(SLASH);
				setState(241);
				match(IDENTIFIER);
				}
				}
				setState(246);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(247);
			match(POINT);
			setState(248);
			match(IDENTIFIER);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Actions_blocContext extends ParserRuleContext {
		public TerminalNode ACTIONS() { return getToken(ExtentedHATPParser.ACTIONS, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<ActionContext> action() {
			return getRuleContexts(ActionContext.class);
		}
		public ActionContext action(int i) {
			return getRuleContext(ActionContext.class,i);
		}
		public Actions_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_actions_bloc; }
	}

	public final Actions_blocContext actions_bloc() throws RecognitionException {
		Actions_blocContext _localctx = new Actions_blocContext(_ctx, getState());
		enterRule(_localctx, 10, RULE_actions_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(250);
			match(ACTIONS);
			setState(251);
			match(OpenCurly);
			setState(255);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==ACTION) {
				{
				{
				setState(252);
				action();
				}
				}
				setState(257);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(258);
			match(CloseCurly);
			setState(259);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class ActionContext extends ParserRuleContext {
		public TerminalNode ACTION() { return getToken(ExtentedHATPParser.ACTION, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode OpenPar() { return getToken(ExtentedHATPParser.OpenPar, 0); }
		public TerminalNode ClosePar() { return getToken(ExtentedHATPParser.ClosePar, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<ArgumentsContext> arguments() {
			return getRuleContexts(ArgumentsContext.class);
		}
		public ArgumentsContext arguments(int i) {
			return getRuleContext(ArgumentsContext.class,i);
		}
		public List<TerminalNode> Comma() { return getTokens(ExtentedHATPParser.Comma); }
		public TerminalNode Comma(int i) {
			return getToken(ExtentedHATPParser.Comma, i);
		}
		public Preconditions_blocContext preconditions_bloc() {
			return getRuleContext(Preconditions_blocContext.class,0);
		}
		public Effects_blocContext effects_bloc() {
			return getRuleContext(Effects_blocContext.class,0);
		}
		public List<Recognition_blocContext> recognition_bloc() {
			return getRuleContexts(Recognition_blocContext.class);
		}
		public Recognition_blocContext recognition_bloc(int i) {
			return getRuleContext(Recognition_blocContext.class,i);
		}
		public List<Execution_blocContext> execution_bloc() {
			return getRuleContexts(Execution_blocContext.class);
		}
		public Execution_blocContext execution_bloc(int i) {
			return getRuleContext(Execution_blocContext.class,i);
		}
		public List<Description_blocContext> description_bloc() {
			return getRuleContexts(Description_blocContext.class);
		}
		public Description_blocContext description_bloc(int i) {
			return getRuleContext(Description_blocContext.class,i);
		}
		public List<Cost_blocContext> cost_bloc() {
			return getRuleContexts(Cost_blocContext.class);
		}
		public Cost_blocContext cost_bloc(int i) {
			return getRuleContext(Cost_blocContext.class,i);
		}
		public List<Duration_blocContext> duration_bloc() {
			return getRuleContexts(Duration_blocContext.class);
		}
		public Duration_blocContext duration_bloc(int i) {
			return getRuleContext(Duration_blocContext.class,i);
		}
		public ActionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_action; }
	}

	public final ActionContext action() throws RecognitionException {
		ActionContext _localctx = new ActionContext(_ctx, getState());
		enterRule(_localctx, 12, RULE_action);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(261);
			match(ACTION);
			setState(262);
			name();
			setState(263);
			match(OpenPar);
			setState(267);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==IDENTIFIER) {
				{
				{
				setState(264);
				arguments();
				}
				}
				setState(269);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(274);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==Comma) {
				{
				{
				setState(270);
				match(Comma);
				setState(271);
				arguments();
				}
				}
				setState(276);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(277);
			match(ClosePar);
			setState(278);
			match(OpenCurly);
			setState(280);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==PRECONDITIONS) {
				{
				setState(279);
				preconditions_bloc();
				}
			}

			setState(283);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==EFFECTS) {
				{
				setState(282);
				effects_bloc();
				}
			}

			setState(290); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				setState(290);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case RECOGNITION:
					{
					setState(285);
					recognition_bloc();
					}
					break;
				case EXECUTION:
					{
					setState(286);
					execution_bloc();
					}
					break;
				case DESCRIPTION:
					{
					setState(287);
					description_bloc();
					}
					break;
				case COST:
					{
					setState(288);
					cost_bloc();
					}
					break;
				case DURATION:
					{
					setState(289);
					duration_bloc();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(292); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( (((_la) & ~0x3f) == 0 && ((1L << _la) & 395456L) != 0) );
			setState(294);
			match(CloseCurly);
			setState(295);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Preconditions_blocContext extends ParserRuleContext {
		public TerminalNode PRECONDITIONS() { return getToken(ExtentedHATPParser.PRECONDITIONS, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<QueryContext> query() {
			return getRuleContexts(QueryContext.class);
		}
		public QueryContext query(int i) {
			return getRuleContext(QueryContext.class,i);
		}
		public List<TripletContext> triplet() {
			return getRuleContexts(TripletContext.class);
		}
		public TripletContext triplet(int i) {
			return getRuleContext(TripletContext.class,i);
		}
		public Preconditions_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_preconditions_bloc; }
	}

	public final Preconditions_blocContext preconditions_bloc() throws RecognitionException {
		Preconditions_blocContext _localctx = new Preconditions_blocContext(_ctx, getState());
		enterRule(_localctx, 14, RULE_preconditions_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(297);
			match(PRECONDITIONS);
			setState(298);
			match(OpenCurly);
			setState(303);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (((((_la - 9)) & ~0x3f) == 0 && ((1L << (_la - 9)) & 72127962782106625L) != 0)) {
				{
				setState(301);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case SELECT:
					{
					setState(299);
					query();
					}
					break;
				case NOT:
				case QUESTIONMARK:
				case IDENTIFIER:
					{
					setState(300);
					triplet();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(305);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(306);
			match(CloseCurly);
			setState(307);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class QueryContext extends ParserRuleContext {
		public TerminalNode SELECT() { return getToken(ExtentedHATPParser.SELECT, 0); }
		public TerminalNode WHERE() { return getToken(ExtentedHATPParser.WHERE, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public Where_clauseContext where_clause() {
			return getRuleContext(Where_clauseContext.class,0);
		}
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public List<VariableContext> variable() {
			return getRuleContexts(VariableContext.class);
		}
		public VariableContext variable(int i) {
			return getRuleContext(VariableContext.class,i);
		}
		public TerminalNode TIMES() { return getToken(ExtentedHATPParser.TIMES, 0); }
		public TerminalNode MINUS() { return getToken(ExtentedHATPParser.MINUS, 0); }
		public List<TerminalNode> Comma() { return getTokens(ExtentedHATPParser.Comma); }
		public TerminalNode Comma(int i) {
			return getToken(ExtentedHATPParser.Comma, i);
		}
		public QueryContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_query; }
	}

	public final QueryContext query() throws RecognitionException {
		QueryContext _localctx = new QueryContext(_ctx, getState());
		enterRule(_localctx, 16, RULE_query);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(309);
			match(SELECT);
			setState(313);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case QUESTIONMARK:
				{
				setState(310);
				variable();
				}
				break;
			case TIMES:
				{
				setState(311);
				match(TIMES);
				}
				break;
			case MINUS:
				{
				setState(312);
				match(MINUS);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			setState(319);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==Comma) {
				{
				{
				setState(315);
				match(Comma);
				setState(316);
				variable();
				}
				}
				setState(321);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(322);
			match(WHERE);
			setState(323);
			match(OpenCurly);
			setState(324);
			where_clause();
			setState(325);
			match(CloseCurly);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Where_clauseContext extends ParserRuleContext {
		public List<Where_statementContext> where_statement() {
			return getRuleContexts(Where_statementContext.class);
		}
		public Where_statementContext where_statement(int i) {
			return getRuleContext(Where_statementContext.class,i);
		}
		public Where_clauseContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_where_clause; }
	}

	public final Where_clauseContext where_clause() throws RecognitionException {
		Where_clauseContext _localctx = new Where_clauseContext(_ctx, getState());
		enterRule(_localctx, 18, RULE_where_clause);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(328); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(327);
				where_statement();
				}
				}
				setState(330); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( ((((_la - 9)) & ~0x3f) == 0 && ((1L << (_la - 9)) & 72127962782105601L) != 0) );
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Where_statementContext extends ParserRuleContext {
		public TerminalNode NOT() { return getToken(ExtentedHATPParser.NOT, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode POINT() { return getToken(ExtentedHATPParser.POINT, 0); }
		public List<Triplet_with_dotContext> triplet_with_dot() {
			return getRuleContexts(Triplet_with_dotContext.class);
		}
		public Triplet_with_dotContext triplet_with_dot(int i) {
			return getRuleContext(Triplet_with_dotContext.class,i);
		}
		public Where_statementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_where_statement; }
	}

	public final Where_statementContext where_statement() throws RecognitionException {
		Where_statementContext _localctx = new Where_statementContext(_ctx, getState());
		enterRule(_localctx, 20, RULE_where_statement);
		int _la;
		try {
			setState(343);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case NOT:
				enterOuterAlt(_localctx, 1);
				{
				setState(332);
				match(NOT);
				setState(333);
				match(OpenCurly);
				setState(335); 
				_errHandler.sync(this);
				_la = _input.LA(1);
				do {
					{
					{
					setState(334);
					triplet_with_dot();
					}
					}
					setState(337); 
					_errHandler.sync(this);
					_la = _input.LA(1);
				} while ( _la==QUESTIONMARK || _la==IDENTIFIER );
				setState(339);
				match(CloseCurly);
				setState(340);
				match(POINT);
				}
				break;
			case QUESTIONMARK:
			case IDENTIFIER:
				enterOuterAlt(_localctx, 2);
				{
				setState(342);
				triplet_with_dot();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Triplet_with_dotContext extends ParserRuleContext {
		public Triplet_patternContext triplet_pattern() {
			return getRuleContext(Triplet_patternContext.class,0);
		}
		public TerminalNode POINT() { return getToken(ExtentedHATPParser.POINT, 0); }
		public List<TerminalNode> SPACE() { return getTokens(ExtentedHATPParser.SPACE); }
		public TerminalNode SPACE(int i) {
			return getToken(ExtentedHATPParser.SPACE, i);
		}
		public Triplet_with_dotContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_triplet_with_dot; }
	}

	public final Triplet_with_dotContext triplet_with_dot() throws RecognitionException {
		Triplet_with_dotContext _localctx = new Triplet_with_dotContext(_ctx, getState());
		enterRule(_localctx, 22, RULE_triplet_with_dot);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(345);
			triplet_pattern();
			setState(346);
			match(POINT);
			setState(350);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==SPACE) {
				{
				{
				setState(347);
				match(SPACE);
				}
				}
				setState(352);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Triplet_patternContext extends ParserRuleContext {
		public SubjectContext subject() {
			return getRuleContext(SubjectContext.class,0);
		}
		public SourceContext source() {
			return getRuleContext(SourceContext.class,0);
		}
		public TerminalNode COLON() { return getToken(ExtentedHATPParser.COLON, 0); }
		public PredicateContext predicate() {
			return getRuleContext(PredicateContext.class,0);
		}
		public ObjectContext object() {
			return getRuleContext(ObjectContext.class,0);
		}
		public Triplet_patternContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_triplet_pattern; }
	}

	public final Triplet_patternContext triplet_pattern() throws RecognitionException {
		Triplet_patternContext _localctx = new Triplet_patternContext(_ctx, getState());
		enterRule(_localctx, 24, RULE_triplet_pattern);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(353);
			subject();
			setState(354);
			source();
			setState(355);
			match(COLON);
			setState(356);
			predicate();
			setState(357);
			object();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Triplet_queryContext extends ParserRuleContext {
		public SubjectContext subject() {
			return getRuleContext(SubjectContext.class,0);
		}
		public SourceContext source() {
			return getRuleContext(SourceContext.class,0);
		}
		public TerminalNode COLON() { return getToken(ExtentedHATPParser.COLON, 0); }
		public PredicateContext predicate() {
			return getRuleContext(PredicateContext.class,0);
		}
		public ObjectContext object() {
			return getRuleContext(ObjectContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public TerminalNode NOT() { return getToken(ExtentedHATPParser.NOT, 0); }
		public Triplet_queryContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_triplet_query; }
	}

	public final Triplet_queryContext triplet_query() throws RecognitionException {
		Triplet_queryContext _localctx = new Triplet_queryContext(_ctx, getState());
		enterRule(_localctx, 26, RULE_triplet_query);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(360);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==NOT) {
				{
				setState(359);
				match(NOT);
				}
			}

			setState(362);
			subject();
			setState(363);
			source();
			setState(364);
			match(COLON);
			setState(365);
			predicate();
			setState(366);
			object();
			setState(367);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class SourceContext extends ParserRuleContext {
		public TerminalNode IDENTIFIER() { return getToken(ExtentedHATPParser.IDENTIFIER, 0); }
		public SourceContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_source; }
	}

	public final SourceContext source() throws RecognitionException {
		SourceContext _localctx = new SourceContext(_ctx, getState());
		enterRule(_localctx, 28, RULE_source);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(369);
			match(IDENTIFIER);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Effects_blocContext extends ParserRuleContext {
		public TerminalNode EFFECTS() { return getToken(ExtentedHATPParser.EFFECTS, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<TripletContext> triplet() {
			return getRuleContexts(TripletContext.class);
		}
		public TripletContext triplet(int i) {
			return getRuleContext(TripletContext.class,i);
		}
		public Effects_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_effects_bloc; }
	}

	public final Effects_blocContext effects_bloc() throws RecognitionException {
		Effects_blocContext _localctx = new Effects_blocContext(_ctx, getState());
		enterRule(_localctx, 30, RULE_effects_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(371);
			match(EFFECTS);
			setState(372);
			match(OpenCurly);
			setState(376);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (((((_la - 9)) & ~0x3f) == 0 && ((1L << (_la - 9)) & 72127962782105601L) != 0)) {
				{
				{
				setState(373);
				triplet();
				}
				}
				setState(378);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(379);
			match(CloseCurly);
			setState(380);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Cost_blocContext extends ParserRuleContext {
		public TerminalNode COST() { return getToken(ExtentedHATPParser.COST, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public Numeric_valueContext numeric_value() {
			return getRuleContext(Numeric_valueContext.class,0);
		}
		public List<TerminalNode> SEMICOLON() { return getTokens(ExtentedHATPParser.SEMICOLON); }
		public TerminalNode SEMICOLON(int i) {
			return getToken(ExtentedHATPParser.SEMICOLON, i);
		}
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public Cost_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_cost_bloc; }
	}

	public final Cost_blocContext cost_bloc() throws RecognitionException {
		Cost_blocContext _localctx = new Cost_blocContext(_ctx, getState());
		enterRule(_localctx, 32, RULE_cost_bloc);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(382);
			match(COST);
			setState(383);
			match(OpenCurly);
			setState(384);
			numeric_value();
			setState(385);
			match(SEMICOLON);
			setState(386);
			match(CloseCurly);
			setState(387);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Duration_blocContext extends ParserRuleContext {
		public TerminalNode DURATION() { return getToken(ExtentedHATPParser.DURATION, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public Numeric_valueContext numeric_value() {
			return getRuleContext(Numeric_valueContext.class,0);
		}
		public List<TerminalNode> SEMICOLON() { return getTokens(ExtentedHATPParser.SEMICOLON); }
		public TerminalNode SEMICOLON(int i) {
			return getToken(ExtentedHATPParser.SEMICOLON, i);
		}
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public Duration_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_duration_bloc; }
	}

	public final Duration_blocContext duration_bloc() throws RecognitionException {
		Duration_blocContext _localctx = new Duration_blocContext(_ctx, getState());
		enterRule(_localctx, 34, RULE_duration_bloc);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(389);
			match(DURATION);
			setState(390);
			match(OpenCurly);
			setState(391);
			numeric_value();
			setState(392);
			match(SEMICOLON);
			setState(393);
			match(CloseCurly);
			setState(394);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Description_blocContext extends ParserRuleContext {
		public TerminalNode DESCRIPTION() { return getToken(ExtentedHATPParser.DESCRIPTION, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<TripletContext> triplet() {
			return getRuleContexts(TripletContext.class);
		}
		public TripletContext triplet(int i) {
			return getRuleContext(TripletContext.class,i);
		}
		public Description_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_description_bloc; }
	}

	public final Description_blocContext description_bloc() throws RecognitionException {
		Description_blocContext _localctx = new Description_blocContext(_ctx, getState());
		enterRule(_localctx, 36, RULE_description_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(396);
			match(DESCRIPTION);
			setState(397);
			match(OpenCurly);
			setState(401);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (((((_la - 9)) & ~0x3f) == 0 && ((1L << (_la - 9)) & 72127962782105601L) != 0)) {
				{
				{
				setState(398);
				triplet();
				}
				}
				setState(403);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(404);
			match(CloseCurly);
			setState(405);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class TripletContext extends ParserRuleContext {
		public SubjectContext subject() {
			return getRuleContext(SubjectContext.class,0);
		}
		public PredicateContext predicate() {
			return getRuleContext(PredicateContext.class,0);
		}
		public ObjectContext object() {
			return getRuleContext(ObjectContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public TerminalNode NOT() { return getToken(ExtentedHATPParser.NOT, 0); }
		public TerminalNode REQUIRED() { return getToken(ExtentedHATPParser.REQUIRED, 0); }
		public TripletContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_triplet; }
	}

	public final TripletContext triplet() throws RecognitionException {
		TripletContext _localctx = new TripletContext(_ctx, getState());
		enterRule(_localctx, 38, RULE_triplet);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(408);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==NOT) {
				{
				setState(407);
				match(NOT);
				}
			}

			setState(410);
			subject();
			setState(411);
			predicate();
			setState(412);
			object();
			setState(414);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==REQUIRED) {
				{
				setState(413);
				match(REQUIRED);
				}
			}

			setState(416);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Execution_blocContext extends ParserRuleContext {
		public TerminalNode EXECUTION() { return getToken(ExtentedHATPParser.EXECUTION, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<Exec_actionContext> exec_action() {
			return getRuleContexts(Exec_actionContext.class);
		}
		public Exec_actionContext exec_action(int i) {
			return getRuleContext(Exec_actionContext.class,i);
		}
		public Execution_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_execution_bloc; }
	}

	public final Execution_blocContext execution_bloc() throws RecognitionException {
		Execution_blocContext _localctx = new Execution_blocContext(_ctx, getState());
		enterRule(_localctx, 40, RULE_execution_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(418);
			match(EXECUTION);
			setState(419);
			match(OpenCurly);
			setState(423);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==OpenClosePar || _la==IDENTIFIER) {
				{
				{
				setState(420);
				exec_action();
				}
				}
				setState(425);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(426);
			match(CloseCurly);
			setState(427);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Exec_actionContext extends ParserRuleContext {
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public TerminalNode OpenPar() { return getToken(ExtentedHATPParser.OpenPar, 0); }
		public List<Exec_action_argContext> exec_action_arg() {
			return getRuleContexts(Exec_action_argContext.class);
		}
		public Exec_action_argContext exec_action_arg(int i) {
			return getRuleContext(Exec_action_argContext.class,i);
		}
		public TerminalNode ClosePar() { return getToken(ExtentedHATPParser.ClosePar, 0); }
		public List<TerminalNode> Comma() { return getTokens(ExtentedHATPParser.Comma); }
		public TerminalNode Comma(int i) {
			return getToken(ExtentedHATPParser.Comma, i);
		}
		public TerminalNode OpenClosePar() { return getToken(ExtentedHATPParser.OpenClosePar, 0); }
		public Exec_actionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_exec_action; }
	}

	public final Exec_actionContext exec_action() throws RecognitionException {
		Exec_actionContext _localctx = new Exec_actionContext(_ctx, getState());
		enterRule(_localctx, 42, RULE_exec_action);
		int _la;
		try {
			setState(445);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case IDENTIFIER:
				enterOuterAlt(_localctx, 1);
				{
				setState(429);
				name();
				{
				setState(430);
				match(OpenPar);
				setState(431);
				exec_action_arg();
				setState(436);
				_errHandler.sync(this);
				_la = _input.LA(1);
				while (_la==Comma) {
					{
					{
					setState(432);
					match(Comma);
					setState(433);
					exec_action_arg();
					}
					}
					setState(438);
					_errHandler.sync(this);
					_la = _input.LA(1);
				}
				setState(439);
				match(ClosePar);
				}
				setState(441);
				match(SEMICOLON);
				}
				break;
			case OpenClosePar:
				enterOuterAlt(_localctx, 2);
				{
				setState(443);
				match(OpenClosePar);
				setState(444);
				match(SEMICOLON);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Exec_action_argContext extends ParserRuleContext {
		public ArgContext arg() {
			return getRuleContext(ArgContext.class,0);
		}
		public Topic_nameContext topic_name() {
			return getRuleContext(Topic_nameContext.class,0);
		}
		public Json_structContext json_struct() {
			return getRuleContext(Json_structContext.class,0);
		}
		public TerminalNode OpenCloseCurly() { return getToken(ExtentedHATPParser.OpenCloseCurly, 0); }
		public Exec_action_argContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_exec_action_arg; }
	}

	public final Exec_action_argContext exec_action_arg() throws RecognitionException {
		Exec_action_argContext _localctx = new Exec_action_argContext(_ctx, getState());
		enterRule(_localctx, 44, RULE_exec_action_arg);
		try {
			setState(451);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case IDENTIFIER:
				enterOuterAlt(_localctx, 1);
				{
				setState(447);
				arg();
				}
				break;
			case STRING:
			case SLASH:
				enterOuterAlt(_localctx, 2);
				{
				setState(448);
				topic_name();
				}
				break;
			case OpenCurly:
				enterOuterAlt(_localctx, 3);
				{
				setState(449);
				json_struct();
				}
				break;
			case OpenCloseCurly:
				enterOuterAlt(_localctx, 4);
				{
				setState(450);
				match(OpenCloseCurly);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class ArgContext extends ParserRuleContext {
		public TerminalNode IDENTIFIER() { return getToken(ExtentedHATPParser.IDENTIFIER, 0); }
		public ArgContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_arg; }
	}

	public final ArgContext arg() throws RecognitionException {
		ArgContext _localctx = new ArgContext(_ctx, getState());
		enterRule(_localctx, 46, RULE_arg);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(453);
			match(IDENTIFIER);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class NameContext extends ParserRuleContext {
		public TerminalNode IDENTIFIER() { return getToken(ExtentedHATPParser.IDENTIFIER, 0); }
		public NameContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_name; }
	}

	public final NameContext name() throws RecognitionException {
		NameContext _localctx = new NameContext(_ctx, getState());
		enterRule(_localctx, 48, RULE_name);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(455);
			match(IDENTIFIER);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Topic_nameContext extends ParserRuleContext {
		public List<TerminalNode> SLASH() { return getTokens(ExtentedHATPParser.SLASH); }
		public TerminalNode SLASH(int i) {
			return getToken(ExtentedHATPParser.SLASH, i);
		}
		public List<TerminalNode> IDENTIFIER() { return getTokens(ExtentedHATPParser.IDENTIFIER); }
		public TerminalNode IDENTIFIER(int i) {
			return getToken(ExtentedHATPParser.IDENTIFIER, i);
		}
		public List<TerminalNode> STRING() { return getTokens(ExtentedHATPParser.STRING); }
		public TerminalNode STRING(int i) {
			return getToken(ExtentedHATPParser.STRING, i);
		}
		public Topic_nameContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_topic_name; }
	}

	public final Topic_nameContext topic_name() throws RecognitionException {
		Topic_nameContext _localctx = new Topic_nameContext(_ctx, getState());
		enterRule(_localctx, 50, RULE_topic_name);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(458);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==STRING) {
				{
				setState(457);
				match(STRING);
				}
			}

			{
			setState(460);
			match(SLASH);
			setState(461);
			match(IDENTIFIER);
			setState(466);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==SLASH) {
				{
				{
				setState(462);
				match(SLASH);
				setState(463);
				match(IDENTIFIER);
				}
				}
				setState(468);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
			setState(470);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==STRING) {
				{
				setState(469);
				match(STRING);
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Json_structContext extends ParserRuleContext {
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public List<Json_pairContext> json_pair() {
			return getRuleContexts(Json_pairContext.class);
		}
		public Json_pairContext json_pair(int i) {
			return getRuleContext(Json_pairContext.class,i);
		}
		public List<TerminalNode> Comma() { return getTokens(ExtentedHATPParser.Comma); }
		public TerminalNode Comma(int i) {
			return getToken(ExtentedHATPParser.Comma, i);
		}
		public Json_structContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_json_struct; }
	}

	public final Json_structContext json_struct() throws RecognitionException {
		Json_structContext _localctx = new Json_structContext(_ctx, getState());
		enterRule(_localctx, 52, RULE_json_struct);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(472);
			match(OpenCurly);
			setState(476);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==STRING || _la==IDENTIFIER) {
				{
				{
				setState(473);
				json_pair();
				}
				}
				setState(478);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(483);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==Comma) {
				{
				{
				setState(479);
				match(Comma);
				setState(480);
				json_pair();
				}
				}
				setState(485);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(486);
			match(CloseCurly);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Json_pairContext extends ParserRuleContext {
		public VarnameContext varname() {
			return getRuleContext(VarnameContext.class,0);
		}
		public TerminalNode COLON() { return getToken(ExtentedHATPParser.COLON, 0); }
		public ValueContext value() {
			return getRuleContext(ValueContext.class,0);
		}
		public List<TerminalNode> STRING() { return getTokens(ExtentedHATPParser.STRING); }
		public TerminalNode STRING(int i) {
			return getToken(ExtentedHATPParser.STRING, i);
		}
		public Json_pairContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_json_pair; }
	}

	public final Json_pairContext json_pair() throws RecognitionException {
		Json_pairContext _localctx = new Json_pairContext(_ctx, getState());
		enterRule(_localctx, 54, RULE_json_pair);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(489);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==STRING) {
				{
				setState(488);
				match(STRING);
				}
			}

			setState(491);
			varname();
			setState(493);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==STRING) {
				{
				setState(492);
				match(STRING);
				}
			}

			setState(495);
			match(COLON);
			setState(497);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==STRING) {
				{
				setState(496);
				match(STRING);
				}
			}

			setState(499);
			value();
			setState(501);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,38,_ctx) ) {
			case 1:
				{
				setState(500);
				match(STRING);
				}
				break;
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class ValueContext extends ParserRuleContext {
		public TerminalNode IDENTIFIER() { return getToken(ExtentedHATPParser.IDENTIFIER, 0); }
		public TerminalNode NUMBER() { return getToken(ExtentedHATPParser.NUMBER, 0); }
		public ValueContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value; }
	}

	public final ValueContext value() throws RecognitionException {
		ValueContext _localctx = new ValueContext(_ctx, getState());
		enterRule(_localctx, 56, RULE_value);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(503);
			_la = _input.LA(1);
			if ( !(_la==IDENTIFIER || _la==NUMBER) ) {
			_errHandler.recoverInline(this);
			}
			else {
				if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
				_errHandler.reportMatch(this);
				consume();
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Recognition_blocContext extends ParserRuleContext {
		public TerminalNode RECOGNITION() { return getToken(ExtentedHATPParser.RECOGNITION, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public Sequence_blocContext sequence_bloc() {
			return getRuleContext(Sequence_blocContext.class,0);
		}
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public Parameters_blocContext parameters_bloc() {
			return getRuleContext(Parameters_blocContext.class,0);
		}
		public Recognition_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_recognition_bloc; }
	}

	public final Recognition_blocContext recognition_bloc() throws RecognitionException {
		Recognition_blocContext _localctx = new Recognition_blocContext(_ctx, getState());
		enterRule(_localctx, 58, RULE_recognition_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(505);
			match(RECOGNITION);
			setState(506);
			match(OpenCurly);
			setState(507);
			sequence_bloc();
			setState(509);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==PARAMETERS) {
				{
				setState(508);
				parameters_bloc();
				}
			}

			setState(511);
			match(CloseCurly);
			setState(512);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Sequence_blocContext extends ParserRuleContext {
		public TerminalNode SEQUENCE() { return getToken(ExtentedHATPParser.SEQUENCE, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<SequenceContext> sequence() {
			return getRuleContexts(SequenceContext.class);
		}
		public SequenceContext sequence(int i) {
			return getRuleContext(SequenceContext.class,i);
		}
		public Sequence_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_sequence_bloc; }
	}

	public final Sequence_blocContext sequence_bloc() throws RecognitionException {
		Sequence_blocContext _localctx = new Sequence_blocContext(_ctx, getState());
		enterRule(_localctx, 60, RULE_sequence_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(514);
			match(SEQUENCE);
			setState(515);
			match(OpenCurly);
			setState(519);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (((((_la - 9)) & ~0x3f) == 0 && ((1L << (_la - 9)) & 72127962782105601L) != 0)) {
				{
				{
				setState(516);
				sequence();
				}
				}
				setState(521);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(522);
			match(CloseCurly);
			setState(523);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class SequenceContext extends ParserRuleContext {
		public SubjectContext subject() {
			return getRuleContext(SubjectContext.class,0);
		}
		public PredicateContext predicate() {
			return getRuleContext(PredicateContext.class,0);
		}
		public ObjectContext object() {
			return getRuleContext(ObjectContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public TerminalNode NOT() { return getToken(ExtentedHATPParser.NOT, 0); }
		public TerminalNode REQUIRED() { return getToken(ExtentedHATPParser.REQUIRED, 0); }
		public SequenceContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_sequence; }
	}

	public final SequenceContext sequence() throws RecognitionException {
		SequenceContext _localctx = new SequenceContext(_ctx, getState());
		enterRule(_localctx, 62, RULE_sequence);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(526);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==NOT) {
				{
				setState(525);
				match(NOT);
				}
			}

			setState(528);
			subject();
			setState(529);
			predicate();
			setState(530);
			object();
			setState(532);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==REQUIRED) {
				{
				setState(531);
				match(REQUIRED);
				}
			}

			setState(534);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Parameters_blocContext extends ParserRuleContext {
		public TerminalNode PARAMETERS() { return getToken(ExtentedHATPParser.PARAMETERS, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<ParameterContext> parameter() {
			return getRuleContexts(ParameterContext.class);
		}
		public ParameterContext parameter(int i) {
			return getRuleContext(ParameterContext.class,i);
		}
		public Parameters_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_parameters_bloc; }
	}

	public final Parameters_blocContext parameters_bloc() throws RecognitionException {
		Parameters_blocContext _localctx = new Parameters_blocContext(_ctx, getState());
		enterRule(_localctx, 64, RULE_parameters_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(536);
			match(PARAMETERS);
			setState(537);
			match(OpenCurly);
			setState(541);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==IDENTIFIER) {
				{
				{
				setState(538);
				parameter();
				}
				}
				setState(543);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(544);
			match(CloseCurly);
			setState(545);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class ParameterContext extends ParserRuleContext {
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode OpenPar() { return getToken(ExtentedHATPParser.OpenPar, 0); }
		public ValueContext value() {
			return getRuleContext(ValueContext.class,0);
		}
		public TerminalNode ClosePar() { return getToken(ExtentedHATPParser.ClosePar, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public ParameterContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_parameter; }
	}

	public final ParameterContext parameter() throws RecognitionException {
		ParameterContext _localctx = new ParameterContext(_ctx, getState());
		enterRule(_localctx, 66, RULE_parameter);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(547);
			name();
			setState(548);
			match(OpenPar);
			setState(549);
			value();
			setState(550);
			match(ClosePar);
			setState(551);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Attentes_blocContext extends ParserRuleContext {
		public TerminalNode ATTENTES() { return getToken(ExtentedHATPParser.ATTENTES, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<RoleContext> role() {
			return getRuleContexts(RoleContext.class);
		}
		public RoleContext role(int i) {
			return getRuleContext(RoleContext.class,i);
		}
		public Attentes_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attentes_bloc; }
	}

	public final Attentes_blocContext attentes_bloc() throws RecognitionException {
		Attentes_blocContext _localctx = new Attentes_blocContext(_ctx, getState());
		enterRule(_localctx, 68, RULE_attentes_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(553);
			match(ATTENTES);
			setState(554);
			match(OpenCurly);
			setState(558);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==ROLE) {
				{
				{
				setState(555);
				role();
				}
				}
				setState(560);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(561);
			match(CloseCurly);
			setState(562);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class RoleContext extends ParserRuleContext {
		public TerminalNode ROLE() { return getToken(ExtentedHATPParser.ROLE, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public ConditionsContext conditions() {
			return getRuleContext(ConditionsContext.class,0);
		}
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<AttenteContext> attente() {
			return getRuleContexts(AttenteContext.class);
		}
		public AttenteContext attente(int i) {
			return getRuleContext(AttenteContext.class,i);
		}
		public RoleContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_role; }
	}

	public final RoleContext role() throws RecognitionException {
		RoleContext _localctx = new RoleContext(_ctx, getState());
		enterRule(_localctx, 70, RULE_role);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(564);
			match(ROLE);
			setState(565);
			name();
			setState(566);
			match(OpenCurly);
			setState(567);
			conditions();
			setState(571);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==ATTENTE) {
				{
				{
				setState(568);
				attente();
				}
				}
				setState(573);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(574);
			match(CloseCurly);
			setState(575);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class AttenteContext extends ParserRuleContext {
		public TerminalNode ATTENTE() { return getToken(ExtentedHATPParser.ATTENTE, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public ConditionsContext conditions() {
			return getRuleContext(ConditionsContext.class,0);
		}
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public AttenteContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attente; }
	}

	public final AttenteContext attente() throws RecognitionException {
		AttenteContext _localctx = new AttenteContext(_ctx, getState());
		enterRule(_localctx, 72, RULE_attente);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(577);
			match(ATTENTE);
			setState(578);
			name();
			setState(579);
			match(OpenCurly);
			setState(580);
			conditions();
			setState(581);
			match(CloseCurly);
			setState(582);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class ConditionsContext extends ParserRuleContext {
		public TerminalNode CONDITIONS() { return getToken(ExtentedHATPParser.CONDITIONS, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<QueryContext> query() {
			return getRuleContexts(QueryContext.class);
		}
		public QueryContext query(int i) {
			return getRuleContext(QueryContext.class,i);
		}
		public List<TripletContext> triplet() {
			return getRuleContexts(TripletContext.class);
		}
		public TripletContext triplet(int i) {
			return getRuleContext(TripletContext.class,i);
		}
		public ConditionsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_conditions; }
	}

	public final ConditionsContext conditions() throws RecognitionException {
		ConditionsContext _localctx = new ConditionsContext(_ctx, getState());
		enterRule(_localctx, 74, RULE_conditions);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(584);
			match(CONDITIONS);
			setState(585);
			match(OpenCurly);
			setState(590);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (((((_la - 9)) & ~0x3f) == 0 && ((1L << (_la - 9)) & 72127962782106625L) != 0)) {
				{
				setState(588);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case SELECT:
					{
					setState(586);
					query();
					}
					break;
				case NOT:
				case QUESTIONMARK:
				case IDENTIFIER:
					{
					setState(587);
					triplet();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(592);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(593);
			match(CloseCurly);
			setState(594);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Tasks_blocContext extends ParserRuleContext {
		public TerminalNode TASKS() { return getToken(ExtentedHATPParser.TASKS, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<TaskContext> task() {
			return getRuleContexts(TaskContext.class);
		}
		public TaskContext task(int i) {
			return getRuleContext(TaskContext.class,i);
		}
		public Tasks_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_tasks_bloc; }
	}

	public final Tasks_blocContext tasks_bloc() throws RecognitionException {
		Tasks_blocContext _localctx = new Tasks_blocContext(_ctx, getState());
		enterRule(_localctx, 76, RULE_tasks_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(596);
			match(TASKS);
			setState(597);
			match(OpenCurly);
			setState(601);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==TASK) {
				{
				{
				setState(598);
				task();
				}
				}
				setState(603);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(604);
			match(CloseCurly);
			setState(605);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class TaskContext extends ParserRuleContext {
		public TerminalNode TASK() { return getToken(ExtentedHATPParser.TASK, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode OpenPar() { return getToken(ExtentedHATPParser.OpenPar, 0); }
		public TerminalNode ClosePar() { return getToken(ExtentedHATPParser.ClosePar, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<ArgumentsContext> arguments() {
			return getRuleContexts(ArgumentsContext.class);
		}
		public ArgumentsContext arguments(int i) {
			return getRuleContext(ArgumentsContext.class,i);
		}
		public List<TerminalNode> Comma() { return getTokens(ExtentedHATPParser.Comma); }
		public TerminalNode Comma(int i) {
			return getToken(ExtentedHATPParser.Comma, i);
		}
		public Entry_stateContext entry_state() {
			return getRuleContext(Entry_stateContext.class,0);
		}
		public GoalContext goal() {
			return getRuleContext(GoalContext.class,0);
		}
		public Methods_blocContext methods_bloc() {
			return getRuleContext(Methods_blocContext.class,0);
		}
		public Cost_blocContext cost_bloc() {
			return getRuleContext(Cost_blocContext.class,0);
		}
		public Duration_blocContext duration_bloc() {
			return getRuleContext(Duration_blocContext.class,0);
		}
		public Parameters_blocContext parameters_bloc() {
			return getRuleContext(Parameters_blocContext.class,0);
		}
		public TaskContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_task; }
	}

	public final TaskContext task() throws RecognitionException {
		TaskContext _localctx = new TaskContext(_ctx, getState());
		enterRule(_localctx, 78, RULE_task);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(607);
			match(TASK);
			setState(608);
			name();
			setState(609);
			match(OpenPar);
			setState(613);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==IDENTIFIER) {
				{
				{
				setState(610);
				arguments();
				}
				}
				setState(615);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(620);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==Comma) {
				{
				{
				setState(616);
				match(Comma);
				setState(617);
				arguments();
				}
				}
				setState(622);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(623);
			match(ClosePar);
			setState(624);
			match(OpenCurly);
			setState(626);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==ENTRYSTATE) {
				{
				setState(625);
				entry_state();
				}
			}

			setState(629);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==GOAL) {
				{
				setState(628);
				goal();
				}
			}

			setState(632);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==METHODS) {
				{
				setState(631);
				methods_bloc();
				}
			}

			setState(635);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==COST) {
				{
				setState(634);
				cost_bloc();
				}
			}

			setState(638);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==DURATION) {
				{
				setState(637);
				duration_bloc();
				}
			}

			setState(641);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==PARAMETERS) {
				{
				setState(640);
				parameters_bloc();
				}
			}

			setState(643);
			match(CloseCurly);
			setState(644);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Entry_stateContext extends ParserRuleContext {
		public TerminalNode ENTRYSTATE() { return getToken(ExtentedHATPParser.ENTRYSTATE, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<TripletContext> triplet() {
			return getRuleContexts(TripletContext.class);
		}
		public TripletContext triplet(int i) {
			return getRuleContext(TripletContext.class,i);
		}
		public List<QueryContext> query() {
			return getRuleContexts(QueryContext.class);
		}
		public QueryContext query(int i) {
			return getRuleContext(QueryContext.class,i);
		}
		public Entry_stateContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_entry_state; }
	}

	public final Entry_stateContext entry_state() throws RecognitionException {
		Entry_stateContext _localctx = new Entry_stateContext(_ctx, getState());
		enterRule(_localctx, 80, RULE_entry_state);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(646);
			match(ENTRYSTATE);
			setState(647);
			match(OpenCurly);
			setState(652);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (((((_la - 9)) & ~0x3f) == 0 && ((1L << (_la - 9)) & 72127962782106625L) != 0)) {
				{
				setState(650);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case NOT:
				case QUESTIONMARK:
				case IDENTIFIER:
					{
					setState(648);
					triplet();
					}
					break;
				case SELECT:
					{
					setState(649);
					query();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(654);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(655);
			match(CloseCurly);
			setState(656);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class GoalContext extends ParserRuleContext {
		public TerminalNode GOAL() { return getToken(ExtentedHATPParser.GOAL, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<TripletContext> triplet() {
			return getRuleContexts(TripletContext.class);
		}
		public TripletContext triplet(int i) {
			return getRuleContext(TripletContext.class,i);
		}
		public GoalContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_goal; }
	}

	public final GoalContext goal() throws RecognitionException {
		GoalContext _localctx = new GoalContext(_ctx, getState());
		enterRule(_localctx, 82, RULE_goal);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(658);
			match(GOAL);
			setState(659);
			match(OpenCurly);
			setState(663);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (((((_la - 9)) & ~0x3f) == 0 && ((1L << (_la - 9)) & 72127962782105601L) != 0)) {
				{
				{
				setState(660);
				triplet();
				}
				}
				setState(665);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(666);
			match(CloseCurly);
			setState(667);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Methods_blocContext extends ParserRuleContext {
		public TerminalNode METHODS() { return getToken(ExtentedHATPParser.METHODS, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public List<MethodContext> method() {
			return getRuleContexts(MethodContext.class);
		}
		public MethodContext method(int i) {
			return getRuleContext(MethodContext.class,i);
		}
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public Methods_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_methods_bloc; }
	}

	public final Methods_blocContext methods_bloc() throws RecognitionException {
		Methods_blocContext _localctx = new Methods_blocContext(_ctx, getState());
		enterRule(_localctx, 84, RULE_methods_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(669);
			match(METHODS);
			setState(670);
			match(OpenCurly);
			setState(671);
			method();
			setState(675);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==IDENTIFIER || _la==NUMBER) {
				{
				{
				setState(672);
				method();
				}
				}
				setState(677);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(678);
			match(CloseCurly);
			setState(679);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class MethodContext extends ParserRuleContext {
		public Id_methodContext id_method() {
			return getRuleContext(Id_methodContext.class,0);
		}
		public TerminalNode COLON() { return getToken(ExtentedHATPParser.COLON, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public List<Preconditions_blocContext> preconditions_bloc() {
			return getRuleContexts(Preconditions_blocContext.class);
		}
		public Preconditions_blocContext preconditions_bloc(int i) {
			return getRuleContext(Preconditions_blocContext.class,i);
		}
		public List<Subtask_blocContext> subtask_bloc() {
			return getRuleContexts(Subtask_blocContext.class);
		}
		public Subtask_blocContext subtask_bloc(int i) {
			return getRuleContext(Subtask_blocContext.class,i);
		}
		public MethodContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_method; }
	}

	public final MethodContext method() throws RecognitionException {
		MethodContext _localctx = new MethodContext(_ctx, getState());
		enterRule(_localctx, 86, RULE_method);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(681);
			id_method();
			setState(682);
			match(COLON);
			setState(683);
			match(OpenCurly);
			setState(687);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==PRECONDITIONS) {
				{
				{
				setState(684);
				preconditions_bloc();
				}
				}
				setState(689);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(693);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==SUBTASKS) {
				{
				{
				setState(690);
				subtask_bloc();
				}
				}
				setState(695);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(696);
			match(CloseCurly);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Subtask_blocContext extends ParserRuleContext {
		public TerminalNode SUBTASKS() { return getToken(ExtentedHATPParser.SUBTASKS, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<Subtask_lineContext> subtask_line() {
			return getRuleContexts(Subtask_lineContext.class);
		}
		public Subtask_lineContext subtask_line(int i) {
			return getRuleContext(Subtask_lineContext.class,i);
		}
		public Subtask_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_subtask_bloc; }
	}

	public final Subtask_blocContext subtask_bloc() throws RecognitionException {
		Subtask_blocContext _localctx = new Subtask_blocContext(_ctx, getState());
		enterRule(_localctx, 88, RULE_subtask_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(698);
			match(SUBTASKS);
			setState(699);
			match(OpenCurly);
			setState(703);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==NUMBER) {
				{
				{
				setState(700);
				subtask_line();
				}
				}
				setState(705);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(706);
			match(CloseCurly);
			setState(707);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Subtask_lineContext extends ParserRuleContext {
		public List<IdContext> id() {
			return getRuleContexts(IdContext.class);
		}
		public IdContext id(int i) {
			return getRuleContext(IdContext.class,i);
		}
		public TerminalNode COLON() { return getToken(ExtentedHATPParser.COLON, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode OpenPar() { return getToken(ExtentedHATPParser.OpenPar, 0); }
		public TerminalNode ClosePar() { return getToken(ExtentedHATPParser.ClosePar, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<ArgContext> arg() {
			return getRuleContexts(ArgContext.class);
		}
		public ArgContext arg(int i) {
			return getRuleContext(ArgContext.class,i);
		}
		public List<TerminalNode> Comma() { return getTokens(ExtentedHATPParser.Comma); }
		public TerminalNode Comma(int i) {
			return getToken(ExtentedHATPParser.Comma, i);
		}
		public List<OperatorContext> operator() {
			return getRuleContexts(OperatorContext.class);
		}
		public OperatorContext operator(int i) {
			return getRuleContext(OperatorContext.class,i);
		}
		public Subtask_lineContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_subtask_line; }
	}

	public final Subtask_lineContext subtask_line() throws RecognitionException {
		Subtask_lineContext _localctx = new Subtask_lineContext(_ctx, getState());
		enterRule(_localctx, 90, RULE_subtask_line);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(709);
			id();
			setState(710);
			match(COLON);
			setState(711);
			name();
			setState(712);
			match(OpenPar);
			setState(716);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==IDENTIFIER) {
				{
				{
				setState(713);
				arg();
				}
				}
				setState(718);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(723);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==Comma) {
				{
				{
				setState(719);
				match(Comma);
				setState(720);
				arg();
				}
				}
				setState(725);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(726);
			match(ClosePar);
			setState(732);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while ((((_la) & ~0x3f) == 0 && ((1L << _la) & 281406257233920L) != 0)) {
				{
				{
				setState(727);
				operator();
				setState(728);
				id();
				}
				}
				setState(734);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(735);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class EventContext extends ParserRuleContext {
		public TerminalNode EVENT() { return getToken(ExtentedHATPParser.EVENT, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<QueryContext> query() {
			return getRuleContexts(QueryContext.class);
		}
		public QueryContext query(int i) {
			return getRuleContext(QueryContext.class,i);
		}
		public EventContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_event; }
	}

	public final EventContext event() throws RecognitionException {
		EventContext _localctx = new EventContext(_ctx, getState());
		enterRule(_localctx, 92, RULE_event);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(737);
			match(EVENT);
			setState(738);
			match(OpenCurly);
			setState(742);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==SELECT) {
				{
				{
				setState(739);
				query();
				}
				}
				setState(744);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(745);
			match(CloseCurly);
			setState(746);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Priority_levelContext extends ParserRuleContext {
		public TerminalNode PRIORITY_LEVEL() { return getToken(ExtentedHATPParser.PRIORITY_LEVEL, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public Numeric_valueContext numeric_value() {
			return getRuleContext(Numeric_valueContext.class,0);
		}
		public List<TerminalNode> SEMICOLON() { return getTokens(ExtentedHATPParser.SEMICOLON); }
		public TerminalNode SEMICOLON(int i) {
			return getToken(ExtentedHATPParser.SEMICOLON, i);
		}
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public Priority_levelContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_priority_level; }
	}

	public final Priority_levelContext priority_level() throws RecognitionException {
		Priority_levelContext _localctx = new Priority_levelContext(_ctx, getState());
		enterRule(_localctx, 94, RULE_priority_level);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(748);
			match(PRIORITY_LEVEL);
			setState(749);
			match(OpenCurly);
			setState(750);
			numeric_value();
			setState(751);
			match(SEMICOLON);
			setState(752);
			match(CloseCurly);
			setState(753);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class ObjectifsContext extends ParserRuleContext {
		public TerminalNode OBJECTIVES() { return getToken(ExtentedHATPParser.OBJECTIVES, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<QueryContext> query() {
			return getRuleContexts(QueryContext.class);
		}
		public QueryContext query(int i) {
			return getRuleContext(QueryContext.class,i);
		}
		public List<TripletContext> triplet() {
			return getRuleContexts(TripletContext.class);
		}
		public TripletContext triplet(int i) {
			return getRuleContext(TripletContext.class,i);
		}
		public ObjectifsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_objectifs; }
	}

	public final ObjectifsContext objectifs() throws RecognitionException {
		ObjectifsContext _localctx = new ObjectifsContext(_ctx, getState());
		enterRule(_localctx, 96, RULE_objectifs);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(755);
			match(OBJECTIVES);
			setState(756);
			match(OpenCurly);
			setState(761);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (((((_la - 9)) & ~0x3f) == 0 && ((1L << (_la - 9)) & 72127962782106625L) != 0)) {
				{
				setState(759);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case SELECT:
					{
					setState(757);
					query();
					}
					break;
				case NOT:
				case QUESTIONMARK:
				case IDENTIFIER:
					{
					setState(758);
					triplet();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(763);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(764);
			match(CloseCurly);
			setState(765);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Priorities_blocContext extends ParserRuleContext {
		public TerminalNode PRIORITIES() { return getToken(ExtentedHATPParser.PRIORITIES, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<PriorityContext> priority() {
			return getRuleContexts(PriorityContext.class);
		}
		public PriorityContext priority(int i) {
			return getRuleContext(PriorityContext.class,i);
		}
		public Priorities_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_priorities_bloc; }
	}

	public final Priorities_blocContext priorities_bloc() throws RecognitionException {
		Priorities_blocContext _localctx = new Priorities_blocContext(_ctx, getState());
		enterRule(_localctx, 98, RULE_priorities_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(767);
			match(PRIORITIES);
			setState(768);
			match(OpenCurly);
			setState(772);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==PRIORITY) {
				{
				{
				setState(769);
				priority();
				}
				}
				setState(774);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(775);
			match(CloseCurly);
			setState(776);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class PriorityContext extends ParserRuleContext {
		public TerminalNode PRIORITY() { return getToken(ExtentedHATPParser.PRIORITY, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<EventContext> event() {
			return getRuleContexts(EventContext.class);
		}
		public EventContext event(int i) {
			return getRuleContext(EventContext.class,i);
		}
		public List<Priority_levelContext> priority_level() {
			return getRuleContexts(Priority_levelContext.class);
		}
		public Priority_levelContext priority_level(int i) {
			return getRuleContext(Priority_levelContext.class,i);
		}
		public List<ObjectifsContext> objectifs() {
			return getRuleContexts(ObjectifsContext.class);
		}
		public ObjectifsContext objectifs(int i) {
			return getRuleContext(ObjectifsContext.class,i);
		}
		public PriorityContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_priority; }
	}

	public final PriorityContext priority() throws RecognitionException {
		PriorityContext _localctx = new PriorityContext(_ctx, getState());
		enterRule(_localctx, 100, RULE_priority);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(778);
			match(PRIORITY);
			setState(779);
			name();
			setState(780);
			match(OpenCurly);
			setState(786);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while ((((_la) & ~0x3f) == 0 && ((1L << _la) & 30064771072L) != 0)) {
				{
				setState(784);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case EVENT:
					{
					setState(781);
					event();
					}
					break;
				case PRIORITY_LEVEL:
					{
					setState(782);
					priority_level();
					}
					break;
				case OBJECTIVES:
					{
					setState(783);
					objectifs();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(788);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(789);
			match(CloseCurly);
			setState(790);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Pratices_frames_blocContext extends ParserRuleContext {
		public TerminalNode PRACTICE_FRAMES() { return getToken(ExtentedHATPParser.PRACTICE_FRAMES, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<Practice_frameContext> practice_frame() {
			return getRuleContexts(Practice_frameContext.class);
		}
		public Practice_frameContext practice_frame(int i) {
			return getRuleContext(Practice_frameContext.class,i);
		}
		public Pratices_frames_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_pratices_frames_bloc; }
	}

	public final Pratices_frames_blocContext pratices_frames_bloc() throws RecognitionException {
		Pratices_frames_blocContext _localctx = new Pratices_frames_blocContext(_ctx, getState());
		enterRule(_localctx, 102, RULE_pratices_frames_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(792);
			match(PRACTICE_FRAMES);
			setState(793);
			match(OpenCurly);
			setState(797);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==PRACTICE_FRAME) {
				{
				{
				setState(794);
				practice_frame();
				}
				}
				setState(799);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(800);
			match(CloseCurly);
			setState(801);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Practice_frameContext extends ParserRuleContext {
		public TerminalNode PRACTICE_FRAME() { return getToken(ExtentedHATPParser.PRACTICE_FRAME, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<Description_practiceContext> description_practice() {
			return getRuleContexts(Description_practiceContext.class);
		}
		public Description_practiceContext description_practice(int i) {
			return getRuleContext(Description_practiceContext.class,i);
		}
		public List<Conditions_practicesContext> conditions_practices() {
			return getRuleContexts(Conditions_practicesContext.class);
		}
		public Conditions_practicesContext conditions_practices(int i) {
			return getRuleContext(Conditions_practicesContext.class,i);
		}
		public List<Practices_listContext> practices_list() {
			return getRuleContexts(Practices_listContext.class);
		}
		public Practices_listContext practices_list(int i) {
			return getRuleContext(Practices_listContext.class,i);
		}
		public List<Roles_with_conditionsContext> roles_with_conditions() {
			return getRuleContexts(Roles_with_conditionsContext.class);
		}
		public Roles_with_conditionsContext roles_with_conditions(int i) {
			return getRuleContext(Roles_with_conditionsContext.class,i);
		}
		public List<Objects_blocContext> objects_bloc() {
			return getRuleContexts(Objects_blocContext.class);
		}
		public Objects_blocContext objects_bloc(int i) {
			return getRuleContext(Objects_blocContext.class,i);
		}
		public List<Rules_blocContext> rules_bloc() {
			return getRuleContexts(Rules_blocContext.class);
		}
		public Rules_blocContext rules_bloc(int i) {
			return getRuleContext(Rules_blocContext.class,i);
		}
		public Practice_frameContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_practice_frame; }
	}

	public final Practice_frameContext practice_frame() throws RecognitionException {
		Practice_frameContext _localctx = new Practice_frameContext(_ctx, getState());
		enterRule(_localctx, 104, RULE_practice_frame);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(803);
			match(PRACTICE_FRAME);
			setState(804);
			name();
			setState(805);
			match(OpenCurly);
			setState(814);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==DESCRIPTION || _la==CONDITIONS || ((((_la - 73)) & ~0x3f) == 0 && ((1L << (_la - 73)) & 15L) != 0)) {
				{
				setState(812);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case DESCRIPTION:
					{
					setState(806);
					description_practice();
					}
					break;
				case CONDITIONS:
					{
					setState(807);
					conditions_practices();
					}
					break;
				case PRACTICES_LIST:
					{
					setState(808);
					practices_list();
					}
					break;
				case ROLES:
					{
					setState(809);
					roles_with_conditions();
					}
					break;
				case OBJECTS:
					{
					setState(810);
					objects_bloc();
					}
					break;
				case RULES:
					{
					setState(811);
					rules_bloc();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(816);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(817);
			match(CloseCurly);
			setState(818);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Practices_listContext extends ParserRuleContext {
		public TerminalNode PRACTICES_LIST() { return getToken(ExtentedHATPParser.PRACTICES_LIST, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<Practice_nameContext> practice_name() {
			return getRuleContexts(Practice_nameContext.class);
		}
		public Practice_nameContext practice_name(int i) {
			return getRuleContext(Practice_nameContext.class,i);
		}
		public Practices_listContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_practices_list; }
	}

	public final Practices_listContext practices_list() throws RecognitionException {
		Practices_listContext _localctx = new Practices_listContext(_ctx, getState());
		enterRule(_localctx, 106, RULE_practices_list);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(820);
			match(PRACTICES_LIST);
			setState(821);
			match(OpenCurly);
			setState(825);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==MINUS) {
				{
				{
				setState(822);
				practice_name();
				}
				}
				setState(827);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(828);
			match(CloseCurly);
			setState(829);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Roles_listContext extends ParserRuleContext {
		public TerminalNode ROLES() { return getToken(ExtentedHATPParser.ROLES, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<Role_nameContext> role_name() {
			return getRuleContexts(Role_nameContext.class);
		}
		public Role_nameContext role_name(int i) {
			return getRuleContext(Role_nameContext.class,i);
		}
		public Roles_listContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_roles_list; }
	}

	public final Roles_listContext roles_list() throws RecognitionException {
		Roles_listContext _localctx = new Roles_listContext(_ctx, getState());
		enterRule(_localctx, 108, RULE_roles_list);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(831);
			match(ROLES);
			setState(832);
			match(OpenCurly);
			setState(836);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==MINUS) {
				{
				{
				setState(833);
				role_name();
				}
				}
				setState(838);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(839);
			match(CloseCurly);
			setState(840);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Role_nameContext extends ParserRuleContext {
		public TerminalNode MINUS() { return getToken(ExtentedHATPParser.MINUS, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public Role_nameContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_role_name; }
	}

	public final Role_nameContext role_name() throws RecognitionException {
		Role_nameContext _localctx = new Role_nameContext(_ctx, getState());
		enterRule(_localctx, 110, RULE_role_name);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(842);
			match(MINUS);
			setState(843);
			name();
			setState(844);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Practice_nameContext extends ParserRuleContext {
		public TerminalNode MINUS() { return getToken(ExtentedHATPParser.MINUS, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public Practice_nameContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_practice_name; }
	}

	public final Practice_nameContext practice_name() throws RecognitionException {
		Practice_nameContext _localctx = new Practice_nameContext(_ctx, getState());
		enterRule(_localctx, 112, RULE_practice_name);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(846);
			match(MINUS);
			setState(847);
			name();
			setState(848);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Description_practiceContext extends ParserRuleContext {
		public TerminalNode DESCRIPTION() { return getToken(ExtentedHATPParser.DESCRIPTION, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public List<TerminalNode> STRING() { return getTokens(ExtentedHATPParser.STRING); }
		public TerminalNode STRING(int i) {
			return getToken(ExtentedHATPParser.STRING, i);
		}
		public SentenceContext sentence() {
			return getRuleContext(SentenceContext.class,0);
		}
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public Description_practiceContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_description_practice; }
	}

	public final Description_practiceContext description_practice() throws RecognitionException {
		Description_practiceContext _localctx = new Description_practiceContext(_ctx, getState());
		enterRule(_localctx, 114, RULE_description_practice);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(850);
			match(DESCRIPTION);
			setState(851);
			match(OpenCurly);
			setState(852);
			match(STRING);
			setState(853);
			sentence();
			setState(854);
			match(STRING);
			setState(855);
			match(CloseCurly);
			setState(856);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Objects_blocContext extends ParserRuleContext {
		public TerminalNode OBJECTS() { return getToken(ExtentedHATPParser.OBJECTS, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<Object_itemContext> object_item() {
			return getRuleContexts(Object_itemContext.class);
		}
		public Object_itemContext object_item(int i) {
			return getRuleContext(Object_itemContext.class,i);
		}
		public Objects_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_objects_bloc; }
	}

	public final Objects_blocContext objects_bloc() throws RecognitionException {
		Objects_blocContext _localctx = new Objects_blocContext(_ctx, getState());
		enterRule(_localctx, 116, RULE_objects_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(858);
			match(OBJECTS);
			setState(859);
			match(OpenCurly);
			setState(863);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==MINUS) {
				{
				{
				setState(860);
				object_item();
				}
				}
				setState(865);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(866);
			match(CloseCurly);
			setState(867);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Object_itemContext extends ParserRuleContext {
		public TerminalNode MINUS() { return getToken(ExtentedHATPParser.MINUS, 0); }
		public ObjectContext object() {
			return getRuleContext(ObjectContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public Object_itemContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_object_item; }
	}

	public final Object_itemContext object_item() throws RecognitionException {
		Object_itemContext _localctx = new Object_itemContext(_ctx, getState());
		enterRule(_localctx, 118, RULE_object_item);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(869);
			match(MINUS);
			setState(870);
			object();
			setState(871);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Rules_blocContext extends ParserRuleContext {
		public TerminalNode RULES() { return getToken(ExtentedHATPParser.RULES, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<Rule_itemContext> rule_item() {
			return getRuleContexts(Rule_itemContext.class);
		}
		public Rule_itemContext rule_item(int i) {
			return getRuleContext(Rule_itemContext.class,i);
		}
		public Rules_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_rules_bloc; }
	}

	public final Rules_blocContext rules_bloc() throws RecognitionException {
		Rules_blocContext _localctx = new Rules_blocContext(_ctx, getState());
		enterRule(_localctx, 120, RULE_rules_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(873);
			match(RULES);
			setState(874);
			match(OpenCurly);
			setState(876); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(875);
				rule_item();
				}
				}
				setState(878); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( _la==MINUS );
			setState(880);
			match(CloseCurly);
			setState(881);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Rule_itemContext extends ParserRuleContext {
		public TerminalNode MINUS() { return getToken(ExtentedHATPParser.MINUS, 0); }
		public List<TerminalNode> STRING() { return getTokens(ExtentedHATPParser.STRING); }
		public TerminalNode STRING(int i) {
			return getToken(ExtentedHATPParser.STRING, i);
		}
		public SentenceContext sentence() {
			return getRuleContext(SentenceContext.class,0);
		}
		public Rule_itemContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_rule_item; }
	}

	public final Rule_itemContext rule_item() throws RecognitionException {
		Rule_itemContext _localctx = new Rule_itemContext(_ctx, getState());
		enterRule(_localctx, 122, RULE_rule_item);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(883);
			match(MINUS);
			setState(884);
			match(STRING);
			setState(885);
			sentence();
			setState(886);
			match(STRING);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Roles_with_conditionsContext extends ParserRuleContext {
		public TerminalNode ROLES() { return getToken(ExtentedHATPParser.ROLES, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<Role_with_conditionContext> role_with_condition() {
			return getRuleContexts(Role_with_conditionContext.class);
		}
		public Role_with_conditionContext role_with_condition(int i) {
			return getRuleContext(Role_with_conditionContext.class,i);
		}
		public Roles_with_conditionsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_roles_with_conditions; }
	}

	public final Roles_with_conditionsContext roles_with_conditions() throws RecognitionException {
		Roles_with_conditionsContext _localctx = new Roles_with_conditionsContext(_ctx, getState());
		enterRule(_localctx, 124, RULE_roles_with_conditions);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(888);
			match(ROLES);
			setState(889);
			match(OpenCurly);
			setState(893);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==MINUS) {
				{
				{
				setState(890);
				role_with_condition();
				}
				}
				setState(895);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(896);
			match(CloseCurly);
			setState(897);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Role_with_conditionContext extends ParserRuleContext {
		public TerminalNode MINUS() { return getToken(ExtentedHATPParser.MINUS, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public ConditionsContext conditions() {
			return getRuleContext(ConditionsContext.class,0);
		}
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public Capacites_listContext capacites_list() {
			return getRuleContext(Capacites_listContext.class,0);
		}
		public Attentes_listContext attentes_list() {
			return getRuleContext(Attentes_listContext.class,0);
		}
		public Role_with_conditionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_role_with_condition; }
	}

	public final Role_with_conditionContext role_with_condition() throws RecognitionException {
		Role_with_conditionContext _localctx = new Role_with_conditionContext(_ctx, getState());
		enterRule(_localctx, 126, RULE_role_with_condition);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(899);
			match(MINUS);
			setState(900);
			name();
			setState(911);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==OpenCurly) {
				{
				setState(901);
				match(OpenCurly);
				setState(902);
				conditions();
				setState(904);
				_errHandler.sync(this);
				_la = _input.LA(1);
				if (_la==CAPACITES) {
					{
					setState(903);
					capacites_list();
					}
				}

				setState(907);
				_errHandler.sync(this);
				_la = _input.LA(1);
				if (_la==ATTENTES) {
					{
					setState(906);
					attentes_list();
					}
				}

				setState(909);
				match(CloseCurly);
				}
			}

			setState(913);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Roles_with_attentesContext extends ParserRuleContext {
		public TerminalNode ROLES() { return getToken(ExtentedHATPParser.ROLES, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<Role_with_attenteContext> role_with_attente() {
			return getRuleContexts(Role_with_attenteContext.class);
		}
		public Role_with_attenteContext role_with_attente(int i) {
			return getRuleContext(Role_with_attenteContext.class,i);
		}
		public Roles_with_attentesContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_roles_with_attentes; }
	}

	public final Roles_with_attentesContext roles_with_attentes() throws RecognitionException {
		Roles_with_attentesContext _localctx = new Roles_with_attentesContext(_ctx, getState());
		enterRule(_localctx, 128, RULE_roles_with_attentes);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(915);
			match(ROLES);
			setState(916);
			match(OpenCurly);
			setState(920);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==MINUS) {
				{
				{
				setState(917);
				role_with_attente();
				}
				}
				setState(922);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(923);
			match(CloseCurly);
			setState(924);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Role_with_attenteContext extends ParserRuleContext {
		public TerminalNode MINUS() { return getToken(ExtentedHATPParser.MINUS, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public Capacites_listContext capacites_list() {
			return getRuleContext(Capacites_listContext.class,0);
		}
		public Attentes_listContext attentes_list() {
			return getRuleContext(Attentes_listContext.class,0);
		}
		public Role_with_attenteContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_role_with_attente; }
	}

	public final Role_with_attenteContext role_with_attente() throws RecognitionException {
		Role_with_attenteContext _localctx = new Role_with_attenteContext(_ctx, getState());
		enterRule(_localctx, 130, RULE_role_with_attente);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(926);
			match(MINUS);
			setState(927);
			name();
			setState(936);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==OpenCurly) {
				{
				setState(928);
				match(OpenCurly);
				setState(930);
				_errHandler.sync(this);
				_la = _input.LA(1);
				if (_la==CAPACITES) {
					{
					setState(929);
					capacites_list();
					}
				}

				setState(933);
				_errHandler.sync(this);
				_la = _input.LA(1);
				if (_la==ATTENTES) {
					{
					setState(932);
					attentes_list();
					}
				}

				setState(935);
				match(CloseCurly);
				}
			}

			setState(938);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Attentes_listContext extends ParserRuleContext {
		public TerminalNode ATTENTES() { return getToken(ExtentedHATPParser.ATTENTES, 0); }
		public TerminalNode COLON() { return getToken(ExtentedHATPParser.COLON, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public List<Attente_extendedContext> attente_extended() {
			return getRuleContexts(Attente_extendedContext.class);
		}
		public Attente_extendedContext attente_extended(int i) {
			return getRuleContext(Attente_extendedContext.class,i);
		}
		public Attentes_listContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attentes_list; }
	}

	public final Attentes_listContext attentes_list() throws RecognitionException {
		Attentes_listContext _localctx = new Attentes_listContext(_ctx, getState());
		enterRule(_localctx, 132, RULE_attentes_list);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(940);
			match(ATTENTES);
			setState(941);
			match(COLON);
			setState(942);
			match(OpenCurly);
			setState(946);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==ATTENTE) {
				{
				{
				setState(943);
				attente_extended();
				}
				}
				setState(948);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(949);
			match(CloseCurly);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Capacites_listContext extends ParserRuleContext {
		public TerminalNode CAPACITES() { return getToken(ExtentedHATPParser.CAPACITES, 0); }
		public TerminalNode COLON() { return getToken(ExtentedHATPParser.COLON, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<CapaciteContext> capacite() {
			return getRuleContexts(CapaciteContext.class);
		}
		public CapaciteContext capacite(int i) {
			return getRuleContext(CapaciteContext.class,i);
		}
		public Capacites_listContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_capacites_list; }
	}

	public final Capacites_listContext capacites_list() throws RecognitionException {
		Capacites_listContext _localctx = new Capacites_listContext(_ctx, getState());
		enterRule(_localctx, 134, RULE_capacites_list);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(951);
			match(CAPACITES);
			setState(952);
			match(COLON);
			setState(953);
			match(OpenCurly);
			setState(957);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==CAPACITE) {
				{
				{
				setState(954);
				capacite();
				}
				}
				setState(959);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(960);
			match(CloseCurly);
			setState(961);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class CapaciteContext extends ParserRuleContext {
		public TerminalNode CAPACITE() { return getToken(ExtentedHATPParser.CAPACITE, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<Can_satisfy_expectationsContext> can_satisfy_expectations() {
			return getRuleContexts(Can_satisfy_expectationsContext.class);
		}
		public Can_satisfy_expectationsContext can_satisfy_expectations(int i) {
			return getRuleContext(Can_satisfy_expectationsContext.class,i);
		}
		public List<Description_capaciteContext> description_capacite() {
			return getRuleContexts(Description_capaciteContext.class);
		}
		public Description_capaciteContext description_capacite(int i) {
			return getRuleContext(Description_capaciteContext.class,i);
		}
		public List<ConditionsContext> conditions() {
			return getRuleContexts(ConditionsContext.class);
		}
		public ConditionsContext conditions(int i) {
			return getRuleContext(ConditionsContext.class,i);
		}
		public CapaciteContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_capacite; }
	}

	public final CapaciteContext capacite() throws RecognitionException {
		CapaciteContext _localctx = new CapaciteContext(_ctx, getState());
		enterRule(_localctx, 136, RULE_capacite);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(963);
			match(CAPACITE);
			setState(964);
			name();
			setState(965);
			match(OpenCurly);
			setState(971);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==DESCRIPTION || _la==CONDITIONS || _la==CAN_SATISFY_EXPECTATIONS) {
				{
				setState(969);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case CAN_SATISFY_EXPECTATIONS:
					{
					setState(966);
					can_satisfy_expectations();
					}
					break;
				case DESCRIPTION:
					{
					setState(967);
					description_capacite();
					}
					break;
				case CONDITIONS:
					{
					setState(968);
					conditions();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(973);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(974);
			match(CloseCurly);
			setState(975);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Can_satisfy_expectationsContext extends ParserRuleContext {
		public TerminalNode CAN_SATISFY_EXPECTATIONS() { return getToken(ExtentedHATPParser.CAN_SATISFY_EXPECTATIONS, 0); }
		public TerminalNode COLON() { return getToken(ExtentedHATPParser.COLON, 0); }
		public TerminalNode OpenSquare() { return getToken(ExtentedHATPParser.OpenSquare, 0); }
		public Expectation_type_listContext expectation_type_list() {
			return getRuleContext(Expectation_type_listContext.class,0);
		}
		public TerminalNode CloseSquare() { return getToken(ExtentedHATPParser.CloseSquare, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public Can_satisfy_expectationsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_can_satisfy_expectations; }
	}

	public final Can_satisfy_expectationsContext can_satisfy_expectations() throws RecognitionException {
		Can_satisfy_expectationsContext _localctx = new Can_satisfy_expectationsContext(_ctx, getState());
		enterRule(_localctx, 138, RULE_can_satisfy_expectations);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(977);
			match(CAN_SATISFY_EXPECTATIONS);
			setState(978);
			match(COLON);
			setState(979);
			match(OpenSquare);
			setState(980);
			expectation_type_list();
			setState(981);
			match(CloseSquare);
			setState(982);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Expectation_type_listContext extends ParserRuleContext {
		public List<NameContext> name() {
			return getRuleContexts(NameContext.class);
		}
		public NameContext name(int i) {
			return getRuleContext(NameContext.class,i);
		}
		public List<TerminalNode> Comma() { return getTokens(ExtentedHATPParser.Comma); }
		public TerminalNode Comma(int i) {
			return getToken(ExtentedHATPParser.Comma, i);
		}
		public Expectation_type_listContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_expectation_type_list; }
	}

	public final Expectation_type_listContext expectation_type_list() throws RecognitionException {
		Expectation_type_listContext _localctx = new Expectation_type_listContext(_ctx, getState());
		enterRule(_localctx, 140, RULE_expectation_type_list);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(984);
			name();
			setState(989);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==Comma) {
				{
				{
				setState(985);
				match(Comma);
				setState(986);
				name();
				}
				}
				setState(991);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Description_capaciteContext extends ParserRuleContext {
		public TerminalNode DESCRIPTION() { return getToken(ExtentedHATPParser.DESCRIPTION, 0); }
		public TerminalNode COLON() { return getToken(ExtentedHATPParser.COLON, 0); }
		public List<TerminalNode> STRING() { return getTokens(ExtentedHATPParser.STRING); }
		public TerminalNode STRING(int i) {
			return getToken(ExtentedHATPParser.STRING, i);
		}
		public SentenceContext sentence() {
			return getRuleContext(SentenceContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public Description_capaciteContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_description_capacite; }
	}

	public final Description_capaciteContext description_capacite() throws RecognitionException {
		Description_capaciteContext _localctx = new Description_capaciteContext(_ctx, getState());
		enterRule(_localctx, 142, RULE_description_capacite);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(992);
			match(DESCRIPTION);
			setState(993);
			match(COLON);
			setState(994);
			match(STRING);
			setState(995);
			sentence();
			setState(996);
			match(STRING);
			setState(997);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Attente_extendedContext extends ParserRuleContext {
		public TerminalNode ATTENTE() { return getToken(ExtentedHATPParser.ATTENTE, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<Attente_typeContext> attente_type() {
			return getRuleContexts(Attente_typeContext.class);
		}
		public Attente_typeContext attente_type(int i) {
			return getRuleContext(Attente_typeContext.class,i);
		}
		public List<Expects_fromContext> expects_from() {
			return getRuleContexts(Expects_fromContext.class);
		}
		public Expects_fromContext expects_from(int i) {
			return getRuleContext(Expects_fromContext.class,i);
		}
		public List<Description_attenteContext> description_attente() {
			return getRuleContexts(Description_attenteContext.class);
		}
		public Description_attenteContext description_attente(int i) {
			return getRuleContext(Description_attenteContext.class,i);
		}
		public List<ConditionsContext> conditions() {
			return getRuleContexts(ConditionsContext.class);
		}
		public ConditionsContext conditions(int i) {
			return getRuleContext(ConditionsContext.class,i);
		}
		public Attente_extendedContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attente_extended; }
	}

	public final Attente_extendedContext attente_extended() throws RecognitionException {
		Attente_extendedContext _localctx = new Attente_extendedContext(_ctx, getState());
		enterRule(_localctx, 144, RULE_attente_extended);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(999);
			match(ATTENTE);
			setState(1000);
			name();
			setState(1001);
			match(OpenCurly);
			setState(1008);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==DESCRIPTION || _la==CONDITIONS || _la==TYPE_KW || _la==EXPECTS_FROM) {
				{
				setState(1006);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case TYPE_KW:
					{
					setState(1002);
					attente_type();
					}
					break;
				case EXPECTS_FROM:
					{
					setState(1003);
					expects_from();
					}
					break;
				case DESCRIPTION:
					{
					setState(1004);
					description_attente();
					}
					break;
				case CONDITIONS:
					{
					setState(1005);
					conditions();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(1010);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(1011);
			match(CloseCurly);
			setState(1012);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Attente_typeContext extends ParserRuleContext {
		public TerminalNode TYPE_KW() { return getToken(ExtentedHATPParser.TYPE_KW, 0); }
		public TerminalNode COLON() { return getToken(ExtentedHATPParser.COLON, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public Attente_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attente_type; }
	}

	public final Attente_typeContext attente_type() throws RecognitionException {
		Attente_typeContext _localctx = new Attente_typeContext(_ctx, getState());
		enterRule(_localctx, 146, RULE_attente_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1014);
			match(TYPE_KW);
			setState(1015);
			match(COLON);
			setState(1016);
			name();
			setState(1017);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Expects_fromContext extends ParserRuleContext {
		public TerminalNode EXPECTS_FROM() { return getToken(ExtentedHATPParser.EXPECTS_FROM, 0); }
		public TerminalNode COLON() { return getToken(ExtentedHATPParser.COLON, 0); }
		public TerminalNode OpenSquare() { return getToken(ExtentedHATPParser.OpenSquare, 0); }
		public Role_listContext role_list() {
			return getRuleContext(Role_listContext.class,0);
		}
		public TerminalNode CloseSquare() { return getToken(ExtentedHATPParser.CloseSquare, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public Expects_fromContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_expects_from; }
	}

	public final Expects_fromContext expects_from() throws RecognitionException {
		Expects_fromContext _localctx = new Expects_fromContext(_ctx, getState());
		enterRule(_localctx, 148, RULE_expects_from);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1019);
			match(EXPECTS_FROM);
			setState(1020);
			match(COLON);
			setState(1021);
			match(OpenSquare);
			setState(1022);
			role_list();
			setState(1023);
			match(CloseSquare);
			setState(1024);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Role_listContext extends ParserRuleContext {
		public List<NameContext> name() {
			return getRuleContexts(NameContext.class);
		}
		public NameContext name(int i) {
			return getRuleContext(NameContext.class,i);
		}
		public List<TerminalNode> Comma() { return getTokens(ExtentedHATPParser.Comma); }
		public TerminalNode Comma(int i) {
			return getToken(ExtentedHATPParser.Comma, i);
		}
		public Role_listContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_role_list; }
	}

	public final Role_listContext role_list() throws RecognitionException {
		Role_listContext _localctx = new Role_listContext(_ctx, getState());
		enterRule(_localctx, 150, RULE_role_list);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1026);
			name();
			setState(1031);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==Comma) {
				{
				{
				setState(1027);
				match(Comma);
				setState(1028);
				name();
				}
				}
				setState(1033);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Description_attenteContext extends ParserRuleContext {
		public TerminalNode DESCRIPTION() { return getToken(ExtentedHATPParser.DESCRIPTION, 0); }
		public TerminalNode COLON() { return getToken(ExtentedHATPParser.COLON, 0); }
		public List<TerminalNode> STRING() { return getTokens(ExtentedHATPParser.STRING); }
		public TerminalNode STRING(int i) {
			return getToken(ExtentedHATPParser.STRING, i);
		}
		public SentenceContext sentence() {
			return getRuleContext(SentenceContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public Description_attenteContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_description_attente; }
	}

	public final Description_attenteContext description_attente() throws RecognitionException {
		Description_attenteContext _localctx = new Description_attenteContext(_ctx, getState());
		enterRule(_localctx, 152, RULE_description_attente);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1034);
			match(DESCRIPTION);
			setState(1035);
			match(COLON);
			setState(1036);
			match(STRING);
			setState(1037);
			sentence();
			setState(1038);
			match(STRING);
			setState(1039);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Practices_blocContext extends ParserRuleContext {
		public TerminalNode PRACTICES() { return getToken(ExtentedHATPParser.PRACTICES, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<PracticeContext> practice() {
			return getRuleContexts(PracticeContext.class);
		}
		public PracticeContext practice(int i) {
			return getRuleContext(PracticeContext.class,i);
		}
		public Practices_blocContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_practices_bloc; }
	}

	public final Practices_blocContext practices_bloc() throws RecognitionException {
		Practices_blocContext _localctx = new Practices_blocContext(_ctx, getState());
		enterRule(_localctx, 154, RULE_practices_bloc);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1041);
			match(PRACTICES);
			setState(1042);
			match(OpenCurly);
			setState(1046);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==PRACTICE) {
				{
				{
				setState(1043);
				practice();
				}
				}
				setState(1048);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(1049);
			match(CloseCurly);
			setState(1050);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class PracticeContext extends ParserRuleContext {
		public TerminalNode PRACTICE() { return getToken(ExtentedHATPParser.PRACTICE, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<Description_practiceContext> description_practice() {
			return getRuleContexts(Description_practiceContext.class);
		}
		public Description_practiceContext description_practice(int i) {
			return getRuleContext(Description_practiceContext.class,i);
		}
		public List<Conditions_practicesContext> conditions_practices() {
			return getRuleContexts(Conditions_practicesContext.class);
		}
		public Conditions_practicesContext conditions_practices(int i) {
			return getRuleContext(Conditions_practicesContext.class,i);
		}
		public List<Roles_with_attentesContext> roles_with_attentes() {
			return getRuleContexts(Roles_with_attentesContext.class);
		}
		public Roles_with_attentesContext roles_with_attentes(int i) {
			return getRuleContext(Roles_with_attentesContext.class,i);
		}
		public List<CompetencesContext> competences() {
			return getRuleContexts(CompetencesContext.class);
		}
		public CompetencesContext competences(int i) {
			return getRuleContext(CompetencesContext.class,i);
		}
		public List<Objects_blocContext> objects_bloc() {
			return getRuleContexts(Objects_blocContext.class);
		}
		public Objects_blocContext objects_bloc(int i) {
			return getRuleContext(Objects_blocContext.class,i);
		}
		public List<Rules_blocContext> rules_bloc() {
			return getRuleContexts(Rules_blocContext.class);
		}
		public Rules_blocContext rules_bloc(int i) {
			return getRuleContext(Rules_blocContext.class,i);
		}
		public PracticeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_practice; }
	}

	public final PracticeContext practice() throws RecognitionException {
		PracticeContext _localctx = new PracticeContext(_ctx, getState());
		enterRule(_localctx, 156, RULE_practice);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1052);
			match(PRACTICE);
			setState(1053);
			name();
			setState(1054);
			match(OpenCurly);
			setState(1063);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==DESCRIPTION || _la==CONDITIONS || ((((_la - 74)) & ~0x3f) == 0 && ((1L << (_la - 74)) & 4103L) != 0)) {
				{
				setState(1061);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case DESCRIPTION:
					{
					setState(1055);
					description_practice();
					}
					break;
				case CONDITIONS:
					{
					setState(1056);
					conditions_practices();
					}
					break;
				case ROLES:
					{
					setState(1057);
					roles_with_attentes();
					}
					break;
				case COMPETENCES:
					{
					setState(1058);
					competences();
					}
					break;
				case OBJECTS:
					{
					setState(1059);
					objects_bloc();
					}
					break;
				case RULES:
					{
					setState(1060);
					rules_bloc();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(1065);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(1066);
			match(CloseCurly);
			setState(1067);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Conditions_practicesContext extends ParserRuleContext {
		public TerminalNode CONDITIONS() { return getToken(ExtentedHATPParser.CONDITIONS, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public QueryContext query() {
			return getRuleContext(QueryContext.class,0);
		}
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public Conditions_practicesContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_conditions_practices; }
	}

	public final Conditions_practicesContext conditions_practices() throws RecognitionException {
		Conditions_practicesContext _localctx = new Conditions_practicesContext(_ctx, getState());
		enterRule(_localctx, 158, RULE_conditions_practices);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1069);
			match(CONDITIONS);
			setState(1070);
			match(OpenCurly);
			setState(1071);
			query();
			setState(1072);
			match(CloseCurly);
			setState(1073);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class CompetencesContext extends ParserRuleContext {
		public TerminalNode COMPETENCES() { return getToken(ExtentedHATPParser.COMPETENCES, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public List<CompetenceContext> competence() {
			return getRuleContexts(CompetenceContext.class);
		}
		public CompetenceContext competence(int i) {
			return getRuleContext(CompetenceContext.class,i);
		}
		public CompetencesContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_competences; }
	}

	public final CompetencesContext competences() throws RecognitionException {
		CompetencesContext _localctx = new CompetencesContext(_ctx, getState());
		enterRule(_localctx, 160, RULE_competences);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1075);
			match(COMPETENCES);
			setState(1076);
			match(OpenCurly);
			setState(1080);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==MINUS) {
				{
				{
				setState(1077);
				competence();
				}
				}
				setState(1082);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(1083);
			match(CloseCurly);
			setState(1084);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class CompetenceContext extends ParserRuleContext {
		public TerminalNode MINUS() { return getToken(ExtentedHATPParser.MINUS, 0); }
		public List<TerminalNode> STRING() { return getTokens(ExtentedHATPParser.STRING); }
		public TerminalNode STRING(int i) {
			return getToken(ExtentedHATPParser.STRING, i);
		}
		public SentenceContext sentence() {
			return getRuleContext(SentenceContext.class,0);
		}
		public CompetenceContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_competence; }
	}

	public final CompetenceContext competence() throws RecognitionException {
		CompetenceContext _localctx = new CompetenceContext(_ctx, getState());
		enterRule(_localctx, 162, RULE_competence);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1086);
			match(MINUS);
			setState(1087);
			match(STRING);
			setState(1088);
			sentence();
			setState(1089);
			match(STRING);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Id_methodContext extends ParserRuleContext {
		public TerminalNode IDENTIFIER() { return getToken(ExtentedHATPParser.IDENTIFIER, 0); }
		public TerminalNode NUMBER() { return getToken(ExtentedHATPParser.NUMBER, 0); }
		public Id_methodContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_id_method; }
	}

	public final Id_methodContext id_method() throws RecognitionException {
		Id_methodContext _localctx = new Id_methodContext(_ctx, getState());
		enterRule(_localctx, 164, RULE_id_method);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1091);
			_la = _input.LA(1);
			if ( !(_la==IDENTIFIER || _la==NUMBER) ) {
			_errHandler.recoverInline(this);
			}
			else {
				if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
				_errHandler.reportMatch(this);
				consume();
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class IdContext extends ParserRuleContext {
		public TerminalNode NUMBER() { return getToken(ExtentedHATPParser.NUMBER, 0); }
		public IdContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_id; }
	}

	public final IdContext id() throws RecognitionException {
		IdContext _localctx = new IdContext(_ctx, getState());
		enterRule(_localctx, 166, RULE_id);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1093);
			match(NUMBER);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Numeric_valueContext extends ParserRuleContext {
		public TerminalNode NUMBER() { return getToken(ExtentedHATPParser.NUMBER, 0); }
		public Numeric_valueContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_numeric_value; }
	}

	public final Numeric_valueContext numeric_value() throws RecognitionException {
		Numeric_valueContext _localctx = new Numeric_valueContext(_ctx, getState());
		enterRule(_localctx, 168, RULE_numeric_value);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1095);
			match(NUMBER);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class SubjectContext extends ParserRuleContext {
		public VariableContext variable() {
			return getRuleContext(VariableContext.class,0);
		}
		public LiteralContext literal() {
			return getRuleContext(LiteralContext.class,0);
		}
		public My_self_varContext my_self_var() {
			return getRuleContext(My_self_varContext.class,0);
		}
		public SubjectContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_subject; }
	}

	public final SubjectContext subject() throws RecognitionException {
		SubjectContext _localctx = new SubjectContext(_ctx, getState());
		enterRule(_localctx, 170, RULE_subject);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1100);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,100,_ctx) ) {
			case 1:
				{
				setState(1097);
				variable();
				}
				break;
			case 2:
				{
				setState(1098);
				literal();
				}
				break;
			case 3:
				{
				setState(1099);
				my_self_var();
				}
				break;
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class VariableContext extends ParserRuleContext {
		public TerminalNode QUESTIONMARK() { return getToken(ExtentedHATPParser.QUESTIONMARK, 0); }
		public TerminalNode IDENTIFIER() { return getToken(ExtentedHATPParser.IDENTIFIER, 0); }
		public VariableContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_variable; }
	}

	public final VariableContext variable() throws RecognitionException {
		VariableContext _localctx = new VariableContext(_ctx, getState());
		enterRule(_localctx, 172, RULE_variable);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1102);
			match(QUESTIONMARK);
			setState(1103);
			match(IDENTIFIER);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class LiteralContext extends ParserRuleContext {
		public TerminalNode IDENTIFIER() { return getToken(ExtentedHATPParser.IDENTIFIER, 0); }
		public LiteralContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_literal; }
	}

	public final LiteralContext literal() throws RecognitionException {
		LiteralContext _localctx = new LiteralContext(_ctx, getState());
		enterRule(_localctx, 174, RULE_literal);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1105);
			match(IDENTIFIER);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class My_self_varContext extends ParserRuleContext {
		public List<TerminalNode> QUESTIONMARK() { return getTokens(ExtentedHATPParser.QUESTIONMARK); }
		public TerminalNode QUESTIONMARK(int i) {
			return getToken(ExtentedHATPParser.QUESTIONMARK, i);
		}
		public My_self_varContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_my_self_var; }
	}

	public final My_self_varContext my_self_var() throws RecognitionException {
		My_self_varContext _localctx = new My_self_varContext(_ctx, getState());
		enterRule(_localctx, 176, RULE_my_self_var);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1107);
			match(QUESTIONMARK);
			setState(1108);
			match(QUESTIONMARK);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class PredicateContext extends ParserRuleContext {
		public TerminalNode IDENTIFIER() { return getToken(ExtentedHATPParser.IDENTIFIER, 0); }
		public PredicateContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_predicate; }
	}

	public final PredicateContext predicate() throws RecognitionException {
		PredicateContext _localctx = new PredicateContext(_ctx, getState());
		enterRule(_localctx, 178, RULE_predicate);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1110);
			match(IDENTIFIER);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class ObjectContext extends ParserRuleContext {
		public VariableContext variable() {
			return getRuleContext(VariableContext.class,0);
		}
		public LiteralContext literal() {
			return getRuleContext(LiteralContext.class,0);
		}
		public My_self_varContext my_self_var() {
			return getRuleContext(My_self_varContext.class,0);
		}
		public ObjectContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_object; }
	}

	public final ObjectContext object() throws RecognitionException {
		ObjectContext _localctx = new ObjectContext(_ctx, getState());
		enterRule(_localctx, 180, RULE_object);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1115);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,101,_ctx) ) {
			case 1:
				{
				setState(1112);
				variable();
				}
				break;
			case 2:
				{
				setState(1113);
				literal();
				}
				break;
			case 3:
				{
				setState(1114);
				my_self_var();
				}
				break;
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class ArgumentsContext extends ParserRuleContext {
		public TypeContext type() {
			return getRuleContext(TypeContext.class,0);
		}
		public VarnameContext varname() {
			return getRuleContext(VarnameContext.class,0);
		}
		public ArgumentsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_arguments; }
	}

	public final ArgumentsContext arguments() throws RecognitionException {
		ArgumentsContext _localctx = new ArgumentsContext(_ctx, getState());
		enterRule(_localctx, 182, RULE_arguments);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1117);
			type();
			setState(1118);
			varname();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class TypeContext extends ParserRuleContext {
		public TerminalNode IDENTIFIER() { return getToken(ExtentedHATPParser.IDENTIFIER, 0); }
		public TypeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_type; }
	}

	public final TypeContext type() throws RecognitionException {
		TypeContext _localctx = new TypeContext(_ctx, getState());
		enterRule(_localctx, 184, RULE_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1120);
			match(IDENTIFIER);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class VarnameContext extends ParserRuleContext {
		public TerminalNode IDENTIFIER() { return getToken(ExtentedHATPParser.IDENTIFIER, 0); }
		public VarnameContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_varname; }
	}

	public final VarnameContext varname() throws RecognitionException {
		VarnameContext _localctx = new VarnameContext(_ctx, getState());
		enterRule(_localctx, 186, RULE_varname);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1122);
			match(IDENTIFIER);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class CommentContext extends ParserRuleContext {
		public TerminalNode COMMENT() { return getToken(ExtentedHATPParser.COMMENT, 0); }
		public TerminalNode LINE_COMMENT() { return getToken(ExtentedHATPParser.LINE_COMMENT, 0); }
		public CommentContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_comment; }
	}

	public final CommentContext comment() throws RecognitionException {
		CommentContext _localctx = new CommentContext(_ctx, getState());
		enterRule(_localctx, 188, RULE_comment);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1124);
			_la = _input.LA(1);
			if ( !(_la==COMMENT || _la==LINE_COMMENT) ) {
			_errHandler.recoverInline(this);
			}
			else {
				if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
				_errHandler.reportMatch(this);
				consume();
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class IgnoreContext extends ParserRuleContext {
		public TerminalNode SEMICOLON() { return getToken(ExtentedHATPParser.SEMICOLON, 0); }
		public IgnoreContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_ignore; }
	}

	public final IgnoreContext ignore() throws RecognitionException {
		IgnoreContext _localctx = new IgnoreContext(_ctx, getState());
		enterRule(_localctx, 190, RULE_ignore);
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(1129);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,102,_ctx);
			while ( _alt!=1 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1+1 ) {
					{
					{
					setState(1126);
					matchWildcard();
					}
					} 
				}
				setState(1131);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,102,_ctx);
			}
			setState(1132);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class OperatorContext extends ParserRuleContext {
		public TerminalNode ADD_IN_SET() { return getToken(ExtentedHATPParser.ADD_IN_SET, 0); }
		public TerminalNode REMOVE_FROM_SET() { return getToken(ExtentedHATPParser.REMOVE_FROM_SET, 0); }
		public TerminalNode EQUAL() { return getToken(ExtentedHATPParser.EQUAL, 0); }
		public TerminalNode TEST_EQUAL() { return getToken(ExtentedHATPParser.TEST_EQUAL, 0); }
		public TerminalNode TEST_DIFF() { return getToken(ExtentedHATPParser.TEST_DIFF, 0); }
		public TerminalNode TEST_SET_IN() { return getToken(ExtentedHATPParser.TEST_SET_IN, 0); }
		public TerminalNode TEST_SET_NOT_IN() { return getToken(ExtentedHATPParser.TEST_SET_NOT_IN, 0); }
		public TerminalNode SUP() { return getToken(ExtentedHATPParser.SUP, 0); }
		public TerminalNode SUP_EQUAL() { return getToken(ExtentedHATPParser.SUP_EQUAL, 0); }
		public TerminalNode INF() { return getToken(ExtentedHATPParser.INF, 0); }
		public TerminalNode INF_EQUAL() { return getToken(ExtentedHATPParser.INF_EQUAL, 0); }
		public TerminalNode SUP_TILD() { return getToken(ExtentedHATPParser.SUP_TILD, 0); }
		public OperatorContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_operator; }
	}

	public final OperatorContext operator() throws RecognitionException {
		OperatorContext _localctx = new OperatorContext(_ctx, getState());
		enterRule(_localctx, 192, RULE_operator);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1134);
			_la = _input.LA(1);
			if ( !((((_la) & ~0x3f) == 0 && ((1L << _la) & 281406257233920L) != 0)) ) {
			_errHandler.recoverInline(this);
			}
			else {
				if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
				_errHandler.reportMatch(this);
				consume();
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class SentenceContext extends ParserRuleContext {
		public List<WordContext> word() {
			return getRuleContexts(WordContext.class);
		}
		public WordContext word(int i) {
			return getRuleContext(WordContext.class,i);
		}
		public List<TerminalNode> SPACE() { return getTokens(ExtentedHATPParser.SPACE); }
		public TerminalNode SPACE(int i) {
			return getToken(ExtentedHATPParser.SPACE, i);
		}
		public SentenceContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_sentence; }
	}

	public final SentenceContext sentence() throws RecognitionException {
		SentenceContext _localctx = new SentenceContext(_ctx, getState());
		enterRule(_localctx, 194, RULE_sentence);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1140);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (((((_la - 48)) & ~0x3f) == 0 && ((1L << (_la - 48)) & 6515951L) != 0)) {
				{
				setState(1138);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case PLUS:
				case MINUS:
				case TIMES:
				case SLASH:
				case POINT:
				case COLON:
				case QUESTIONMARK:
				case OpenPar:
				case ClosePar:
				case OpenCurly:
				case CloseCurly:
				case Comma:
				case IDENTIFIER:
				case NUMBER:
					{
					setState(1136);
					word();
					}
					break;
				case SPACE:
					{
					setState(1137);
					match(SPACE);
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(1142);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class WordContext extends ParserRuleContext {
		public TerminalNode IDENTIFIER() { return getToken(ExtentedHATPParser.IDENTIFIER, 0); }
		public TerminalNode Comma() { return getToken(ExtentedHATPParser.Comma, 0); }
		public TerminalNode POINT() { return getToken(ExtentedHATPParser.POINT, 0); }
		public TerminalNode COLON() { return getToken(ExtentedHATPParser.COLON, 0); }
		public TerminalNode QUESTIONMARK() { return getToken(ExtentedHATPParser.QUESTIONMARK, 0); }
		public TerminalNode NUMBER() { return getToken(ExtentedHATPParser.NUMBER, 0); }
		public TerminalNode SLASH() { return getToken(ExtentedHATPParser.SLASH, 0); }
		public TerminalNode PLUS() { return getToken(ExtentedHATPParser.PLUS, 0); }
		public TerminalNode MINUS() { return getToken(ExtentedHATPParser.MINUS, 0); }
		public TerminalNode TIMES() { return getToken(ExtentedHATPParser.TIMES, 0); }
		public TerminalNode OpenPar() { return getToken(ExtentedHATPParser.OpenPar, 0); }
		public TerminalNode ClosePar() { return getToken(ExtentedHATPParser.ClosePar, 0); }
		public TerminalNode OpenCurly() { return getToken(ExtentedHATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(ExtentedHATPParser.CloseCurly, 0); }
		public WordContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_word; }
	}

	public final WordContext word() throws RecognitionException {
		WordContext _localctx = new WordContext(_ctx, getState());
		enterRule(_localctx, 196, RULE_word);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1143);
			_la = _input.LA(1);
			if ( !(((((_la - 48)) & ~0x3f) == 0 && ((1L << (_la - 48)) & 2321647L) != 0)) ) {
			_errHandler.recoverInline(this);
			}
			else {
				if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
				_errHandler.reportMatch(this);
				consume();
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static final String _serializedATN =
		"\u0004\u0001V\u047a\u0002\u0000\u0007\u0000\u0002\u0001\u0007\u0001\u0002"+
		"\u0002\u0007\u0002\u0002\u0003\u0007\u0003\u0002\u0004\u0007\u0004\u0002"+
		"\u0005\u0007\u0005\u0002\u0006\u0007\u0006\u0002\u0007\u0007\u0007\u0002"+
		"\b\u0007\b\u0002\t\u0007\t\u0002\n\u0007\n\u0002\u000b\u0007\u000b\u0002"+
		"\f\u0007\f\u0002\r\u0007\r\u0002\u000e\u0007\u000e\u0002\u000f\u0007\u000f"+
		"\u0002\u0010\u0007\u0010\u0002\u0011\u0007\u0011\u0002\u0012\u0007\u0012"+
		"\u0002\u0013\u0007\u0013\u0002\u0014\u0007\u0014\u0002\u0015\u0007\u0015"+
		"\u0002\u0016\u0007\u0016\u0002\u0017\u0007\u0017\u0002\u0018\u0007\u0018"+
		"\u0002\u0019\u0007\u0019\u0002\u001a\u0007\u001a\u0002\u001b\u0007\u001b"+
		"\u0002\u001c\u0007\u001c\u0002\u001d\u0007\u001d\u0002\u001e\u0007\u001e"+
		"\u0002\u001f\u0007\u001f\u0002 \u0007 \u0002!\u0007!\u0002\"\u0007\"\u0002"+
		"#\u0007#\u0002$\u0007$\u0002%\u0007%\u0002&\u0007&\u0002\'\u0007\'\u0002"+
		"(\u0007(\u0002)\u0007)\u0002*\u0007*\u0002+\u0007+\u0002,\u0007,\u0002"+
		"-\u0007-\u0002.\u0007.\u0002/\u0007/\u00020\u00070\u00021\u00071\u0002"+
		"2\u00072\u00023\u00073\u00024\u00074\u00025\u00075\u00026\u00076\u0002"+
		"7\u00077\u00028\u00078\u00029\u00079\u0002:\u0007:\u0002;\u0007;\u0002"+
		"<\u0007<\u0002=\u0007=\u0002>\u0007>\u0002?\u0007?\u0002@\u0007@\u0002"+
		"A\u0007A\u0002B\u0007B\u0002C\u0007C\u0002D\u0007D\u0002E\u0007E\u0002"+
		"F\u0007F\u0002G\u0007G\u0002H\u0007H\u0002I\u0007I\u0002J\u0007J\u0002"+
		"K\u0007K\u0002L\u0007L\u0002M\u0007M\u0002N\u0007N\u0002O\u0007O\u0002"+
		"P\u0007P\u0002Q\u0007Q\u0002R\u0007R\u0002S\u0007S\u0002T\u0007T\u0002"+
		"U\u0007U\u0002V\u0007V\u0002W\u0007W\u0002X\u0007X\u0002Y\u0007Y\u0002"+
		"Z\u0007Z\u0002[\u0007[\u0002\\\u0007\\\u0002]\u0007]\u0002^\u0007^\u0002"+
		"_\u0007_\u0002`\u0007`\u0002a\u0007a\u0002b\u0007b\u0001\u0000\u0001\u0000"+
		"\u0001\u0000\u0001\u0000\u0001\u0000\u0001\u0000\u0001\u0000\u0001\u0000"+
		"\u0004\u0000\u00cf\b\u0000\u000b\u0000\f\u0000\u00d0\u0001\u0000\u0001"+
		"\u0000\u0001\u0001\u0001\u0001\u0001\u0001\u0005\u0001\u00d8\b\u0001\n"+
		"\u0001\f\u0001\u00db\t\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001"+
		"\u0002\u0001\u0002\u0001\u0002\u0003\u0002\u00e3\b\u0002\u0001\u0002\u0001"+
		"\u0002\u0001\u0002\u0001\u0003\u0001\u0003\u0001\u0003\u0001\u0003\u0001"+
		"\u0003\u0001\u0004\u0003\u0004\u00ee\b\u0004\u0001\u0004\u0001\u0004\u0001"+
		"\u0004\u0005\u0004\u00f3\b\u0004\n\u0004\f\u0004\u00f6\t\u0004\u0001\u0004"+
		"\u0001\u0004\u0001\u0004\u0001\u0005\u0001\u0005\u0001\u0005\u0005\u0005"+
		"\u00fe\b\u0005\n\u0005\f\u0005\u0101\t\u0005\u0001\u0005\u0001\u0005\u0001"+
		"\u0005\u0001\u0006\u0001\u0006\u0001\u0006\u0001\u0006\u0005\u0006\u010a"+
		"\b\u0006\n\u0006\f\u0006\u010d\t\u0006\u0001\u0006\u0001\u0006\u0005\u0006"+
		"\u0111\b\u0006\n\u0006\f\u0006\u0114\t\u0006\u0001\u0006\u0001\u0006\u0001"+
		"\u0006\u0003\u0006\u0119\b\u0006\u0001\u0006\u0003\u0006\u011c\b\u0006"+
		"\u0001\u0006\u0001\u0006\u0001\u0006\u0001\u0006\u0001\u0006\u0004\u0006"+
		"\u0123\b\u0006\u000b\u0006\f\u0006\u0124\u0001\u0006\u0001\u0006\u0001"+
		"\u0006\u0001\u0007\u0001\u0007\u0001\u0007\u0001\u0007\u0005\u0007\u012e"+
		"\b\u0007\n\u0007\f\u0007\u0131\t\u0007\u0001\u0007\u0001\u0007\u0001\u0007"+
		"\u0001\b\u0001\b\u0001\b\u0001\b\u0003\b\u013a\b\b\u0001\b\u0001\b\u0005"+
		"\b\u013e\b\b\n\b\f\b\u0141\t\b\u0001\b\u0001\b\u0001\b\u0001\b\u0001\b"+
		"\u0001\t\u0004\t\u0149\b\t\u000b\t\f\t\u014a\u0001\n\u0001\n\u0001\n\u0004"+
		"\n\u0150\b\n\u000b\n\f\n\u0151\u0001\n\u0001\n\u0001\n\u0001\n\u0003\n"+
		"\u0158\b\n\u0001\u000b\u0001\u000b\u0001\u000b\u0005\u000b\u015d\b\u000b"+
		"\n\u000b\f\u000b\u0160\t\u000b\u0001\f\u0001\f\u0001\f\u0001\f\u0001\f"+
		"\u0001\f\u0001\r\u0003\r\u0169\b\r\u0001\r\u0001\r\u0001\r\u0001\r\u0001"+
		"\r\u0001\r\u0001\r\u0001\u000e\u0001\u000e\u0001\u000f\u0001\u000f\u0001"+
		"\u000f\u0005\u000f\u0177\b\u000f\n\u000f\f\u000f\u017a\t\u000f\u0001\u000f"+
		"\u0001\u000f\u0001\u000f\u0001\u0010\u0001\u0010\u0001\u0010\u0001\u0010"+
		"\u0001\u0010\u0001\u0010\u0001\u0010\u0001\u0011\u0001\u0011\u0001\u0011"+
		"\u0001\u0011\u0001\u0011\u0001\u0011\u0001\u0011\u0001\u0012\u0001\u0012"+
		"\u0001\u0012\u0005\u0012\u0190\b\u0012\n\u0012\f\u0012\u0193\t\u0012\u0001"+
		"\u0012\u0001\u0012\u0001\u0012\u0001\u0013\u0003\u0013\u0199\b\u0013\u0001"+
		"\u0013\u0001\u0013\u0001\u0013\u0001\u0013\u0003\u0013\u019f\b\u0013\u0001"+
		"\u0013\u0001\u0013\u0001\u0014\u0001\u0014\u0001\u0014\u0005\u0014\u01a6"+
		"\b\u0014\n\u0014\f\u0014\u01a9\t\u0014\u0001\u0014\u0001\u0014\u0001\u0014"+
		"\u0001\u0015\u0001\u0015\u0001\u0015\u0001\u0015\u0001\u0015\u0005\u0015"+
		"\u01b3\b\u0015\n\u0015\f\u0015\u01b6\t\u0015\u0001\u0015\u0001\u0015\u0001"+
		"\u0015\u0001\u0015\u0001\u0015\u0001\u0015\u0003\u0015\u01be\b\u0015\u0001"+
		"\u0016\u0001\u0016\u0001\u0016\u0001\u0016\u0003\u0016\u01c4\b\u0016\u0001"+
		"\u0017\u0001\u0017\u0001\u0018\u0001\u0018\u0001\u0019\u0003\u0019\u01cb"+
		"\b\u0019\u0001\u0019\u0001\u0019\u0001\u0019\u0001\u0019\u0005\u0019\u01d1"+
		"\b\u0019\n\u0019\f\u0019\u01d4\t\u0019\u0001\u0019\u0003\u0019\u01d7\b"+
		"\u0019\u0001\u001a\u0001\u001a\u0005\u001a\u01db\b\u001a\n\u001a\f\u001a"+
		"\u01de\t\u001a\u0001\u001a\u0001\u001a\u0005\u001a\u01e2\b\u001a\n\u001a"+
		"\f\u001a\u01e5\t\u001a\u0001\u001a\u0001\u001a\u0001\u001b\u0003\u001b"+
		"\u01ea\b\u001b\u0001\u001b\u0001\u001b\u0003\u001b\u01ee\b\u001b\u0001"+
		"\u001b\u0001\u001b\u0003\u001b\u01f2\b\u001b\u0001\u001b\u0001\u001b\u0003"+
		"\u001b\u01f6\b\u001b\u0001\u001c\u0001\u001c\u0001\u001d\u0001\u001d\u0001"+
		"\u001d\u0001\u001d\u0003\u001d\u01fe\b\u001d\u0001\u001d\u0001\u001d\u0001"+
		"\u001d\u0001\u001e\u0001\u001e\u0001\u001e\u0005\u001e\u0206\b\u001e\n"+
		"\u001e\f\u001e\u0209\t\u001e\u0001\u001e\u0001\u001e\u0001\u001e\u0001"+
		"\u001f\u0003\u001f\u020f\b\u001f\u0001\u001f\u0001\u001f\u0001\u001f\u0001"+
		"\u001f\u0003\u001f\u0215\b\u001f\u0001\u001f\u0001\u001f\u0001 \u0001"+
		" \u0001 \u0005 \u021c\b \n \f \u021f\t \u0001 \u0001 \u0001 \u0001!\u0001"+
		"!\u0001!\u0001!\u0001!\u0001!\u0001\"\u0001\"\u0001\"\u0005\"\u022d\b"+
		"\"\n\"\f\"\u0230\t\"\u0001\"\u0001\"\u0001\"\u0001#\u0001#\u0001#\u0001"+
		"#\u0001#\u0005#\u023a\b#\n#\f#\u023d\t#\u0001#\u0001#\u0001#\u0001$\u0001"+
		"$\u0001$\u0001$\u0001$\u0001$\u0001$\u0001%\u0001%\u0001%\u0001%\u0005"+
		"%\u024d\b%\n%\f%\u0250\t%\u0001%\u0001%\u0001%\u0001&\u0001&\u0001&\u0005"+
		"&\u0258\b&\n&\f&\u025b\t&\u0001&\u0001&\u0001&\u0001\'\u0001\'\u0001\'"+
		"\u0001\'\u0005\'\u0264\b\'\n\'\f\'\u0267\t\'\u0001\'\u0001\'\u0005\'\u026b"+
		"\b\'\n\'\f\'\u026e\t\'\u0001\'\u0001\'\u0001\'\u0003\'\u0273\b\'\u0001"+
		"\'\u0003\'\u0276\b\'\u0001\'\u0003\'\u0279\b\'\u0001\'\u0003\'\u027c\b"+
		"\'\u0001\'\u0003\'\u027f\b\'\u0001\'\u0003\'\u0282\b\'\u0001\'\u0001\'"+
		"\u0001\'\u0001(\u0001(\u0001(\u0001(\u0005(\u028b\b(\n(\f(\u028e\t(\u0001"+
		"(\u0001(\u0001(\u0001)\u0001)\u0001)\u0005)\u0296\b)\n)\f)\u0299\t)\u0001"+
		")\u0001)\u0001)\u0001*\u0001*\u0001*\u0001*\u0005*\u02a2\b*\n*\f*\u02a5"+
		"\t*\u0001*\u0001*\u0001*\u0001+\u0001+\u0001+\u0001+\u0005+\u02ae\b+\n"+
		"+\f+\u02b1\t+\u0001+\u0005+\u02b4\b+\n+\f+\u02b7\t+\u0001+\u0001+\u0001"+
		",\u0001,\u0001,\u0005,\u02be\b,\n,\f,\u02c1\t,\u0001,\u0001,\u0001,\u0001"+
		"-\u0001-\u0001-\u0001-\u0001-\u0005-\u02cb\b-\n-\f-\u02ce\t-\u0001-\u0001"+
		"-\u0005-\u02d2\b-\n-\f-\u02d5\t-\u0001-\u0001-\u0001-\u0001-\u0005-\u02db"+
		"\b-\n-\f-\u02de\t-\u0001-\u0001-\u0001.\u0001.\u0001.\u0005.\u02e5\b."+
		"\n.\f.\u02e8\t.\u0001.\u0001.\u0001.\u0001/\u0001/\u0001/\u0001/\u0001"+
		"/\u0001/\u0001/\u00010\u00010\u00010\u00010\u00050\u02f8\b0\n0\f0\u02fb"+
		"\t0\u00010\u00010\u00010\u00011\u00011\u00011\u00051\u0303\b1\n1\f1\u0306"+
		"\t1\u00011\u00011\u00011\u00012\u00012\u00012\u00012\u00012\u00012\u0005"+
		"2\u0311\b2\n2\f2\u0314\t2\u00012\u00012\u00012\u00013\u00013\u00013\u0005"+
		"3\u031c\b3\n3\f3\u031f\t3\u00013\u00013\u00013\u00014\u00014\u00014\u0001"+
		"4\u00014\u00014\u00014\u00014\u00014\u00054\u032d\b4\n4\f4\u0330\t4\u0001"+
		"4\u00014\u00014\u00015\u00015\u00015\u00055\u0338\b5\n5\f5\u033b\t5\u0001"+
		"5\u00015\u00015\u00016\u00016\u00016\u00056\u0343\b6\n6\f6\u0346\t6\u0001"+
		"6\u00016\u00016\u00017\u00017\u00017\u00017\u00018\u00018\u00018\u0001"+
		"8\u00019\u00019\u00019\u00019\u00019\u00019\u00019\u00019\u0001:\u0001"+
		":\u0001:\u0005:\u035e\b:\n:\f:\u0361\t:\u0001:\u0001:\u0001:\u0001;\u0001"+
		";\u0001;\u0001;\u0001<\u0001<\u0001<\u0004<\u036d\b<\u000b<\f<\u036e\u0001"+
		"<\u0001<\u0001<\u0001=\u0001=\u0001=\u0001=\u0001=\u0001>\u0001>\u0001"+
		">\u0005>\u037c\b>\n>\f>\u037f\t>\u0001>\u0001>\u0001>\u0001?\u0001?\u0001"+
		"?\u0001?\u0001?\u0003?\u0389\b?\u0001?\u0003?\u038c\b?\u0001?\u0001?\u0003"+
		"?\u0390\b?\u0001?\u0001?\u0001@\u0001@\u0001@\u0005@\u0397\b@\n@\f@\u039a"+
		"\t@\u0001@\u0001@\u0001@\u0001A\u0001A\u0001A\u0001A\u0003A\u03a3\bA\u0001"+
		"A\u0003A\u03a6\bA\u0001A\u0003A\u03a9\bA\u0001A\u0001A\u0001B\u0001B\u0001"+
		"B\u0001B\u0005B\u03b1\bB\nB\fB\u03b4\tB\u0001B\u0001B\u0001C\u0001C\u0001"+
		"C\u0001C\u0005C\u03bc\bC\nC\fC\u03bf\tC\u0001C\u0001C\u0001C\u0001D\u0001"+
		"D\u0001D\u0001D\u0001D\u0001D\u0005D\u03ca\bD\nD\fD\u03cd\tD\u0001D\u0001"+
		"D\u0001D\u0001E\u0001E\u0001E\u0001E\u0001E\u0001E\u0001E\u0001F\u0001"+
		"F\u0001F\u0005F\u03dc\bF\nF\fF\u03df\tF\u0001G\u0001G\u0001G\u0001G\u0001"+
		"G\u0001G\u0001G\u0001H\u0001H\u0001H\u0001H\u0001H\u0001H\u0001H\u0005"+
		"H\u03ef\bH\nH\fH\u03f2\tH\u0001H\u0001H\u0001H\u0001I\u0001I\u0001I\u0001"+
		"I\u0001I\u0001J\u0001J\u0001J\u0001J\u0001J\u0001J\u0001J\u0001K\u0001"+
		"K\u0001K\u0005K\u0406\bK\nK\fK\u0409\tK\u0001L\u0001L\u0001L\u0001L\u0001"+
		"L\u0001L\u0001L\u0001M\u0001M\u0001M\u0005M\u0415\bM\nM\fM\u0418\tM\u0001"+
		"M\u0001M\u0001M\u0001N\u0001N\u0001N\u0001N\u0001N\u0001N\u0001N\u0001"+
		"N\u0001N\u0005N\u0426\bN\nN\fN\u0429\tN\u0001N\u0001N\u0001N\u0001O\u0001"+
		"O\u0001O\u0001O\u0001O\u0001O\u0001P\u0001P\u0001P\u0005P\u0437\bP\nP"+
		"\fP\u043a\tP\u0001P\u0001P\u0001P\u0001Q\u0001Q\u0001Q\u0001Q\u0001Q\u0001"+
		"R\u0001R\u0001S\u0001S\u0001T\u0001T\u0001U\u0001U\u0001U\u0003U\u044d"+
		"\bU\u0001V\u0001V\u0001V\u0001W\u0001W\u0001X\u0001X\u0001X\u0001Y\u0001"+
		"Y\u0001Z\u0001Z\u0001Z\u0003Z\u045c\bZ\u0001[\u0001[\u0001[\u0001\\\u0001"+
		"\\\u0001]\u0001]\u0001^\u0001^\u0001_\u0005_\u0468\b_\n_\f_\u046b\t_\u0001"+
		"_\u0001_\u0001`\u0001`\u0001a\u0001a\u0005a\u0473\ba\na\fa\u0476\ta\u0001"+
		"b\u0001b\u0001b\u0001\u0469\u0000c\u0000\u0002\u0004\u0006\b\n\f\u000e"+
		"\u0010\u0012\u0014\u0016\u0018\u001a\u001c\u001e \"$&(*,.02468:<>@BDF"+
		"HJLNPRTVXZ\\^`bdfhjlnprtvxz|~\u0080\u0082\u0084\u0086\u0088\u008a\u008c"+
		"\u008e\u0090\u0092\u0094\u0096\u0098\u009a\u009c\u009e\u00a0\u00a2\u00a4"+
		"\u00a6\u00a8\u00aa\u00ac\u00ae\u00b0\u00b2\u00b4\u00b6\u00b8\u00ba\u00bc"+
		"\u00be\u00c0\u00c2\u00c4\u0000\u0004\u0002\u0000AAEE\u0001\u000089\u0001"+
		"\u0000$/\u0006\u00000357:;=>@AEE\u0499\u0000\u00ce\u0001\u0000\u0000\u0000"+
		"\u0002\u00d4\u0001\u0000\u0000\u0000\u0004\u00df\u0001\u0000\u0000\u0000"+
		"\u0006\u00e7\u0001\u0000\u0000\u0000\b\u00ed\u0001\u0000\u0000\u0000\n"+
		"\u00fa\u0001\u0000\u0000\u0000\f\u0105\u0001\u0000\u0000\u0000\u000e\u0129"+
		"\u0001\u0000\u0000\u0000\u0010\u0135\u0001\u0000\u0000\u0000\u0012\u0148"+
		"\u0001\u0000\u0000\u0000\u0014\u0157\u0001\u0000\u0000\u0000\u0016\u0159"+
		"\u0001\u0000\u0000\u0000\u0018\u0161\u0001\u0000\u0000\u0000\u001a\u0168"+
		"\u0001\u0000\u0000\u0000\u001c\u0171\u0001\u0000\u0000\u0000\u001e\u0173"+
		"\u0001\u0000\u0000\u0000 \u017e\u0001\u0000\u0000\u0000\"\u0185\u0001"+
		"\u0000\u0000\u0000$\u018c\u0001\u0000\u0000\u0000&\u0198\u0001\u0000\u0000"+
		"\u0000(\u01a2\u0001\u0000\u0000\u0000*\u01bd\u0001\u0000\u0000\u0000,"+
		"\u01c3\u0001\u0000\u0000\u0000.\u01c5\u0001\u0000\u0000\u00000\u01c7\u0001"+
		"\u0000\u0000\u00002\u01ca\u0001\u0000\u0000\u00004\u01d8\u0001\u0000\u0000"+
		"\u00006\u01e9\u0001\u0000\u0000\u00008\u01f7\u0001\u0000\u0000\u0000:"+
		"\u01f9\u0001\u0000\u0000\u0000<\u0202\u0001\u0000\u0000\u0000>\u020e\u0001"+
		"\u0000\u0000\u0000@\u0218\u0001\u0000\u0000\u0000B\u0223\u0001\u0000\u0000"+
		"\u0000D\u0229\u0001\u0000\u0000\u0000F\u0234\u0001\u0000\u0000\u0000H"+
		"\u0241\u0001\u0000\u0000\u0000J\u0248\u0001\u0000\u0000\u0000L\u0254\u0001"+
		"\u0000\u0000\u0000N\u025f\u0001\u0000\u0000\u0000P\u0286\u0001\u0000\u0000"+
		"\u0000R\u0292\u0001\u0000\u0000\u0000T\u029d\u0001\u0000\u0000\u0000V"+
		"\u02a9\u0001\u0000\u0000\u0000X\u02ba\u0001\u0000\u0000\u0000Z\u02c5\u0001"+
		"\u0000\u0000\u0000\\\u02e1\u0001\u0000\u0000\u0000^\u02ec\u0001\u0000"+
		"\u0000\u0000`\u02f3\u0001\u0000\u0000\u0000b\u02ff\u0001\u0000\u0000\u0000"+
		"d\u030a\u0001\u0000\u0000\u0000f\u0318\u0001\u0000\u0000\u0000h\u0323"+
		"\u0001\u0000\u0000\u0000j\u0334\u0001\u0000\u0000\u0000l\u033f\u0001\u0000"+
		"\u0000\u0000n\u034a\u0001\u0000\u0000\u0000p\u034e\u0001\u0000\u0000\u0000"+
		"r\u0352\u0001\u0000\u0000\u0000t\u035a\u0001\u0000\u0000\u0000v\u0365"+
		"\u0001\u0000\u0000\u0000x\u0369\u0001\u0000\u0000\u0000z\u0373\u0001\u0000"+
		"\u0000\u0000|\u0378\u0001\u0000\u0000\u0000~\u0383\u0001\u0000\u0000\u0000"+
		"\u0080\u0393\u0001\u0000\u0000\u0000\u0082\u039e\u0001\u0000\u0000\u0000"+
		"\u0084\u03ac\u0001\u0000\u0000\u0000\u0086\u03b7\u0001\u0000\u0000\u0000"+
		"\u0088\u03c3\u0001\u0000\u0000\u0000\u008a\u03d1\u0001\u0000\u0000\u0000"+
		"\u008c\u03d8\u0001\u0000\u0000\u0000\u008e\u03e0\u0001\u0000\u0000\u0000"+
		"\u0090\u03e7\u0001\u0000\u0000\u0000\u0092\u03f6\u0001\u0000\u0000\u0000"+
		"\u0094\u03fb\u0001\u0000\u0000\u0000\u0096\u0402\u0001\u0000\u0000\u0000"+
		"\u0098\u040a\u0001\u0000\u0000\u0000\u009a\u0411\u0001\u0000\u0000\u0000"+
		"\u009c\u041c\u0001\u0000\u0000\u0000\u009e\u042d\u0001\u0000\u0000\u0000"+
		"\u00a0\u0433\u0001\u0000\u0000\u0000\u00a2\u043e\u0001\u0000\u0000\u0000"+
		"\u00a4\u0443\u0001\u0000\u0000\u0000\u00a6\u0445\u0001\u0000\u0000\u0000"+
		"\u00a8\u0447\u0001\u0000\u0000\u0000\u00aa\u044c\u0001\u0000\u0000\u0000"+
		"\u00ac\u044e\u0001\u0000\u0000\u0000\u00ae\u0451\u0001\u0000\u0000\u0000"+
		"\u00b0\u0453\u0001\u0000\u0000\u0000\u00b2\u0456\u0001\u0000\u0000\u0000"+
		"\u00b4\u045b\u0001\u0000\u0000\u0000\u00b6\u045d\u0001\u0000\u0000\u0000"+
		"\u00b8\u0460\u0001\u0000\u0000\u0000\u00ba\u0462\u0001\u0000\u0000\u0000"+
		"\u00bc\u0464\u0001\u0000\u0000\u0000\u00be\u0469\u0001\u0000\u0000\u0000"+
		"\u00c0\u046e\u0001\u0000\u0000\u0000\u00c2\u0474\u0001\u0000\u0000\u0000"+
		"\u00c4\u0477\u0001\u0000\u0000\u0000\u00c6\u00cf\u0003\u0002\u0001\u0000"+
		"\u00c7\u00cf\u0003\u00bc^\u0000\u00c8\u00cf\u0003\n\u0005\u0000\u00c9"+
		"\u00cf\u0003D\"\u0000\u00ca\u00cf\u0003L&\u0000\u00cb\u00cf\u0003b1\u0000"+
		"\u00cc\u00cf\u0003f3\u0000\u00cd\u00cf\u0003\u009aM\u0000\u00ce\u00c6"+
		"\u0001\u0000\u0000\u0000\u00ce\u00c7\u0001\u0000\u0000\u0000\u00ce\u00c8"+
		"\u0001\u0000\u0000\u0000\u00ce\u00c9\u0001\u0000\u0000\u0000\u00ce\u00ca"+
		"\u0001\u0000\u0000\u0000\u00ce\u00cb\u0001\u0000\u0000\u0000\u00ce\u00cc"+
		"\u0001\u0000\u0000\u0000\u00ce\u00cd\u0001\u0000\u0000\u0000\u00cf\u00d0"+
		"\u0001\u0000\u0000\u0000\u00d0\u00ce\u0001\u0000\u0000\u0000\u00d0\u00d1"+
		"\u0001\u0000\u0000\u0000\u00d1\u00d2\u0001\u0000\u0000\u0000\u00d2\u00d3"+
		"\u0005\u0000\u0000\u0001\u00d3\u0001\u0001\u0000\u0000\u0000\u00d4\u00d5"+
		"\u0005\u0015\u0000\u0000\u00d5\u00d9\u0005=\u0000\u0000\u00d6\u00d8\u0003"+
		"\u0004\u0002\u0000\u00d7\u00d6\u0001\u0000\u0000\u0000\u00d8\u00db\u0001"+
		"\u0000\u0000\u0000\u00d9\u00d7\u0001\u0000\u0000\u0000\u00d9\u00da\u0001"+
		"\u0000\u0000\u0000\u00da\u00dc\u0001\u0000\u0000\u0000\u00db\u00d9\u0001"+
		"\u0000\u0000\u0000\u00dc\u00dd\u0005>\u0000\u0000\u00dd\u00de\u00054\u0000"+
		"\u0000\u00de\u0003\u0001\u0000\u0000\u0000\u00df\u00e2\u0005\u0001\u0000"+
		"\u0000\u00e0\u00e3\u0003\u0006\u0003\u0000\u00e1\u00e3\u0003\b\u0004\u0000"+
		"\u00e2\u00e0\u0001\u0000\u0000\u0000\u00e2\u00e1\u0001\u0000\u0000\u0000"+
		"\u00e3\u00e4\u0001\u0000\u0000\u0000\u00e4\u00e5\u0005\u0001\u0000\u0000"+
		"\u00e5\u00e6\u00054\u0000\u0000\u00e6\u0005\u0001\u0000\u0000\u0000\u00e7"+
		"\u00e8\u0005\u0014\u0000\u0000\u00e8\u00e9\u00056\u0000\u0000\u00e9\u00ea"+
		"\u00056\u0000\u0000\u00ea\u00eb\u0003\b\u0004\u0000\u00eb\u0007\u0001"+
		"\u0000\u0000\u0000\u00ec\u00ee\u00053\u0000\u0000\u00ed\u00ec\u0001\u0000"+
		"\u0000\u0000\u00ed\u00ee\u0001\u0000\u0000\u0000\u00ee\u00ef\u0001\u0000"+
		"\u0000\u0000\u00ef\u00f4\u0005A\u0000\u0000\u00f0\u00f1\u00053\u0000\u0000"+
		"\u00f1\u00f3\u0005A\u0000\u0000\u00f2\u00f0\u0001\u0000\u0000\u0000\u00f3"+
		"\u00f6\u0001\u0000\u0000\u0000\u00f4\u00f2\u0001\u0000\u0000\u0000\u00f4"+
		"\u00f5\u0001\u0000\u0000\u0000\u00f5\u00f7\u0001\u0000\u0000\u0000\u00f6"+
		"\u00f4\u0001\u0000\u0000\u0000\u00f7\u00f8\u00055\u0000\u0000\u00f8\u00f9"+
		"\u0005A\u0000\u0000\u00f9\t\u0001\u0000\u0000\u0000\u00fa\u00fb\u0005"+
		"\u0003\u0000\u0000\u00fb\u00ff\u0005=\u0000\u0000\u00fc\u00fe\u0003\f"+
		"\u0006\u0000\u00fd\u00fc\u0001\u0000\u0000\u0000\u00fe\u0101\u0001\u0000"+
		"\u0000\u0000\u00ff\u00fd\u0001\u0000\u0000\u0000\u00ff\u0100\u0001\u0000"+
		"\u0000\u0000\u0100\u0102\u0001\u0000\u0000\u0000\u0101\u00ff\u0001\u0000"+
		"\u0000\u0000\u0102\u0103\u0005>\u0000\u0000\u0103\u0104\u00054\u0000\u0000"+
		"\u0104\u000b\u0001\u0000\u0000\u0000\u0105\u0106\u0005\u0004\u0000\u0000"+
		"\u0106\u0107\u00030\u0018\u0000\u0107\u010b\u0005:\u0000\u0000\u0108\u010a"+
		"\u0003\u00b6[\u0000\u0109\u0108\u0001\u0000\u0000\u0000\u010a\u010d\u0001"+
		"\u0000\u0000\u0000\u010b\u0109\u0001\u0000\u0000\u0000\u010b\u010c\u0001"+
		"\u0000\u0000\u0000\u010c\u0112\u0001\u0000\u0000\u0000\u010d\u010b\u0001"+
		"\u0000\u0000\u0000\u010e\u010f\u0005@\u0000\u0000\u010f\u0111\u0003\u00b6"+
		"[\u0000\u0110\u010e\u0001\u0000\u0000\u0000\u0111\u0114\u0001\u0000\u0000"+
		"\u0000\u0112\u0110\u0001\u0000\u0000\u0000\u0112\u0113\u0001\u0000\u0000"+
		"\u0000\u0113\u0115\u0001\u0000\u0000\u0000\u0114\u0112\u0001\u0000\u0000"+
		"\u0000\u0115\u0116\u0005;\u0000\u0000\u0116\u0118\u0005=\u0000\u0000\u0117"+
		"\u0119\u0003\u000e\u0007\u0000\u0118\u0117\u0001\u0000\u0000\u0000\u0118"+
		"\u0119\u0001\u0000\u0000\u0000\u0119\u011b\u0001\u0000\u0000\u0000\u011a"+
		"\u011c\u0003\u001e\u000f\u0000\u011b\u011a\u0001\u0000\u0000\u0000\u011b"+
		"\u011c\u0001\u0000\u0000\u0000\u011c\u0122\u0001\u0000\u0000\u0000\u011d"+
		"\u0123\u0003:\u001d\u0000\u011e\u0123\u0003(\u0014\u0000\u011f\u0123\u0003"+
		"$\u0012\u0000\u0120\u0123\u0003 \u0010\u0000\u0121\u0123\u0003\"\u0011"+
		"\u0000\u0122\u011d\u0001\u0000\u0000\u0000\u0122\u011e\u0001\u0000\u0000"+
		"\u0000\u0122\u011f\u0001\u0000\u0000\u0000\u0122\u0120\u0001\u0000\u0000"+
		"\u0000\u0122\u0121\u0001\u0000\u0000\u0000\u0123\u0124\u0001\u0000\u0000"+
		"\u0000\u0124\u0122\u0001\u0000\u0000\u0000\u0124\u0125\u0001\u0000\u0000"+
		"\u0000\u0125\u0126\u0001\u0000\u0000\u0000\u0126\u0127\u0005>\u0000\u0000"+
		"\u0127\u0128\u00054\u0000\u0000\u0128\r\u0001\u0000\u0000\u0000\u0129"+
		"\u012a\u0005\u0005\u0000\u0000\u012a\u012f\u0005=\u0000\u0000\u012b\u012e"+
		"\u0003\u0010\b\u0000\u012c\u012e\u0003&\u0013\u0000\u012d\u012b\u0001"+
		"\u0000\u0000\u0000\u012d\u012c\u0001\u0000\u0000\u0000\u012e\u0131\u0001"+
		"\u0000\u0000\u0000\u012f\u012d\u0001\u0000\u0000\u0000\u012f\u0130\u0001"+
		"\u0000\u0000\u0000\u0130\u0132\u0001\u0000\u0000\u0000\u0131\u012f\u0001"+
		"\u0000\u0000\u0000\u0132\u0133\u0005>\u0000\u0000\u0133\u0134\u00054\u0000"+
		"\u0000\u0134\u000f\u0001\u0000\u0000\u0000\u0135\u0139\u0005\u0013\u0000"+
		"\u0000\u0136\u013a\u0003\u00acV\u0000\u0137\u013a\u00052\u0000\u0000\u0138"+
		"\u013a\u00051\u0000\u0000\u0139\u0136\u0001\u0000\u0000\u0000\u0139\u0137"+
		"\u0001\u0000\u0000\u0000\u0139\u0138\u0001\u0000\u0000\u0000\u013a\u013f"+
		"\u0001\u0000\u0000\u0000\u013b\u013c\u0005@\u0000\u0000\u013c\u013e\u0003"+
		"\u00acV\u0000\u013d\u013b\u0001\u0000\u0000\u0000\u013e\u0141\u0001\u0000"+
		"\u0000\u0000\u013f\u013d\u0001\u0000\u0000\u0000\u013f\u0140\u0001\u0000"+
		"\u0000\u0000\u0140\u0142\u0001\u0000\u0000\u0000\u0141\u013f\u0001\u0000"+
		"\u0000\u0000\u0142\u0143\u0005\u0016\u0000\u0000\u0143\u0144\u0005=\u0000"+
		"\u0000\u0144\u0145\u0003\u0012\t\u0000\u0145\u0146\u0005>\u0000\u0000"+
		"\u0146\u0011\u0001\u0000\u0000\u0000\u0147\u0149\u0003\u0014\n\u0000\u0148"+
		"\u0147\u0001\u0000\u0000\u0000\u0149\u014a\u0001\u0000\u0000\u0000\u014a"+
		"\u0148\u0001\u0000\u0000\u0000\u014a\u014b\u0001\u0000\u0000\u0000\u014b"+
		"\u0013\u0001\u0000\u0000\u0000\u014c\u014d\u0005\t\u0000\u0000\u014d\u014f"+
		"\u0005=\u0000\u0000\u014e\u0150\u0003\u0016\u000b\u0000\u014f\u014e\u0001"+
		"\u0000\u0000\u0000\u0150\u0151\u0001\u0000\u0000\u0000\u0151\u014f\u0001"+
		"\u0000\u0000\u0000\u0151\u0152\u0001\u0000\u0000\u0000\u0152\u0153\u0001"+
		"\u0000\u0000\u0000\u0153\u0154\u0005>\u0000\u0000\u0154\u0155\u00055\u0000"+
		"\u0000\u0155\u0158\u0001\u0000\u0000\u0000\u0156\u0158\u0003\u0016\u000b"+
		"\u0000\u0157\u014c\u0001\u0000\u0000\u0000\u0157\u0156\u0001\u0000\u0000"+
		"\u0000\u0158\u0015\u0001\u0000\u0000\u0000\u0159\u015a\u0003\u0018\f\u0000"+
		"\u015a\u015e\u00055\u0000\u0000\u015b\u015d\u0005F\u0000\u0000\u015c\u015b"+
		"\u0001\u0000\u0000\u0000\u015d\u0160\u0001\u0000\u0000\u0000\u015e\u015c"+
		"\u0001\u0000\u0000\u0000\u015e\u015f\u0001\u0000\u0000\u0000\u015f\u0017"+
		"\u0001\u0000\u0000\u0000\u0160\u015e\u0001\u0000\u0000\u0000\u0161\u0162"+
		"\u0003\u00aaU\u0000\u0162\u0163\u0003\u001c\u000e\u0000\u0163\u0164\u0005"+
		"6\u0000\u0000\u0164\u0165\u0003\u00b2Y\u0000\u0165\u0166\u0003\u00b4Z"+
		"\u0000\u0166\u0019\u0001\u0000\u0000\u0000\u0167\u0169\u0005\t\u0000\u0000"+
		"\u0168\u0167\u0001\u0000\u0000\u0000\u0168\u0169\u0001\u0000\u0000\u0000"+
		"\u0169\u016a\u0001\u0000\u0000\u0000\u016a\u016b\u0003\u00aaU\u0000\u016b"+
		"\u016c\u0003\u001c\u000e\u0000\u016c\u016d\u00056\u0000\u0000\u016d\u016e"+
		"\u0003\u00b2Y\u0000\u016e\u016f\u0003\u00b4Z\u0000\u016f\u0170\u00054"+
		"\u0000\u0000\u0170\u001b\u0001\u0000\u0000\u0000\u0171\u0172\u0005A\u0000"+
		"\u0000\u0172\u001d\u0001\u0000\u0000\u0000\u0173\u0174\u0005\f\u0000\u0000"+
		"\u0174\u0178\u0005=\u0000\u0000\u0175\u0177\u0003&\u0013\u0000\u0176\u0175"+
		"\u0001\u0000\u0000\u0000\u0177\u017a\u0001\u0000\u0000\u0000\u0178\u0176"+
		"\u0001\u0000\u0000\u0000\u0178\u0179\u0001\u0000\u0000\u0000\u0179\u017b"+
		"\u0001\u0000\u0000\u0000\u017a\u0178\u0001\u0000\u0000\u0000\u017b\u017c"+
		"\u0005>\u0000\u0000\u017c\u017d\u00054\u0000\u0000\u017d\u001f\u0001\u0000"+
		"\u0000\u0000\u017e\u017f\u0005\u0011\u0000\u0000\u017f\u0180\u0005=\u0000"+
		"\u0000\u0180\u0181\u0003\u00a8T\u0000\u0181\u0182\u00054\u0000\u0000\u0182"+
		"\u0183\u0005>\u0000\u0000\u0183\u0184\u00054\u0000\u0000\u0184!\u0001"+
		"\u0000\u0000\u0000\u0185\u0186\u0005\u0012\u0000\u0000\u0186\u0187\u0005"+
		"=\u0000\u0000\u0187\u0188\u0003\u00a8T\u0000\u0188\u0189\u00054\u0000"+
		"\u0000\u0189\u018a\u0005>\u0000\u0000\u018a\u018b\u00054\u0000\u0000\u018b"+
		"#\u0001\u0000\u0000\u0000\u018c\u018d\u0005\u0007\u0000\u0000\u018d\u0191"+
		"\u0005=\u0000\u0000\u018e\u0190\u0003&\u0013\u0000\u018f\u018e\u0001\u0000"+
		"\u0000\u0000\u0190\u0193\u0001\u0000\u0000\u0000\u0191\u018f\u0001\u0000"+
		"\u0000\u0000\u0191\u0192\u0001\u0000\u0000\u0000\u0192\u0194\u0001\u0000"+
		"\u0000\u0000\u0193\u0191\u0001\u0000\u0000\u0000\u0194\u0195\u0005>\u0000"+
		"\u0000\u0195\u0196\u00054\u0000\u0000\u0196%\u0001\u0000\u0000\u0000\u0197"+
		"\u0199\u0005\t\u0000\u0000\u0198\u0197\u0001\u0000\u0000\u0000\u0198\u0199"+
		"\u0001\u0000\u0000\u0000\u0199\u019a\u0001\u0000\u0000\u0000\u019a\u019b"+
		"\u0003\u00aaU\u0000\u019b\u019c\u0003\u00b2Y\u0000\u019c\u019e\u0003\u00b4"+
		"Z\u0000\u019d\u019f\u0005\n\u0000\u0000\u019e\u019d\u0001\u0000\u0000"+
		"\u0000\u019e\u019f\u0001\u0000\u0000\u0000\u019f\u01a0\u0001\u0000\u0000"+
		"\u0000\u01a0\u01a1\u00054\u0000\u0000\u01a1\'\u0001\u0000\u0000\u0000"+
		"\u01a2\u01a3\u0005\u000b\u0000\u0000\u01a3\u01a7\u0005=\u0000\u0000\u01a4"+
		"\u01a6\u0003*\u0015\u0000\u01a5\u01a4\u0001\u0000\u0000\u0000\u01a6\u01a9"+
		"\u0001\u0000\u0000\u0000\u01a7\u01a5\u0001\u0000\u0000\u0000\u01a7\u01a8"+
		"\u0001\u0000\u0000\u0000\u01a8\u01aa\u0001\u0000\u0000\u0000\u01a9\u01a7"+
		"\u0001\u0000\u0000\u0000\u01aa\u01ab\u0005>\u0000\u0000\u01ab\u01ac\u0005"+
		"4\u0000\u0000\u01ac)\u0001\u0000\u0000\u0000\u01ad\u01ae\u00030\u0018"+
		"\u0000\u01ae\u01af\u0005:\u0000\u0000\u01af\u01b4\u0003,\u0016\u0000\u01b0"+
		"\u01b1\u0005@\u0000\u0000\u01b1\u01b3\u0003,\u0016\u0000\u01b2\u01b0\u0001"+
		"\u0000\u0000\u0000\u01b3\u01b6\u0001\u0000\u0000\u0000\u01b4\u01b2\u0001"+
		"\u0000\u0000\u0000\u01b4\u01b5\u0001\u0000\u0000\u0000\u01b5\u01b7\u0001"+
		"\u0000\u0000\u0000\u01b6\u01b4\u0001\u0000\u0000\u0000\u01b7\u01b8\u0005"+
		";\u0000\u0000\u01b8\u01b9\u0001\u0000\u0000\u0000\u01b9\u01ba\u00054\u0000"+
		"\u0000\u01ba\u01be\u0001\u0000\u0000\u0000\u01bb\u01bc\u0005<\u0000\u0000"+
		"\u01bc\u01be\u00054\u0000\u0000\u01bd\u01ad\u0001\u0000\u0000\u0000\u01bd"+
		"\u01bb\u0001\u0000\u0000\u0000\u01be+\u0001\u0000\u0000\u0000\u01bf\u01c4"+
		"\u0003.\u0017\u0000\u01c0\u01c4\u00032\u0019\u0000\u01c1\u01c4\u00034"+
		"\u001a\u0000\u01c2\u01c4\u0005?\u0000\u0000\u01c3\u01bf\u0001\u0000\u0000"+
		"\u0000\u01c3\u01c0\u0001\u0000\u0000\u0000\u01c3\u01c1\u0001\u0000\u0000"+
		"\u0000\u01c3\u01c2\u0001\u0000\u0000\u0000\u01c4-\u0001\u0000\u0000\u0000"+
		"\u01c5\u01c6\u0005A\u0000\u0000\u01c6/\u0001\u0000\u0000\u0000\u01c7\u01c8"+
		"\u0005A\u0000\u0000\u01c81\u0001\u0000\u0000\u0000\u01c9\u01cb\u0005\u0001"+
		"\u0000\u0000\u01ca\u01c9\u0001\u0000\u0000\u0000\u01ca\u01cb\u0001\u0000"+
		"\u0000\u0000\u01cb\u01cc\u0001\u0000\u0000\u0000\u01cc\u01cd\u00053\u0000"+
		"\u0000\u01cd\u01d2\u0005A\u0000\u0000\u01ce\u01cf\u00053\u0000\u0000\u01cf"+
		"\u01d1\u0005A\u0000\u0000\u01d0\u01ce\u0001\u0000\u0000\u0000\u01d1\u01d4"+
		"\u0001\u0000\u0000\u0000\u01d2\u01d0\u0001\u0000\u0000\u0000\u01d2\u01d3"+
		"\u0001\u0000\u0000\u0000\u01d3\u01d6\u0001\u0000\u0000\u0000\u01d4\u01d2"+
		"\u0001\u0000\u0000\u0000\u01d5\u01d7\u0005\u0001\u0000\u0000\u01d6\u01d5"+
		"\u0001\u0000\u0000\u0000\u01d6\u01d7\u0001\u0000\u0000\u0000\u01d73\u0001"+
		"\u0000\u0000\u0000\u01d8\u01dc\u0005=\u0000\u0000\u01d9\u01db\u00036\u001b"+
		"\u0000\u01da\u01d9\u0001\u0000\u0000\u0000\u01db\u01de\u0001\u0000\u0000"+
		"\u0000\u01dc\u01da\u0001\u0000\u0000\u0000\u01dc\u01dd\u0001\u0000\u0000"+
		"\u0000\u01dd\u01e3\u0001\u0000\u0000\u0000\u01de\u01dc\u0001\u0000\u0000"+
		"\u0000\u01df\u01e0\u0005@\u0000\u0000\u01e0\u01e2\u00036\u001b\u0000\u01e1"+
		"\u01df\u0001\u0000\u0000\u0000\u01e2\u01e5\u0001\u0000\u0000\u0000\u01e3"+
		"\u01e1\u0001\u0000\u0000\u0000\u01e3\u01e4\u0001\u0000\u0000\u0000\u01e4"+
		"\u01e6\u0001\u0000\u0000\u0000\u01e5\u01e3\u0001\u0000\u0000\u0000\u01e6"+
		"\u01e7\u0005>\u0000\u0000\u01e75\u0001\u0000\u0000\u0000\u01e8\u01ea\u0005"+
		"\u0001\u0000\u0000\u01e9\u01e8\u0001\u0000\u0000\u0000\u01e9\u01ea\u0001"+
		"\u0000\u0000\u0000\u01ea\u01eb\u0001\u0000\u0000\u0000\u01eb\u01ed\u0003"+
		"\u00ba]\u0000\u01ec\u01ee\u0005\u0001\u0000\u0000\u01ed\u01ec\u0001\u0000"+
		"\u0000\u0000\u01ed\u01ee\u0001\u0000\u0000\u0000\u01ee\u01ef\u0001\u0000"+
		"\u0000\u0000\u01ef\u01f1\u00056\u0000\u0000\u01f0\u01f2\u0005\u0001\u0000"+
		"\u0000\u01f1\u01f0\u0001\u0000\u0000\u0000\u01f1\u01f2\u0001\u0000\u0000"+
		"\u0000\u01f2\u01f3\u0001\u0000\u0000\u0000\u01f3\u01f5\u00038\u001c\u0000"+
		"\u01f4\u01f6\u0005\u0001\u0000\u0000\u01f5\u01f4\u0001\u0000\u0000\u0000"+
		"\u01f5\u01f6\u0001\u0000\u0000\u0000\u01f67\u0001\u0000\u0000\u0000\u01f7"+
		"\u01f8\u0007\u0000\u0000\u0000\u01f89\u0001\u0000\u0000\u0000\u01f9\u01fa"+
		"\u0005\u0006\u0000\u0000\u01fa\u01fb\u0005=\u0000\u0000\u01fb\u01fd\u0003"+
		"<\u001e\u0000\u01fc\u01fe\u0003@ \u0000\u01fd\u01fc\u0001\u0000\u0000"+
		"\u0000\u01fd\u01fe\u0001\u0000\u0000\u0000\u01fe\u01ff\u0001\u0000\u0000"+
		"\u0000\u01ff\u0200\u0005>\u0000\u0000\u0200\u0201\u00054\u0000\u0000\u0201"+
		";\u0001\u0000\u0000\u0000\u0202\u0203\u0005\b\u0000\u0000\u0203\u0207"+
		"\u0005=\u0000\u0000\u0204\u0206\u0003>\u001f\u0000\u0205\u0204\u0001\u0000"+
		"\u0000\u0000\u0206\u0209\u0001\u0000\u0000\u0000\u0207\u0205\u0001\u0000"+
		"\u0000\u0000\u0207\u0208\u0001\u0000\u0000\u0000\u0208\u020a\u0001\u0000"+
		"\u0000\u0000\u0209\u0207\u0001\u0000\u0000\u0000\u020a\u020b\u0005>\u0000"+
		"\u0000\u020b\u020c\u00054\u0000\u0000\u020c=\u0001\u0000\u0000\u0000\u020d"+
		"\u020f\u0005\t\u0000\u0000\u020e\u020d\u0001\u0000\u0000\u0000\u020e\u020f"+
		"\u0001\u0000\u0000\u0000\u020f\u0210\u0001\u0000\u0000\u0000\u0210\u0211"+
		"\u0003\u00aaU\u0000\u0211\u0212\u0003\u00b2Y\u0000\u0212\u0214\u0003\u00b4"+
		"Z\u0000\u0213\u0215\u0005\n\u0000\u0000\u0214\u0213\u0001\u0000\u0000"+
		"\u0000\u0214\u0215\u0001\u0000\u0000\u0000\u0215\u0216\u0001\u0000\u0000"+
		"\u0000\u0216\u0217\u00054\u0000\u0000\u0217?\u0001\u0000\u0000\u0000\u0218"+
		"\u0219\u0005\u001d\u0000\u0000\u0219\u021d\u0005=\u0000\u0000\u021a\u021c"+
		"\u0003B!\u0000\u021b\u021a\u0001\u0000\u0000\u0000\u021c\u021f\u0001\u0000"+
		"\u0000\u0000\u021d\u021b\u0001\u0000\u0000\u0000\u021d\u021e\u0001\u0000"+
		"\u0000\u0000\u021e\u0220\u0001\u0000\u0000\u0000\u021f\u021d\u0001\u0000"+
		"\u0000\u0000\u0220\u0221\u0005>\u0000\u0000\u0221\u0222\u00054\u0000\u0000"+
		"\u0222A\u0001\u0000\u0000\u0000\u0223\u0224\u00030\u0018\u0000\u0224\u0225"+
		"\u0005:\u0000\u0000\u0225\u0226\u00038\u001c\u0000\u0226\u0227\u0005;"+
		"\u0000\u0000\u0227\u0228\u00054\u0000\u0000\u0228C\u0001\u0000\u0000\u0000"+
		"\u0229\u022a\u0005\u0017\u0000\u0000\u022a\u022e\u0005=\u0000\u0000\u022b"+
		"\u022d\u0003F#\u0000\u022c\u022b\u0001\u0000\u0000\u0000\u022d\u0230\u0001"+
		"\u0000\u0000\u0000\u022e\u022c\u0001\u0000\u0000\u0000\u022e\u022f\u0001"+
		"\u0000\u0000\u0000\u022f\u0231\u0001\u0000\u0000\u0000\u0230\u022e\u0001"+
		"\u0000\u0000\u0000\u0231\u0232\u0005>\u0000\u0000\u0232\u0233\u00054\u0000"+
		"\u0000\u0233E\u0001\u0000\u0000\u0000\u0234\u0235\u0005\u001a\u0000\u0000"+
		"\u0235\u0236\u00030\u0018\u0000\u0236\u0237\u0005=\u0000\u0000\u0237\u023b"+
		"\u0003J%\u0000\u0238\u023a\u0003H$\u0000\u0239\u0238\u0001\u0000\u0000"+
		"\u0000\u023a\u023d\u0001\u0000\u0000\u0000\u023b\u0239\u0001\u0000\u0000"+
		"\u0000\u023b\u023c\u0001\u0000\u0000\u0000\u023c\u023e\u0001\u0000\u0000"+
		"\u0000\u023d\u023b\u0001\u0000\u0000\u0000\u023e\u023f\u0005>\u0000\u0000"+
		"\u023f\u0240\u00054\u0000\u0000\u0240G\u0001\u0000\u0000\u0000\u0241\u0242"+
		"\u0005\u0018\u0000\u0000\u0242\u0243\u00030\u0018\u0000\u0243\u0244\u0005"+
		"=\u0000\u0000\u0244\u0245\u0003J%\u0000\u0245\u0246\u0005>\u0000\u0000"+
		"\u0246\u0247\u00054\u0000\u0000\u0247I\u0001\u0000\u0000\u0000\u0248\u0249"+
		"\u0005\u0019\u0000\u0000\u0249\u024e\u0005=\u0000\u0000\u024a\u024d\u0003"+
		"\u0010\b\u0000\u024b\u024d\u0003&\u0013\u0000\u024c\u024a\u0001\u0000"+
		"\u0000\u0000\u024c\u024b\u0001\u0000\u0000\u0000\u024d\u0250\u0001\u0000"+
		"\u0000\u0000\u024e\u024c\u0001\u0000\u0000\u0000\u024e\u024f\u0001\u0000"+
		"\u0000\u0000\u024f\u0251\u0001\u0000\u0000\u0000\u0250\u024e\u0001\u0000"+
		"\u0000\u0000\u0251\u0252\u0005>\u0000\u0000\u0252\u0253\u00054\u0000\u0000"+
		"\u0253K\u0001\u0000\u0000\u0000\u0254\u0255\u0005\r\u0000\u0000\u0255"+
		"\u0259\u0005=\u0000\u0000\u0256\u0258\u0003N\'\u0000\u0257\u0256\u0001"+
		"\u0000\u0000\u0000\u0258\u025b\u0001\u0000\u0000\u0000\u0259\u0257\u0001"+
		"\u0000\u0000\u0000\u0259\u025a\u0001\u0000\u0000\u0000\u025a\u025c\u0001"+
		"\u0000\u0000\u0000\u025b\u0259\u0001\u0000\u0000\u0000\u025c\u025d\u0005"+
		">\u0000\u0000\u025d\u025e\u00054\u0000\u0000\u025eM\u0001\u0000\u0000"+
		"\u0000\u025f\u0260\u0005\u000e\u0000\u0000\u0260\u0261\u00030\u0018\u0000"+
		"\u0261\u0265\u0005:\u0000\u0000\u0262\u0264\u0003\u00b6[\u0000\u0263\u0262"+
		"\u0001\u0000\u0000\u0000\u0264\u0267\u0001\u0000\u0000\u0000\u0265\u0263"+
		"\u0001\u0000\u0000\u0000\u0265\u0266\u0001\u0000\u0000\u0000\u0266\u026c"+
		"\u0001\u0000\u0000\u0000\u0267\u0265\u0001\u0000\u0000\u0000\u0268\u0269"+
		"\u0005@\u0000\u0000\u0269\u026b\u0003\u00b6[\u0000\u026a\u0268\u0001\u0000"+
		"\u0000\u0000\u026b\u026e\u0001\u0000\u0000\u0000\u026c\u026a\u0001\u0000"+
		"\u0000\u0000\u026c\u026d\u0001\u0000\u0000\u0000\u026d\u026f\u0001\u0000"+
		"\u0000\u0000\u026e\u026c\u0001\u0000\u0000\u0000\u026f\u0270\u0005;\u0000"+
		"\u0000\u0270\u0272\u0005=\u0000\u0000\u0271\u0273\u0003P(\u0000\u0272"+
		"\u0271\u0001\u0000\u0000\u0000\u0272\u0273\u0001\u0000\u0000\u0000\u0273"+
		"\u0275\u0001\u0000\u0000\u0000\u0274\u0276\u0003R)\u0000\u0275\u0274\u0001"+
		"\u0000\u0000\u0000\u0275\u0276\u0001\u0000\u0000\u0000\u0276\u0278\u0001"+
		"\u0000\u0000\u0000\u0277\u0279\u0003T*\u0000\u0278\u0277\u0001\u0000\u0000"+
		"\u0000\u0278\u0279\u0001\u0000\u0000\u0000\u0279\u027b\u0001\u0000\u0000"+
		"\u0000\u027a\u027c\u0003 \u0010\u0000\u027b\u027a\u0001\u0000\u0000\u0000"+
		"\u027b\u027c\u0001\u0000\u0000\u0000\u027c\u027e\u0001\u0000\u0000\u0000"+
		"\u027d\u027f\u0003\"\u0011\u0000\u027e\u027d\u0001\u0000\u0000\u0000\u027e"+
		"\u027f\u0001\u0000\u0000\u0000\u027f\u0281\u0001\u0000\u0000\u0000\u0280"+
		"\u0282\u0003@ \u0000\u0281\u0280\u0001\u0000\u0000\u0000\u0281\u0282\u0001"+
		"\u0000\u0000\u0000\u0282\u0283\u0001\u0000\u0000\u0000\u0283\u0284\u0005"+
		">\u0000\u0000\u0284\u0285\u00054\u0000\u0000\u0285O\u0001\u0000\u0000"+
		"\u0000\u0286\u0287\u0005\u001b\u0000\u0000\u0287\u028c\u0005=\u0000\u0000"+
		"\u0288\u028b\u0003&\u0013\u0000\u0289\u028b\u0003\u0010\b\u0000\u028a"+
		"\u0288\u0001\u0000\u0000\u0000\u028a\u0289\u0001\u0000\u0000\u0000\u028b"+
		"\u028e\u0001\u0000\u0000\u0000\u028c\u028a\u0001\u0000\u0000\u0000\u028c"+
		"\u028d\u0001\u0000\u0000\u0000\u028d\u028f\u0001\u0000\u0000\u0000\u028e"+
		"\u028c\u0001\u0000\u0000\u0000\u028f\u0290\u0005>\u0000\u0000\u0290\u0291"+
		"\u00054\u0000\u0000\u0291Q\u0001\u0000\u0000\u0000\u0292\u0293\u0005\u000f"+
		"\u0000\u0000\u0293\u0297\u0005=\u0000\u0000\u0294\u0296\u0003&\u0013\u0000"+
		"\u0295\u0294\u0001\u0000\u0000\u0000\u0296\u0299\u0001\u0000\u0000\u0000"+
		"\u0297\u0295\u0001\u0000\u0000\u0000\u0297\u0298\u0001\u0000\u0000\u0000"+
		"\u0298\u029a\u0001\u0000\u0000\u0000\u0299\u0297\u0001\u0000\u0000\u0000"+
		"\u029a\u029b\u0005>\u0000\u0000\u029b\u029c\u00054\u0000\u0000\u029cS"+
		"\u0001\u0000\u0000\u0000\u029d\u029e\u0005\u001c\u0000\u0000\u029e\u029f"+
		"\u0005=\u0000\u0000\u029f\u02a3\u0003V+\u0000\u02a0\u02a2\u0003V+\u0000"+
		"\u02a1\u02a0\u0001\u0000\u0000\u0000\u02a2\u02a5\u0001\u0000\u0000\u0000"+
		"\u02a3\u02a1\u0001\u0000\u0000\u0000\u02a3\u02a4\u0001\u0000\u0000\u0000"+
		"\u02a4\u02a6\u0001\u0000\u0000\u0000\u02a5\u02a3\u0001\u0000\u0000\u0000"+
		"\u02a6\u02a7\u0005>\u0000\u0000\u02a7\u02a8\u00054\u0000\u0000\u02a8U"+
		"\u0001\u0000\u0000\u0000\u02a9\u02aa\u0003\u00a4R\u0000\u02aa\u02ab\u0005"+
		"6\u0000\u0000\u02ab\u02af\u0005=\u0000\u0000\u02ac\u02ae\u0003\u000e\u0007"+
		"\u0000\u02ad\u02ac\u0001\u0000\u0000\u0000\u02ae\u02b1\u0001\u0000\u0000"+
		"\u0000\u02af\u02ad\u0001\u0000\u0000\u0000\u02af\u02b0\u0001\u0000\u0000"+
		"\u0000\u02b0\u02b5\u0001\u0000\u0000\u0000\u02b1\u02af\u0001\u0000\u0000"+
		"\u0000\u02b2\u02b4\u0003X,\u0000\u02b3\u02b2\u0001\u0000\u0000\u0000\u02b4"+
		"\u02b7\u0001\u0000\u0000\u0000\u02b5\u02b3\u0001\u0000\u0000\u0000\u02b5"+
		"\u02b6\u0001\u0000\u0000\u0000\u02b6\u02b8\u0001\u0000\u0000\u0000\u02b7"+
		"\u02b5\u0001\u0000\u0000\u0000\u02b8\u02b9\u0005>\u0000\u0000\u02b9W\u0001"+
		"\u0000\u0000\u0000\u02ba\u02bb\u0005\u0010\u0000\u0000\u02bb\u02bf\u0005"+
		"=\u0000\u0000\u02bc\u02be\u0003Z-\u0000\u02bd\u02bc\u0001\u0000\u0000"+
		"\u0000\u02be\u02c1\u0001\u0000\u0000\u0000\u02bf\u02bd\u0001\u0000\u0000"+
		"\u0000\u02bf\u02c0\u0001\u0000\u0000\u0000\u02c0\u02c2\u0001\u0000\u0000"+
		"\u0000\u02c1\u02bf\u0001\u0000\u0000\u0000\u02c2\u02c3\u0005>\u0000\u0000"+
		"\u02c3\u02c4\u00054\u0000\u0000\u02c4Y\u0001\u0000\u0000\u0000\u02c5\u02c6"+
		"\u0003\u00a6S\u0000\u02c6\u02c7\u00056\u0000\u0000\u02c7\u02c8\u00030"+
		"\u0018\u0000\u02c8\u02cc\u0005:\u0000\u0000\u02c9\u02cb\u0003.\u0017\u0000"+
		"\u02ca\u02c9\u0001\u0000\u0000\u0000\u02cb\u02ce\u0001\u0000\u0000\u0000"+
		"\u02cc\u02ca\u0001\u0000\u0000\u0000\u02cc\u02cd\u0001\u0000\u0000\u0000"+
		"\u02cd\u02d3\u0001\u0000\u0000\u0000\u02ce\u02cc\u0001\u0000\u0000\u0000"+
		"\u02cf\u02d0\u0005@\u0000\u0000\u02d0\u02d2\u0003.\u0017\u0000\u02d1\u02cf"+
		"\u0001\u0000\u0000\u0000\u02d2\u02d5\u0001\u0000\u0000\u0000\u02d3\u02d1"+
		"\u0001\u0000\u0000\u0000\u02d3\u02d4\u0001\u0000\u0000\u0000\u02d4\u02d6"+
		"\u0001\u0000\u0000\u0000\u02d5\u02d3\u0001\u0000\u0000\u0000\u02d6\u02dc"+
		"\u0005;\u0000\u0000\u02d7\u02d8\u0003\u00c0`\u0000\u02d8\u02d9\u0003\u00a6"+
		"S\u0000\u02d9\u02db\u0001\u0000\u0000\u0000\u02da\u02d7\u0001\u0000\u0000"+
		"\u0000\u02db\u02de\u0001\u0000\u0000\u0000\u02dc\u02da\u0001\u0000\u0000"+
		"\u0000\u02dc\u02dd\u0001\u0000\u0000\u0000\u02dd\u02df\u0001\u0000\u0000"+
		"\u0000\u02de\u02dc\u0001\u0000\u0000\u0000\u02df\u02e0\u00054\u0000\u0000"+
		"\u02e0[\u0001\u0000\u0000\u0000\u02e1\u02e2\u0005 \u0000\u0000\u02e2\u02e6"+
		"\u0005=\u0000\u0000\u02e3\u02e5\u0003\u0010\b\u0000\u02e4\u02e3\u0001"+
		"\u0000\u0000\u0000\u02e5\u02e8\u0001\u0000\u0000\u0000\u02e6\u02e4\u0001"+
		"\u0000\u0000\u0000\u02e6\u02e7\u0001\u0000\u0000\u0000\u02e7\u02e9\u0001"+
		"\u0000\u0000\u0000\u02e8\u02e6\u0001\u0000\u0000\u0000\u02e9\u02ea\u0005"+
		">\u0000\u0000\u02ea\u02eb\u00054\u0000\u0000\u02eb]\u0001\u0000\u0000"+
		"\u0000\u02ec\u02ed\u0005\"\u0000\u0000\u02ed\u02ee\u0005=\u0000\u0000"+
		"\u02ee\u02ef\u0003\u00a8T\u0000\u02ef\u02f0\u00054\u0000\u0000\u02f0\u02f1"+
		"\u0005>\u0000\u0000\u02f1\u02f2\u00054\u0000\u0000\u02f2_\u0001\u0000"+
		"\u0000\u0000\u02f3\u02f4\u0005!\u0000\u0000\u02f4\u02f9\u0005=\u0000\u0000"+
		"\u02f5\u02f8\u0003\u0010\b\u0000\u02f6\u02f8\u0003&\u0013\u0000\u02f7"+
		"\u02f5\u0001\u0000\u0000\u0000\u02f7\u02f6\u0001\u0000\u0000\u0000\u02f8"+
		"\u02fb\u0001\u0000\u0000\u0000\u02f9\u02f7\u0001\u0000\u0000\u0000\u02f9"+
		"\u02fa\u0001\u0000\u0000\u0000\u02fa\u02fc\u0001\u0000\u0000\u0000\u02fb"+
		"\u02f9\u0001\u0000\u0000\u0000\u02fc\u02fd\u0005>\u0000\u0000\u02fd\u02fe"+
		"\u00054\u0000\u0000\u02fea\u0001\u0000\u0000\u0000\u02ff\u0300\u0005\u001e"+
		"\u0000\u0000\u0300\u0304\u0005=\u0000\u0000\u0301\u0303\u0003d2\u0000"+
		"\u0302\u0301\u0001\u0000\u0000\u0000\u0303\u0306\u0001\u0000\u0000\u0000"+
		"\u0304\u0302\u0001\u0000\u0000\u0000\u0304\u0305\u0001\u0000\u0000\u0000"+
		"\u0305\u0307\u0001\u0000\u0000\u0000\u0306\u0304\u0001\u0000\u0000\u0000"+
		"\u0307\u0308\u0005>\u0000\u0000\u0308\u0309\u00054\u0000\u0000\u0309c"+
		"\u0001\u0000\u0000\u0000\u030a\u030b\u0005\u001f\u0000\u0000\u030b\u030c"+
		"\u00030\u0018\u0000\u030c\u0312\u0005=\u0000\u0000\u030d\u0311\u0003\\"+
		".\u0000\u030e\u0311\u0003^/\u0000\u030f\u0311\u0003`0\u0000\u0310\u030d"+
		"\u0001\u0000\u0000\u0000\u0310\u030e\u0001\u0000\u0000\u0000\u0310\u030f"+
		"\u0001\u0000\u0000\u0000\u0311\u0314\u0001\u0000\u0000\u0000\u0312\u0310"+
		"\u0001\u0000\u0000\u0000\u0312\u0313\u0001\u0000\u0000\u0000\u0313\u0315"+
		"\u0001\u0000\u0000\u0000\u0314\u0312\u0001\u0000\u0000\u0000\u0315\u0316"+
		"\u0005>\u0000\u0000\u0316\u0317\u00054\u0000\u0000\u0317e\u0001\u0000"+
		"\u0000\u0000\u0318\u0319\u0005G\u0000\u0000\u0319\u031d\u0005=\u0000\u0000"+
		"\u031a\u031c\u0003h4\u0000\u031b\u031a\u0001\u0000\u0000\u0000\u031c\u031f"+
		"\u0001\u0000\u0000\u0000\u031d\u031b\u0001\u0000\u0000\u0000\u031d\u031e"+
		"\u0001\u0000\u0000\u0000\u031e\u0320\u0001\u0000\u0000\u0000\u031f\u031d"+
		"\u0001\u0000\u0000\u0000\u0320\u0321\u0005>\u0000\u0000\u0321\u0322\u0005"+
		"4\u0000\u0000\u0322g\u0001\u0000\u0000\u0000\u0323\u0324\u0005H\u0000"+
		"\u0000\u0324\u0325\u00030\u0018\u0000\u0325\u032e\u0005=\u0000\u0000\u0326"+
		"\u032d\u0003r9\u0000\u0327\u032d\u0003\u009eO\u0000\u0328\u032d\u0003"+
		"j5\u0000\u0329\u032d\u0003|>\u0000\u032a\u032d\u0003t:\u0000\u032b\u032d"+
		"\u0003x<\u0000\u032c\u0326\u0001\u0000\u0000\u0000\u032c\u0327\u0001\u0000"+
		"\u0000\u0000\u032c\u0328\u0001\u0000\u0000\u0000\u032c\u0329\u0001\u0000"+
		"\u0000\u0000\u032c\u032a\u0001\u0000\u0000\u0000\u032c\u032b\u0001\u0000"+
		"\u0000\u0000\u032d\u0330\u0001\u0000\u0000\u0000\u032e\u032c\u0001\u0000"+
		"\u0000\u0000\u032e\u032f\u0001\u0000\u0000\u0000\u032f\u0331\u0001\u0000"+
		"\u0000\u0000\u0330\u032e\u0001\u0000\u0000\u0000\u0331\u0332\u0005>\u0000"+
		"\u0000\u0332\u0333\u00054\u0000\u0000\u0333i\u0001\u0000\u0000\u0000\u0334"+
		"\u0335\u0005I\u0000\u0000\u0335\u0339\u0005=\u0000\u0000\u0336\u0338\u0003"+
		"p8\u0000\u0337\u0336\u0001\u0000\u0000\u0000\u0338\u033b\u0001\u0000\u0000"+
		"\u0000\u0339\u0337\u0001\u0000\u0000\u0000\u0339\u033a\u0001\u0000\u0000"+
		"\u0000\u033a\u033c\u0001\u0000\u0000\u0000\u033b\u0339\u0001\u0000\u0000"+
		"\u0000\u033c\u033d\u0005>\u0000\u0000\u033d\u033e\u00054\u0000\u0000\u033e"+
		"k\u0001\u0000\u0000\u0000\u033f\u0340\u0005J\u0000\u0000\u0340\u0344\u0005"+
		"=\u0000\u0000\u0341\u0343\u0003n7\u0000\u0342\u0341\u0001\u0000\u0000"+
		"\u0000\u0343\u0346\u0001\u0000\u0000\u0000\u0344\u0342\u0001\u0000\u0000"+
		"\u0000\u0344\u0345\u0001\u0000\u0000\u0000\u0345\u0347\u0001\u0000\u0000"+
		"\u0000\u0346\u0344\u0001\u0000\u0000\u0000\u0347\u0348\u0005>\u0000\u0000"+
		"\u0348\u0349\u00054\u0000\u0000\u0349m\u0001\u0000\u0000\u0000\u034a\u034b"+
		"\u00051\u0000\u0000\u034b\u034c\u00030\u0018\u0000\u034c\u034d\u00054"+
		"\u0000\u0000\u034do\u0001\u0000\u0000\u0000\u034e\u034f\u00051\u0000\u0000"+
		"\u034f\u0350\u00030\u0018\u0000\u0350\u0351\u00054\u0000\u0000\u0351q"+
		"\u0001\u0000\u0000\u0000\u0352\u0353\u0005\u0007\u0000\u0000\u0353\u0354"+
		"\u0005=\u0000\u0000\u0354\u0355\u0005\u0001\u0000\u0000\u0355\u0356\u0003"+
		"\u00c2a\u0000\u0356\u0357\u0005\u0001\u0000\u0000\u0357\u0358\u0005>\u0000"+
		"\u0000\u0358\u0359\u00054\u0000\u0000\u0359s\u0001\u0000\u0000\u0000\u035a"+
		"\u035b\u0005K\u0000\u0000\u035b\u035f\u0005=\u0000\u0000\u035c\u035e\u0003"+
		"v;\u0000\u035d\u035c\u0001\u0000\u0000\u0000\u035e\u0361\u0001\u0000\u0000"+
		"\u0000\u035f\u035d\u0001\u0000\u0000\u0000\u035f\u0360\u0001\u0000\u0000"+
		"\u0000\u0360\u0362\u0001\u0000\u0000\u0000\u0361\u035f\u0001\u0000\u0000"+
		"\u0000\u0362\u0363\u0005>\u0000\u0000\u0363\u0364\u00054\u0000\u0000\u0364"+
		"u\u0001\u0000\u0000\u0000\u0365\u0366\u00051\u0000\u0000\u0366\u0367\u0003"+
		"\u00b4Z\u0000\u0367\u0368\u00054\u0000\u0000\u0368w\u0001\u0000\u0000"+
		"\u0000\u0369\u036a\u0005L\u0000\u0000\u036a\u036c\u0005=\u0000\u0000\u036b"+
		"\u036d\u0003z=\u0000\u036c\u036b\u0001\u0000\u0000\u0000\u036d\u036e\u0001"+
		"\u0000\u0000\u0000\u036e\u036c\u0001\u0000\u0000\u0000\u036e\u036f\u0001"+
		"\u0000\u0000\u0000\u036f\u0370\u0001\u0000\u0000\u0000\u0370\u0371\u0005"+
		">\u0000\u0000\u0371\u0372\u00054\u0000\u0000\u0372y\u0001\u0000\u0000"+
		"\u0000\u0373\u0374\u00051\u0000\u0000\u0374\u0375\u0005\u0001\u0000\u0000"+
		"\u0375\u0376\u0003\u00c2a\u0000\u0376\u0377\u0005\u0001\u0000\u0000\u0377"+
		"{\u0001\u0000\u0000\u0000\u0378\u0379\u0005J\u0000\u0000\u0379\u037d\u0005"+
		"=\u0000\u0000\u037a\u037c\u0003~?\u0000\u037b\u037a\u0001\u0000\u0000"+
		"\u0000\u037c\u037f\u0001\u0000\u0000\u0000\u037d\u037b\u0001\u0000\u0000"+
		"\u0000\u037d\u037e\u0001\u0000\u0000\u0000\u037e\u0380\u0001\u0000\u0000"+
		"\u0000\u037f\u037d\u0001\u0000\u0000\u0000\u0380\u0381\u0005>\u0000\u0000"+
		"\u0381\u0382\u00054\u0000\u0000\u0382}\u0001\u0000\u0000\u0000\u0383\u0384"+
		"\u00051\u0000\u0000\u0384\u038f\u00030\u0018\u0000\u0385\u0386\u0005="+
		"\u0000\u0000\u0386\u0388\u0003J%\u0000\u0387\u0389\u0003\u0086C\u0000"+
		"\u0388\u0387\u0001\u0000\u0000\u0000\u0388\u0389\u0001\u0000\u0000\u0000"+
		"\u0389\u038b\u0001\u0000\u0000\u0000\u038a\u038c\u0003\u0084B\u0000\u038b"+
		"\u038a\u0001\u0000\u0000\u0000\u038b\u038c\u0001\u0000\u0000\u0000\u038c"+
		"\u038d\u0001\u0000\u0000\u0000\u038d\u038e\u0005>\u0000\u0000\u038e\u0390"+
		"\u0001\u0000\u0000\u0000\u038f\u0385\u0001\u0000\u0000\u0000\u038f\u0390"+
		"\u0001\u0000\u0000\u0000\u0390\u0391\u0001\u0000\u0000\u0000\u0391\u0392"+
		"\u00054\u0000\u0000\u0392\u007f\u0001\u0000\u0000\u0000\u0393\u0394\u0005"+
		"J\u0000\u0000\u0394\u0398\u0005=\u0000\u0000\u0395\u0397\u0003\u0082A"+
		"\u0000\u0396\u0395\u0001\u0000\u0000\u0000\u0397\u039a\u0001\u0000\u0000"+
		"\u0000\u0398\u0396\u0001\u0000\u0000\u0000\u0398\u0399\u0001\u0000\u0000"+
		"\u0000\u0399\u039b\u0001\u0000\u0000\u0000\u039a\u0398\u0001\u0000\u0000"+
		"\u0000\u039b\u039c\u0005>\u0000\u0000\u039c\u039d\u00054\u0000\u0000\u039d"+
		"\u0081\u0001\u0000\u0000\u0000\u039e\u039f\u00051\u0000\u0000\u039f\u03a8"+
		"\u00030\u0018\u0000\u03a0\u03a2\u0005=\u0000\u0000\u03a1\u03a3\u0003\u0086"+
		"C\u0000\u03a2\u03a1\u0001\u0000\u0000\u0000\u03a2\u03a3\u0001\u0000\u0000"+
		"\u0000\u03a3\u03a5\u0001\u0000\u0000\u0000\u03a4\u03a6\u0003\u0084B\u0000"+
		"\u03a5\u03a4\u0001\u0000\u0000\u0000\u03a5\u03a6\u0001\u0000\u0000\u0000"+
		"\u03a6\u03a7\u0001\u0000\u0000\u0000\u03a7\u03a9\u0005>\u0000\u0000\u03a8"+
		"\u03a0\u0001\u0000\u0000\u0000\u03a8\u03a9\u0001\u0000\u0000\u0000\u03a9"+
		"\u03aa\u0001\u0000\u0000\u0000\u03aa\u03ab\u00054\u0000\u0000\u03ab\u0083"+
		"\u0001\u0000\u0000\u0000\u03ac\u03ad\u0005\u0017\u0000\u0000\u03ad\u03ae"+
		"\u00056\u0000\u0000\u03ae\u03b2\u0005=\u0000\u0000\u03af\u03b1\u0003\u0090"+
		"H\u0000\u03b0\u03af\u0001\u0000\u0000\u0000\u03b1\u03b4\u0001\u0000\u0000"+
		"\u0000\u03b2\u03b0\u0001\u0000\u0000\u0000\u03b2\u03b3\u0001\u0000\u0000"+
		"\u0000\u03b3\u03b5\u0001\u0000\u0000\u0000\u03b4\u03b2\u0001\u0000\u0000"+
		"\u0000\u03b5\u03b6\u0005>\u0000\u0000\u03b6\u0085\u0001\u0000\u0000\u0000"+
		"\u03b7\u03b8\u0005M\u0000\u0000\u03b8\u03b9\u00056\u0000\u0000\u03b9\u03bd"+
		"\u0005=\u0000\u0000\u03ba\u03bc\u0003\u0088D\u0000\u03bb\u03ba\u0001\u0000"+
		"\u0000\u0000\u03bc\u03bf\u0001\u0000\u0000\u0000\u03bd\u03bb\u0001\u0000"+
		"\u0000\u0000\u03bd\u03be\u0001\u0000\u0000\u0000\u03be\u03c0\u0001\u0000"+
		"\u0000\u0000\u03bf\u03bd\u0001\u0000\u0000\u0000\u03c0\u03c1\u0005>\u0000"+
		"\u0000\u03c1\u03c2\u00054\u0000\u0000\u03c2\u0087\u0001\u0000\u0000\u0000"+
		"\u03c3\u03c4\u0005N\u0000\u0000\u03c4\u03c5\u00030\u0018\u0000\u03c5\u03cb"+
		"\u0005=\u0000\u0000\u03c6\u03ca\u0003\u008aE\u0000\u03c7\u03ca\u0003\u008e"+
		"G\u0000\u03c8\u03ca\u0003J%\u0000\u03c9\u03c6\u0001\u0000\u0000\u0000"+
		"\u03c9\u03c7\u0001\u0000\u0000\u0000\u03c9\u03c8\u0001\u0000\u0000\u0000"+
		"\u03ca\u03cd\u0001\u0000\u0000\u0000\u03cb\u03c9\u0001\u0000\u0000\u0000"+
		"\u03cb\u03cc\u0001\u0000\u0000\u0000\u03cc\u03ce\u0001\u0000\u0000\u0000"+
		"\u03cd\u03cb\u0001\u0000\u0000\u0000\u03ce\u03cf\u0005>\u0000\u0000\u03cf"+
		"\u03d0\u00054\u0000\u0000\u03d0\u0089\u0001\u0000\u0000\u0000\u03d1\u03d2"+
		"\u0005O\u0000\u0000\u03d2\u03d3\u00056\u0000\u0000\u03d3\u03d4\u0005P"+
		"\u0000\u0000\u03d4\u03d5\u0003\u008cF\u0000\u03d5\u03d6\u0005Q\u0000\u0000"+
		"\u03d6\u03d7\u00054\u0000\u0000\u03d7\u008b\u0001\u0000\u0000\u0000\u03d8"+
		"\u03dd\u00030\u0018\u0000\u03d9\u03da\u0005@\u0000\u0000\u03da\u03dc\u0003"+
		"0\u0018\u0000\u03db\u03d9\u0001\u0000\u0000\u0000\u03dc\u03df\u0001\u0000"+
		"\u0000\u0000\u03dd\u03db\u0001\u0000\u0000\u0000\u03dd\u03de\u0001\u0000"+
		"\u0000\u0000\u03de\u008d\u0001\u0000\u0000\u0000\u03df\u03dd\u0001\u0000"+
		"\u0000\u0000\u03e0\u03e1\u0005\u0007\u0000\u0000\u03e1\u03e2\u00056\u0000"+
		"\u0000\u03e2\u03e3\u0005\u0001\u0000\u0000\u03e3\u03e4\u0003\u00c2a\u0000"+
		"\u03e4\u03e5\u0005\u0001\u0000\u0000\u03e5\u03e6\u00054\u0000\u0000\u03e6"+
		"\u008f\u0001\u0000\u0000\u0000\u03e7\u03e8\u0005\u0018\u0000\u0000\u03e8"+
		"\u03e9\u00030\u0018\u0000\u03e9\u03f0\u0005=\u0000\u0000\u03ea\u03ef\u0003"+
		"\u0092I\u0000\u03eb\u03ef\u0003\u0094J\u0000\u03ec\u03ef\u0003\u0098L"+
		"\u0000\u03ed\u03ef\u0003J%\u0000\u03ee\u03ea\u0001\u0000\u0000\u0000\u03ee"+
		"\u03eb\u0001\u0000\u0000\u0000\u03ee\u03ec\u0001\u0000\u0000\u0000\u03ee"+
		"\u03ed\u0001\u0000\u0000\u0000\u03ef\u03f2\u0001\u0000\u0000\u0000\u03f0"+
		"\u03ee\u0001\u0000\u0000\u0000\u03f0\u03f1\u0001\u0000\u0000\u0000\u03f1"+
		"\u03f3\u0001\u0000\u0000\u0000\u03f2\u03f0\u0001\u0000\u0000\u0000\u03f3"+
		"\u03f4\u0005>\u0000\u0000\u03f4\u03f5\u00054\u0000\u0000\u03f5\u0091\u0001"+
		"\u0000\u0000\u0000\u03f6\u03f7\u0005R\u0000\u0000\u03f7\u03f8\u00056\u0000"+
		"\u0000\u03f8\u03f9\u00030\u0018\u0000\u03f9\u03fa\u00054\u0000\u0000\u03fa"+
		"\u0093\u0001\u0000\u0000\u0000\u03fb\u03fc\u0005S\u0000\u0000\u03fc\u03fd"+
		"\u00056\u0000\u0000\u03fd\u03fe\u0005P\u0000\u0000\u03fe\u03ff\u0003\u0096"+
		"K\u0000\u03ff\u0400\u0005Q\u0000\u0000\u0400\u0401\u00054\u0000\u0000"+
		"\u0401\u0095\u0001\u0000\u0000\u0000\u0402\u0407\u00030\u0018\u0000\u0403"+
		"\u0404\u0005@\u0000\u0000\u0404\u0406\u00030\u0018\u0000\u0405\u0403\u0001"+
		"\u0000\u0000\u0000\u0406\u0409\u0001\u0000\u0000\u0000\u0407\u0405\u0001"+
		"\u0000\u0000\u0000\u0407\u0408\u0001\u0000\u0000\u0000\u0408\u0097\u0001"+
		"\u0000\u0000\u0000\u0409\u0407\u0001\u0000\u0000\u0000\u040a\u040b\u0005"+
		"\u0007\u0000\u0000\u040b\u040c\u00056\u0000\u0000\u040c\u040d\u0005\u0001"+
		"\u0000\u0000\u040d\u040e\u0003\u00c2a\u0000\u040e\u040f\u0005\u0001\u0000"+
		"\u0000\u040f\u0410\u00054\u0000\u0000\u0410\u0099\u0001\u0000\u0000\u0000"+
		"\u0411\u0412\u0005T\u0000\u0000\u0412\u0416\u0005=\u0000\u0000\u0413\u0415"+
		"\u0003\u009cN\u0000\u0414\u0413\u0001\u0000\u0000\u0000\u0415\u0418\u0001"+
		"\u0000\u0000\u0000\u0416\u0414\u0001\u0000\u0000\u0000\u0416\u0417\u0001"+
		"\u0000\u0000\u0000\u0417\u0419\u0001\u0000\u0000\u0000\u0418\u0416\u0001"+
		"\u0000\u0000\u0000\u0419\u041a\u0005>\u0000\u0000\u041a\u041b\u00054\u0000"+
		"\u0000\u041b\u009b\u0001\u0000\u0000\u0000\u041c\u041d\u0005U\u0000\u0000"+
		"\u041d\u041e\u00030\u0018\u0000\u041e\u0427\u0005=\u0000\u0000\u041f\u0426"+
		"\u0003r9\u0000\u0420\u0426\u0003\u009eO\u0000\u0421\u0426\u0003\u0080"+
		"@\u0000\u0422\u0426\u0003\u00a0P\u0000\u0423\u0426\u0003t:\u0000\u0424"+
		"\u0426\u0003x<\u0000\u0425\u041f\u0001\u0000\u0000\u0000\u0425\u0420\u0001"+
		"\u0000\u0000\u0000\u0425\u0421\u0001\u0000\u0000\u0000\u0425\u0422\u0001"+
		"\u0000\u0000\u0000\u0425\u0423\u0001\u0000\u0000\u0000\u0425\u0424\u0001"+
		"\u0000\u0000\u0000\u0426\u0429\u0001\u0000\u0000\u0000\u0427\u0425\u0001"+
		"\u0000\u0000\u0000\u0427\u0428\u0001\u0000\u0000\u0000\u0428\u042a\u0001"+
		"\u0000\u0000\u0000\u0429\u0427\u0001\u0000\u0000\u0000\u042a\u042b\u0005"+
		">\u0000\u0000\u042b\u042c\u00054\u0000\u0000\u042c\u009d\u0001\u0000\u0000"+
		"\u0000\u042d\u042e\u0005\u0019\u0000\u0000\u042e\u042f\u0005=\u0000\u0000"+
		"\u042f\u0430\u0003\u0010\b\u0000\u0430\u0431\u0005>\u0000\u0000\u0431"+
		"\u0432\u00054\u0000\u0000\u0432\u009f\u0001\u0000\u0000\u0000\u0433\u0434"+
		"\u0005V\u0000\u0000\u0434\u0438\u0005=\u0000\u0000\u0435\u0437\u0003\u00a2"+
		"Q\u0000\u0436\u0435\u0001\u0000\u0000\u0000\u0437\u043a\u0001\u0000\u0000"+
		"\u0000\u0438\u0436\u0001\u0000\u0000\u0000\u0438\u0439\u0001\u0000\u0000"+
		"\u0000\u0439\u043b\u0001\u0000\u0000\u0000\u043a\u0438\u0001\u0000\u0000"+
		"\u0000\u043b\u043c\u0005>\u0000\u0000\u043c\u043d\u00054\u0000\u0000\u043d"+
		"\u00a1\u0001\u0000\u0000\u0000\u043e\u043f\u00051\u0000\u0000\u043f\u0440"+
		"\u0005\u0001\u0000\u0000\u0440\u0441\u0003\u00c2a\u0000\u0441\u0442\u0005"+
		"\u0001\u0000\u0000\u0442\u00a3\u0001\u0000\u0000\u0000\u0443\u0444\u0007"+
		"\u0000\u0000\u0000\u0444\u00a5\u0001\u0000\u0000\u0000\u0445\u0446\u0005"+
		"E\u0000\u0000\u0446\u00a7\u0001\u0000\u0000\u0000\u0447\u0448\u0005E\u0000"+
		"\u0000\u0448\u00a9\u0001\u0000\u0000\u0000\u0449\u044d\u0003\u00acV\u0000"+
		"\u044a\u044d\u0003\u00aeW\u0000\u044b\u044d\u0003\u00b0X\u0000\u044c\u0449"+
		"\u0001\u0000\u0000\u0000\u044c\u044a\u0001\u0000\u0000\u0000\u044c\u044b"+
		"\u0001\u0000\u0000\u0000\u044d\u00ab\u0001\u0000\u0000\u0000\u044e\u044f"+
		"\u00057\u0000\u0000\u044f\u0450\u0005A\u0000\u0000\u0450\u00ad\u0001\u0000"+
		"\u0000\u0000\u0451\u0452\u0005A\u0000\u0000\u0452\u00af\u0001\u0000\u0000"+
		"\u0000\u0453\u0454\u00057\u0000\u0000\u0454\u0455\u00057\u0000\u0000\u0455"+
		"\u00b1\u0001\u0000\u0000\u0000\u0456\u0457\u0005A\u0000\u0000\u0457\u00b3"+
		"\u0001\u0000\u0000\u0000\u0458\u045c\u0003\u00acV\u0000\u0459\u045c\u0003"+
		"\u00aeW\u0000\u045a\u045c\u0003\u00b0X\u0000\u045b\u0458\u0001\u0000\u0000"+
		"\u0000\u045b\u0459\u0001\u0000\u0000\u0000\u045b\u045a\u0001\u0000\u0000"+
		"\u0000\u045c\u00b5\u0001\u0000\u0000\u0000\u045d\u045e\u0003\u00b8\\\u0000"+
		"\u045e\u045f\u0003\u00ba]\u0000\u045f\u00b7\u0001\u0000\u0000\u0000\u0460"+
		"\u0461\u0005A\u0000\u0000\u0461\u00b9\u0001\u0000\u0000\u0000\u0462\u0463"+
		"\u0005A\u0000\u0000\u0463\u00bb\u0001\u0000\u0000\u0000\u0464\u0465\u0007"+
		"\u0001\u0000\u0000\u0465\u00bd\u0001\u0000\u0000\u0000\u0466\u0468\t\u0000"+
		"\u0000\u0000\u0467\u0466\u0001\u0000\u0000\u0000\u0468\u046b\u0001\u0000"+
		"\u0000\u0000\u0469\u046a\u0001\u0000\u0000\u0000\u0469\u0467\u0001\u0000"+
		"\u0000\u0000\u046a\u046c\u0001\u0000\u0000\u0000\u046b\u0469\u0001\u0000"+
		"\u0000\u0000\u046c\u046d\u00054\u0000\u0000\u046d\u00bf\u0001\u0000\u0000"+
		"\u0000\u046e\u046f\u0007\u0002\u0000\u0000\u046f\u00c1\u0001\u0000\u0000"+
		"\u0000\u0470\u0473\u0003\u00c4b\u0000\u0471\u0473\u0005F\u0000\u0000\u0472"+
		"\u0470\u0001\u0000\u0000\u0000\u0472\u0471\u0001\u0000\u0000\u0000\u0473"+
		"\u0476\u0001\u0000\u0000\u0000\u0474\u0472\u0001\u0000\u0000\u0000\u0474"+
		"\u0475\u0001\u0000\u0000\u0000\u0475\u00c3\u0001\u0000\u0000\u0000\u0476"+
		"\u0474\u0001\u0000\u0000\u0000\u0477\u0478\u0007\u0003\u0000\u0000\u0478"+
		"\u00c5\u0001\u0000\u0000\u0000i\u00ce\u00d0\u00d9\u00e2\u00ed\u00f4\u00ff"+
		"\u010b\u0112\u0118\u011b\u0122\u0124\u012d\u012f\u0139\u013f\u014a\u0151"+
		"\u0157\u015e\u0168\u0178\u0191\u0198\u019e\u01a7\u01b4\u01bd\u01c3\u01ca"+
		"\u01d2\u01d6\u01dc\u01e3\u01e9\u01ed\u01f1\u01f5\u01fd\u0207\u020e\u0214"+
		"\u021d\u022e\u023b\u024c\u024e\u0259\u0265\u026c\u0272\u0275\u0278\u027b"+
		"\u027e\u0281\u028a\u028c\u0297\u02a3\u02af\u02b5\u02bf\u02cc\u02d3\u02dc"+
		"\u02e6\u02f7\u02f9\u0304\u0310\u0312\u031d\u032c\u032e\u0339\u0344\u035f"+
		"\u036e\u037d\u0388\u038b\u038f\u0398\u03a2\u03a5\u03a8\u03b2\u03bd\u03c9"+
		"\u03cb\u03dd\u03ee\u03f0\u0407\u0416\u0425\u0427\u0438\u044c\u045b\u0469"+
		"\u0472\u0474";
	public static final ATN _ATN =
		new ATNDeserializer().deserialize(_serializedATN.toCharArray());
	static {
		_decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
		for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
			_decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
		}
	}
}