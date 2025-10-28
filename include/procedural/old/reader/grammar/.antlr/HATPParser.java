// Generated from /home/avigne/Projets/Procedural/catkin_ws/src/Procedural/include/procedural/old/reader/grammar/HATPParser.g4 by ANTLR 4.13.1
import org.antlr.v4.runtime.atn.*;
import org.antlr.v4.runtime.dfa.DFA;
import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.misc.*;
import org.antlr.v4.runtime.tree.*;
import java.util.List;
import java.util.Iterator;
import java.util.ArrayList;

@SuppressWarnings({"all", "warnings", "unchecked", "unused", "cast", "CheckReturnValue"})
public class HATPParser extends Parser {
	static { RuntimeMetaData.checkVersion("4.13.1", RuntimeMetaData.VERSION); }

	protected static final DFA[] _decisionToDFA;
	protected static final PredictionContextCache _sharedContextCache =
		new PredictionContextCache();
	public static final int
		HTN=1, ACTION=2, PRECONDITIONS=3, EFFECTS=4, METHOD=5, GOAL=6, SUBTASK=7, 
		TIMEPART=8, FACTDATABASE=9, FORALL=10, COST=11, DURATION=12, SELECT=13, 
		WS=14, ADD_IN_SET=15, REMOVE_FROM_SET=16, EQUAL=17, TEST_EQUAL=18, TEST_DIFF=19, 
		TEST_SET_IN=20, TEST_SET_NOT_IN=21, SUP=22, SUP_EQUAL=23, INF=24, INF_EQUAL=25, 
		SUP_TILD=26, PLUS=27, MINUS=28, TIMES=29, SLASH=30, SEMICOLON=31, POINT=32, 
		STRING=33, COLON=34, QUESTIONMARK=35, COMMENT=36, LINE_COMMENT=37, OpenPar=38, 
		ClosePar=39, OpenClosePar=40, OpenCurly=41, CloseCurly=42, Comma=43, IDENTIFIER=44, 
		TYPE=45, VARNAME=46, ATTRIBUT=47, NUMBER=48;
	public static final int
		RULE_hatp = 0, RULE_timepart = 1, RULE_factbase = 2, RULE_htn = 3, RULE_actions = 4, 
		RULE_action_name = 5, RULE_preconditions = 6, RULE_effects = 7, RULE_arguments = 8, 
		RULE_type = 9, RULE_varname = 10, RULE_value = 11, RULE_methods = 12, 
		RULE_decomposition = 13, RULE_subtask = 14, RULE_goal = 15, RULE_comment = 16, 
		RULE_ignore = 17, RULE_attribut = 18, RULE_subject = 19, RULE_object = 20, 
		RULE_operator = 21, RULE_expression = 22, RULE_subselection = 23, RULE_selectcase = 24, 
		RULE_list = 25, RULE_function = 26, RULE_order = 27, RULE_forall = 28, 
		RULE_cost = 29, RULE_duration = 30;
	private static String[] makeRuleNames() {
		return new String[] {
			"hatp", "timepart", "factbase", "htn", "actions", "action_name", "preconditions", 
			"effects", "arguments", "type", "varname", "value", "methods", "decomposition", 
			"subtask", "goal", "comment", "ignore", "attribut", "subject", "object", 
			"operator", "expression", "subselection", "selectcase", "list", "function", 
			"order", "forall", "cost", "duration"
		};
	}
	public static final String[] ruleNames = makeRuleNames();

	private static String[] makeLiteralNames() {
		return new String[] {
			null, "'HTN'", "'action'", "'preconditions'", "'effects'", "'method'", 
			"'goal'", "'subtasks'", "'timePart'", "'factdatabase'", "'FORALL'", "'cost'", 
			"'duration'", "'SELECT'", null, "'<<='", "'=>>'", "'='", "'=='", "'!='", 
			"'>>'", "'!>>'", "'>'", "'>='", "'<'", "'<='", "'~>'", "'+'", "'-'", 
			"'*'", "'/'", "';'", "'.'", "'\"'", "':'", "'?'", null, null, "'('", 
			"')'", "'()'", "'{'", "'}'", "','"
		};
	}
	private static final String[] _LITERAL_NAMES = makeLiteralNames();
	private static String[] makeSymbolicNames() {
		return new String[] {
			null, "HTN", "ACTION", "PRECONDITIONS", "EFFECTS", "METHOD", "GOAL", 
			"SUBTASK", "TIMEPART", "FACTDATABASE", "FORALL", "COST", "DURATION", 
			"SELECT", "WS", "ADD_IN_SET", "REMOVE_FROM_SET", "EQUAL", "TEST_EQUAL", 
			"TEST_DIFF", "TEST_SET_IN", "TEST_SET_NOT_IN", "SUP", "SUP_EQUAL", "INF", 
			"INF_EQUAL", "SUP_TILD", "PLUS", "MINUS", "TIMES", "SLASH", "SEMICOLON", 
			"POINT", "STRING", "COLON", "QUESTIONMARK", "COMMENT", "LINE_COMMENT", 
			"OpenPar", "ClosePar", "OpenClosePar", "OpenCurly", "CloseCurly", "Comma", 
			"IDENTIFIER", "TYPE", "VARNAME", "ATTRIBUT", "NUMBER"
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
	public String getGrammarFileName() { return "HATPParser.g4"; }

	@Override
	public String[] getRuleNames() { return ruleNames; }

	@Override
	public String getSerializedATN() { return _serializedATN; }

	@Override
	public ATN getATN() { return _ATN; }

	public HATPParser(TokenStream input) {
		super(input);
		_interp = new ParserATNSimulator(this,_ATN,_decisionToDFA,_sharedContextCache);
	}

	@SuppressWarnings("CheckReturnValue")
	public static class HatpContext extends ParserRuleContext {
		public HtnContext htn() {
			return getRuleContext(HtnContext.class,0);
		}
		public TimepartContext timepart() {
			return getRuleContext(TimepartContext.class,0);
		}
		public TerminalNode EOF() { return getToken(HATPParser.EOF, 0); }
		public List<CommentContext> comment() {
			return getRuleContexts(CommentContext.class);
		}
		public CommentContext comment(int i) {
			return getRuleContext(CommentContext.class,i);
		}
		public List<FactbaseContext> factbase() {
			return getRuleContexts(FactbaseContext.class);
		}
		public FactbaseContext factbase(int i) {
			return getRuleContext(FactbaseContext.class,i);
		}
		public HatpContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_hatp; }
	}

	public final HatpContext hatp() throws RecognitionException {
		HatpContext _localctx = new HatpContext(_ctx, getState());
		enterRule(_localctx, 0, RULE_hatp);
		int _la;
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(65);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,0,_ctx);
			while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1 ) {
					{
					{
					setState(62);
					comment();
					}
					} 
				}
				setState(67);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,0,_ctx);
			}
			setState(71);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==FACTDATABASE) {
				{
				{
				setState(68);
				factbase();
				}
				}
				setState(73);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(77);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMENT || _la==LINE_COMMENT) {
				{
				{
				setState(74);
				comment();
				}
				}
				setState(79);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(80);
			htn();
			setState(84);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMENT || _la==LINE_COMMENT) {
				{
				{
				setState(81);
				comment();
				}
				}
				setState(86);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(87);
			timepart();
			setState(91);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMENT || _la==LINE_COMMENT) {
				{
				{
				setState(88);
				comment();
				}
				}
				setState(93);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(94);
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
	public static class TimepartContext extends ParserRuleContext {
		public TerminalNode TIMEPART() { return getToken(HATPParser.TIMEPART, 0); }
		public TerminalNode OpenCurly() { return getToken(HATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(HATPParser.CloseCurly, 0); }
		public List<IgnoreContext> ignore() {
			return getRuleContexts(IgnoreContext.class);
		}
		public IgnoreContext ignore(int i) {
			return getRuleContext(IgnoreContext.class,i);
		}
		public TimepartContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_timepart; }
	}

	public final TimepartContext timepart() throws RecognitionException {
		TimepartContext _localctx = new TimepartContext(_ctx, getState());
		enterRule(_localctx, 2, RULE_timepart);
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(96);
			match(TIMEPART);
			setState(97);
			match(OpenCurly);
			setState(101);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,5,_ctx);
			while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1 ) {
					{
					{
					setState(98);
					ignore();
					}
					} 
				}
				setState(103);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,5,_ctx);
			}
			setState(104);
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
	public static class FactbaseContext extends ParserRuleContext {
		public TerminalNode FACTDATABASE() { return getToken(HATPParser.FACTDATABASE, 0); }
		public TerminalNode OpenCurly() { return getToken(HATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(HATPParser.CloseCurly, 0); }
		public List<IgnoreContext> ignore() {
			return getRuleContexts(IgnoreContext.class);
		}
		public IgnoreContext ignore(int i) {
			return getRuleContext(IgnoreContext.class,i);
		}
		public FactbaseContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_factbase; }
	}

	public final FactbaseContext factbase() throws RecognitionException {
		FactbaseContext _localctx = new FactbaseContext(_ctx, getState());
		enterRule(_localctx, 4, RULE_factbase);
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(106);
			match(FACTDATABASE);
			setState(107);
			match(OpenCurly);
			setState(111);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,6,_ctx);
			while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1 ) {
					{
					{
					setState(108);
					ignore();
					}
					} 
				}
				setState(113);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,6,_ctx);
			}
			setState(114);
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
	public static class HtnContext extends ParserRuleContext {
		public TerminalNode HTN() { return getToken(HATPParser.HTN, 0); }
		public TerminalNode OpenCurly() { return getToken(HATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(HATPParser.CloseCurly, 0); }
		public List<ActionsContext> actions() {
			return getRuleContexts(ActionsContext.class);
		}
		public ActionsContext actions(int i) {
			return getRuleContext(ActionsContext.class,i);
		}
		public List<MethodsContext> methods() {
			return getRuleContexts(MethodsContext.class);
		}
		public MethodsContext methods(int i) {
			return getRuleContext(MethodsContext.class,i);
		}
		public HtnContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_htn; }
	}

	public final HtnContext htn() throws RecognitionException {
		HtnContext _localctx = new HtnContext(_ctx, getState());
		enterRule(_localctx, 6, RULE_htn);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(116);
			match(HTN);
			setState(117);
			match(OpenCurly);
			setState(121);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==ACTION) {
				{
				{
				setState(118);
				actions();
				}
				}
				setState(123);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(127);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==METHOD) {
				{
				{
				setState(124);
				methods();
				}
				}
				setState(129);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(130);
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
	public static class ActionsContext extends ParserRuleContext {
		public TerminalNode ACTION() { return getToken(HATPParser.ACTION, 0); }
		public Action_nameContext action_name() {
			return getRuleContext(Action_nameContext.class,0);
		}
		public TerminalNode OpenPar() { return getToken(HATPParser.OpenPar, 0); }
		public List<ArgumentsContext> arguments() {
			return getRuleContexts(ArgumentsContext.class);
		}
		public ArgumentsContext arguments(int i) {
			return getRuleContext(ArgumentsContext.class,i);
		}
		public TerminalNode ClosePar() { return getToken(HATPParser.ClosePar, 0); }
		public TerminalNode OpenCurly() { return getToken(HATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(HATPParser.CloseCurly, 0); }
		public List<TerminalNode> Comma() { return getTokens(HATPParser.Comma); }
		public TerminalNode Comma(int i) {
			return getToken(HATPParser.Comma, i);
		}
		public List<PreconditionsContext> preconditions() {
			return getRuleContexts(PreconditionsContext.class);
		}
		public PreconditionsContext preconditions(int i) {
			return getRuleContext(PreconditionsContext.class,i);
		}
		public List<EffectsContext> effects() {
			return getRuleContexts(EffectsContext.class);
		}
		public EffectsContext effects(int i) {
			return getRuleContext(EffectsContext.class,i);
		}
		public CostContext cost() {
			return getRuleContext(CostContext.class,0);
		}
		public DurationContext duration() {
			return getRuleContext(DurationContext.class,0);
		}
		public ActionsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_actions; }
	}

	public final ActionsContext actions() throws RecognitionException {
		ActionsContext _localctx = new ActionsContext(_ctx, getState());
		enterRule(_localctx, 8, RULE_actions);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(132);
			match(ACTION);
			setState(133);
			action_name();
			setState(134);
			match(OpenPar);
			setState(135);
			arguments();
			setState(140);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==Comma) {
				{
				{
				setState(136);
				match(Comma);
				setState(137);
				arguments();
				}
				}
				setState(142);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(143);
			match(ClosePar);
			setState(144);
			match(OpenCurly);
			setState(146); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(145);
				preconditions();
				}
				}
				setState(148); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( _la==PRECONDITIONS );
			setState(151); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(150);
				effects();
				}
				}
				setState(153); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( _la==EFFECTS );
			setState(156);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==COST) {
				{
				setState(155);
				cost();
				}
			}

			setState(159);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==DURATION) {
				{
				setState(158);
				duration();
				}
			}

			setState(161);
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
	public static class Action_nameContext extends ParserRuleContext {
		public TerminalNode IDENTIFIER() { return getToken(HATPParser.IDENTIFIER, 0); }
		public Action_nameContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_action_name; }
	}

	public final Action_nameContext action_name() throws RecognitionException {
		Action_nameContext _localctx = new Action_nameContext(_ctx, getState());
		enterRule(_localctx, 10, RULE_action_name);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(163);
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
	public static class PreconditionsContext extends ParserRuleContext {
		public TerminalNode PRECONDITIONS() { return getToken(HATPParser.PRECONDITIONS, 0); }
		public TerminalNode OpenCurly() { return getToken(HATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(HATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(HATPParser.SEMICOLON, 0); }
		public List<ExpressionContext> expression() {
			return getRuleContexts(ExpressionContext.class);
		}
		public ExpressionContext expression(int i) {
			return getRuleContext(ExpressionContext.class,i);
		}
		public PreconditionsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_preconditions; }
	}

	public final PreconditionsContext preconditions() throws RecognitionException {
		PreconditionsContext _localctx = new PreconditionsContext(_ctx, getState());
		enterRule(_localctx, 12, RULE_preconditions);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(165);
			match(PRECONDITIONS);
			setState(166);
			match(OpenCurly);
			setState(170);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==IDENTIFIER) {
				{
				{
				setState(167);
				expression();
				}
				}
				setState(172);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(173);
			match(CloseCurly);
			setState(174);
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
	public static class EffectsContext extends ParserRuleContext {
		public TerminalNode EFFECTS() { return getToken(HATPParser.EFFECTS, 0); }
		public TerminalNode OpenCurly() { return getToken(HATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(HATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(HATPParser.SEMICOLON, 0); }
		public List<ForallContext> forall() {
			return getRuleContexts(ForallContext.class);
		}
		public ForallContext forall(int i) {
			return getRuleContext(ForallContext.class,i);
		}
		public List<ExpressionContext> expression() {
			return getRuleContexts(ExpressionContext.class);
		}
		public ExpressionContext expression(int i) {
			return getRuleContext(ExpressionContext.class,i);
		}
		public EffectsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_effects; }
	}

	public final EffectsContext effects() throws RecognitionException {
		EffectsContext _localctx = new EffectsContext(_ctx, getState());
		enterRule(_localctx, 14, RULE_effects);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(176);
			match(EFFECTS);
			setState(177);
			match(OpenCurly);
			setState(182);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==FORALL || _la==IDENTIFIER) {
				{
				setState(180);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case FORALL:
					{
					setState(178);
					forall();
					}
					break;
				case IDENTIFIER:
					{
					setState(179);
					expression();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(184);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(185);
			match(CloseCurly);
			setState(186);
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
		enterRule(_localctx, 16, RULE_arguments);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(188);
			type();
			setState(189);
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
		public TerminalNode IDENTIFIER() { return getToken(HATPParser.IDENTIFIER, 0); }
		public TypeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_type; }
	}

	public final TypeContext type() throws RecognitionException {
		TypeContext _localctx = new TypeContext(_ctx, getState());
		enterRule(_localctx, 18, RULE_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(191);
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
		public TerminalNode IDENTIFIER() { return getToken(HATPParser.IDENTIFIER, 0); }
		public VarnameContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_varname; }
	}

	public final VarnameContext varname() throws RecognitionException {
		VarnameContext _localctx = new VarnameContext(_ctx, getState());
		enterRule(_localctx, 20, RULE_varname);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(193);
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
	public static class ValueContext extends ParserRuleContext {
		public TerminalNode IDENTIFIER() { return getToken(HATPParser.IDENTIFIER, 0); }
		public ValueContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value; }
	}

	public final ValueContext value() throws RecognitionException {
		ValueContext _localctx = new ValueContext(_ctx, getState());
		enterRule(_localctx, 22, RULE_value);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(195);
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
	public static class MethodsContext extends ParserRuleContext {
		public TerminalNode METHOD() { return getToken(HATPParser.METHOD, 0); }
		public TerminalNode IDENTIFIER() { return getToken(HATPParser.IDENTIFIER, 0); }
		public TerminalNode OpenPar() { return getToken(HATPParser.OpenPar, 0); }
		public List<ArgumentsContext> arguments() {
			return getRuleContexts(ArgumentsContext.class);
		}
		public ArgumentsContext arguments(int i) {
			return getRuleContext(ArgumentsContext.class,i);
		}
		public TerminalNode ClosePar() { return getToken(HATPParser.ClosePar, 0); }
		public TerminalNode OpenCurly() { return getToken(HATPParser.OpenCurly, 0); }
		public GoalContext goal() {
			return getRuleContext(GoalContext.class,0);
		}
		public TerminalNode CloseCurly() { return getToken(HATPParser.CloseCurly, 0); }
		public List<TerminalNode> Comma() { return getTokens(HATPParser.Comma); }
		public TerminalNode Comma(int i) {
			return getToken(HATPParser.Comma, i);
		}
		public List<DecompositionContext> decomposition() {
			return getRuleContexts(DecompositionContext.class);
		}
		public DecompositionContext decomposition(int i) {
			return getRuleContext(DecompositionContext.class,i);
		}
		public MethodsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_methods; }
	}

	public final MethodsContext methods() throws RecognitionException {
		MethodsContext _localctx = new MethodsContext(_ctx, getState());
		enterRule(_localctx, 24, RULE_methods);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(197);
			match(METHOD);
			setState(198);
			match(IDENTIFIER);
			setState(199);
			match(OpenPar);
			setState(200);
			arguments();
			setState(205);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==Comma) {
				{
				{
				setState(201);
				match(Comma);
				setState(202);
				arguments();
				}
				}
				setState(207);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(208);
			match(ClosePar);
			setState(209);
			match(OpenCurly);
			setState(210);
			goal();
			setState(212); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(211);
				decomposition();
				}
				}
				setState(214); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( _la==OpenCurly );
			setState(216);
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
	public static class DecompositionContext extends ParserRuleContext {
		public TerminalNode OpenCurly() { return getToken(HATPParser.OpenCurly, 0); }
		public PreconditionsContext preconditions() {
			return getRuleContext(PreconditionsContext.class,0);
		}
		public SubtaskContext subtask() {
			return getRuleContext(SubtaskContext.class,0);
		}
		public TerminalNode CloseCurly() { return getToken(HATPParser.CloseCurly, 0); }
		public DecompositionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_decomposition; }
	}

	public final DecompositionContext decomposition() throws RecognitionException {
		DecompositionContext _localctx = new DecompositionContext(_ctx, getState());
		enterRule(_localctx, 26, RULE_decomposition);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(218);
			match(OpenCurly);
			setState(219);
			preconditions();
			setState(220);
			subtask();
			setState(221);
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
	public static class SubtaskContext extends ParserRuleContext {
		public TerminalNode SUBTASK() { return getToken(HATPParser.SUBTASK, 0); }
		public TerminalNode OpenCurly() { return getToken(HATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(HATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(HATPParser.SEMICOLON, 0); }
		public List<SubselectionContext> subselection() {
			return getRuleContexts(SubselectionContext.class);
		}
		public SubselectionContext subselection(int i) {
			return getRuleContext(SubselectionContext.class,i);
		}
		public List<ListContext> list() {
			return getRuleContexts(ListContext.class);
		}
		public ListContext list(int i) {
			return getRuleContext(ListContext.class,i);
		}
		public SubtaskContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_subtask; }
	}

	public final SubtaskContext subtask() throws RecognitionException {
		SubtaskContext _localctx = new SubtaskContext(_ctx, getState());
		enterRule(_localctx, 28, RULE_subtask);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(223);
			match(SUBTASK);
			setState(224);
			match(OpenCurly);
			setState(229);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==IDENTIFIER || _la==NUMBER) {
				{
				setState(227);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case IDENTIFIER:
					{
					setState(225);
					subselection();
					}
					break;
				case NUMBER:
					{
					setState(226);
					list();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(231);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(232);
			match(CloseCurly);
			setState(233);
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
		public TerminalNode GOAL() { return getToken(HATPParser.GOAL, 0); }
		public TerminalNode OpenCurly() { return getToken(HATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(HATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(HATPParser.SEMICOLON, 0); }
		public List<ExpressionContext> expression() {
			return getRuleContexts(ExpressionContext.class);
		}
		public ExpressionContext expression(int i) {
			return getRuleContext(ExpressionContext.class,i);
		}
		public GoalContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_goal; }
	}

	public final GoalContext goal() throws RecognitionException {
		GoalContext _localctx = new GoalContext(_ctx, getState());
		enterRule(_localctx, 30, RULE_goal);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(235);
			match(GOAL);
			setState(236);
			match(OpenCurly);
			setState(238); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(237);
				expression();
				}
				}
				setState(240); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( _la==IDENTIFIER );
			setState(242);
			match(CloseCurly);
			setState(243);
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
	public static class CommentContext extends ParserRuleContext {
		public TerminalNode COMMENT() { return getToken(HATPParser.COMMENT, 0); }
		public TerminalNode LINE_COMMENT() { return getToken(HATPParser.LINE_COMMENT, 0); }
		public CommentContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_comment; }
	}

	public final CommentContext comment() throws RecognitionException {
		CommentContext _localctx = new CommentContext(_ctx, getState());
		enterRule(_localctx, 32, RULE_comment);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(245);
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
		public TerminalNode SEMICOLON() { return getToken(HATPParser.SEMICOLON, 0); }
		public IgnoreContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_ignore; }
	}

	public final IgnoreContext ignore() throws RecognitionException {
		IgnoreContext _localctx = new IgnoreContext(_ctx, getState());
		enterRule(_localctx, 34, RULE_ignore);
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(250);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,22,_ctx);
			while ( _alt!=1 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1+1 ) {
					{
					{
					setState(247);
					matchWildcard();
					}
					} 
				}
				setState(252);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,22,_ctx);
			}
			setState(253);
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
	public static class AttributContext extends ParserRuleContext {
		public VarnameContext varname() {
			return getRuleContext(VarnameContext.class,0);
		}
		public TerminalNode POINT() { return getToken(HATPParser.POINT, 0); }
		public ValueContext value() {
			return getRuleContext(ValueContext.class,0);
		}
		public AttributContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attribut; }
	}

	public final AttributContext attribut() throws RecognitionException {
		AttributContext _localctx = new AttributContext(_ctx, getState());
		enterRule(_localctx, 36, RULE_attribut);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(255);
			varname();
			setState(258);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==POINT) {
				{
				setState(256);
				match(POINT);
				setState(257);
				value();
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
	public static class SubjectContext extends ParserRuleContext {
		public AttributContext attribut() {
			return getRuleContext(AttributContext.class,0);
		}
		public SubjectContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_subject; }
	}

	public final SubjectContext subject() throws RecognitionException {
		SubjectContext _localctx = new SubjectContext(_ctx, getState());
		enterRule(_localctx, 38, RULE_subject);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(260);
			attribut();
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
		public AttributContext attribut() {
			return getRuleContext(AttributContext.class,0);
		}
		public ObjectContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_object; }
	}

	public final ObjectContext object() throws RecognitionException {
		ObjectContext _localctx = new ObjectContext(_ctx, getState());
		enterRule(_localctx, 40, RULE_object);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(262);
			attribut();
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
		public TerminalNode ADD_IN_SET() { return getToken(HATPParser.ADD_IN_SET, 0); }
		public TerminalNode REMOVE_FROM_SET() { return getToken(HATPParser.REMOVE_FROM_SET, 0); }
		public TerminalNode EQUAL() { return getToken(HATPParser.EQUAL, 0); }
		public TerminalNode TEST_EQUAL() { return getToken(HATPParser.TEST_EQUAL, 0); }
		public TerminalNode TEST_DIFF() { return getToken(HATPParser.TEST_DIFF, 0); }
		public TerminalNode TEST_SET_IN() { return getToken(HATPParser.TEST_SET_IN, 0); }
		public TerminalNode TEST_SET_NOT_IN() { return getToken(HATPParser.TEST_SET_NOT_IN, 0); }
		public TerminalNode SUP() { return getToken(HATPParser.SUP, 0); }
		public TerminalNode SUP_EQUAL() { return getToken(HATPParser.SUP_EQUAL, 0); }
		public TerminalNode INF() { return getToken(HATPParser.INF, 0); }
		public TerminalNode INF_EQUAL() { return getToken(HATPParser.INF_EQUAL, 0); }
		public TerminalNode SUP_TILD() { return getToken(HATPParser.SUP_TILD, 0); }
		public OperatorContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_operator; }
	}

	public final OperatorContext operator() throws RecognitionException {
		OperatorContext _localctx = new OperatorContext(_ctx, getState());
		enterRule(_localctx, 42, RULE_operator);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(264);
			_la = _input.LA(1);
			if ( !((((_la) & ~0x3f) == 0 && ((1L << _la) & 134184960L) != 0)) ) {
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
	public static class ExpressionContext extends ParserRuleContext {
		public SubjectContext subject() {
			return getRuleContext(SubjectContext.class,0);
		}
		public OperatorContext operator() {
			return getRuleContext(OperatorContext.class,0);
		}
		public ObjectContext object() {
			return getRuleContext(ObjectContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(HATPParser.SEMICOLON, 0); }
		public ExpressionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_expression; }
	}

	public final ExpressionContext expression() throws RecognitionException {
		ExpressionContext _localctx = new ExpressionContext(_ctx, getState());
		enterRule(_localctx, 44, RULE_expression);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(266);
			subject();
			setState(267);
			operator();
			setState(268);
			object();
			setState(269);
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
	public static class SubselectionContext extends ParserRuleContext {
		public AttributContext attribut() {
			return getRuleContext(AttributContext.class,0);
		}
		public OperatorContext operator() {
			return getRuleContext(OperatorContext.class,0);
		}
		public SelectcaseContext selectcase() {
			return getRuleContext(SelectcaseContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(HATPParser.SEMICOLON, 0); }
		public SubselectionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_subselection; }
	}

	public final SubselectionContext subselection() throws RecognitionException {
		SubselectionContext _localctx = new SubselectionContext(_ctx, getState());
		enterRule(_localctx, 46, RULE_subselection);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(271);
			attribut();
			setState(272);
			operator();
			setState(273);
			selectcase();
			setState(274);
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
	public static class SelectcaseContext extends ParserRuleContext {
		public TerminalNode SELECT() { return getToken(HATPParser.SELECT, 0); }
		public TerminalNode OpenPar() { return getToken(HATPParser.OpenPar, 0); }
		public TerminalNode IDENTIFIER() { return getToken(HATPParser.IDENTIFIER, 0); }
		public TerminalNode Comma() { return getToken(HATPParser.Comma, 0); }
		public TerminalNode OpenCurly() { return getToken(HATPParser.OpenCurly, 0); }
		public TerminalNode CloseCurly() { return getToken(HATPParser.CloseCurly, 0); }
		public TerminalNode ClosePar() { return getToken(HATPParser.ClosePar, 0); }
		public ExpressionContext expression() {
			return getRuleContext(ExpressionContext.class,0);
		}
		public SelectcaseContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_selectcase; }
	}

	public final SelectcaseContext selectcase() throws RecognitionException {
		SelectcaseContext _localctx = new SelectcaseContext(_ctx, getState());
		enterRule(_localctx, 48, RULE_selectcase);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(276);
			match(SELECT);
			setState(277);
			match(OpenPar);
			setState(278);
			match(IDENTIFIER);
			setState(279);
			match(Comma);
			setState(280);
			match(OpenCurly);
			setState(282);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==IDENTIFIER) {
				{
				setState(281);
				expression();
				}
			}

			setState(284);
			match(CloseCurly);
			setState(285);
			match(ClosePar);
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
	public static class ListContext extends ParserRuleContext {
		public TerminalNode NUMBER() { return getToken(HATPParser.NUMBER, 0); }
		public TerminalNode COLON() { return getToken(HATPParser.COLON, 0); }
		public FunctionContext function() {
			return getRuleContext(FunctionContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(HATPParser.SEMICOLON, 0); }
		public List<OrderContext> order() {
			return getRuleContexts(OrderContext.class);
		}
		public OrderContext order(int i) {
			return getRuleContext(OrderContext.class,i);
		}
		public List<TerminalNode> Comma() { return getTokens(HATPParser.Comma); }
		public TerminalNode Comma(int i) {
			return getToken(HATPParser.Comma, i);
		}
		public ListContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_list; }
	}

	public final ListContext list() throws RecognitionException {
		ListContext _localctx = new ListContext(_ctx, getState());
		enterRule(_localctx, 50, RULE_list);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(287);
			match(NUMBER);
			setState(288);
			match(COLON);
			setState(289);
			function();
			setState(295);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while ((((_la) & ~0x3f) == 0 && ((1L << _la) & 8796227207168L) != 0)) {
				{
				setState(293);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case ADD_IN_SET:
				case REMOVE_FROM_SET:
				case EQUAL:
				case TEST_EQUAL:
				case TEST_DIFF:
				case TEST_SET_IN:
				case TEST_SET_NOT_IN:
				case SUP:
				case SUP_EQUAL:
				case INF:
				case INF_EQUAL:
				case SUP_TILD:
					{
					setState(290);
					order();
					}
					break;
				case Comma:
					{
					setState(291);
					match(Comma);
					setState(292);
					order();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(297);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(298);
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
	public static class FunctionContext extends ParserRuleContext {
		public TerminalNode IDENTIFIER() { return getToken(HATPParser.IDENTIFIER, 0); }
		public TerminalNode OpenPar() { return getToken(HATPParser.OpenPar, 0); }
		public List<VarnameContext> varname() {
			return getRuleContexts(VarnameContext.class);
		}
		public VarnameContext varname(int i) {
			return getRuleContext(VarnameContext.class,i);
		}
		public TerminalNode ClosePar() { return getToken(HATPParser.ClosePar, 0); }
		public List<TerminalNode> Comma() { return getTokens(HATPParser.Comma); }
		public TerminalNode Comma(int i) {
			return getToken(HATPParser.Comma, i);
		}
		public FunctionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_function; }
	}

	public final FunctionContext function() throws RecognitionException {
		FunctionContext _localctx = new FunctionContext(_ctx, getState());
		enterRule(_localctx, 52, RULE_function);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(300);
			match(IDENTIFIER);
			setState(301);
			match(OpenPar);
			setState(302);
			varname();
			setState(307);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==Comma) {
				{
				{
				setState(303);
				match(Comma);
				setState(304);
				varname();
				}
				}
				setState(309);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(310);
			match(ClosePar);
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
	public static class OrderContext extends ParserRuleContext {
		public OperatorContext operator() {
			return getRuleContext(OperatorContext.class,0);
		}
		public TerminalNode NUMBER() { return getToken(HATPParser.NUMBER, 0); }
		public OrderContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_order; }
	}

	public final OrderContext order() throws RecognitionException {
		OrderContext _localctx = new OrderContext(_ctx, getState());
		enterRule(_localctx, 54, RULE_order);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(312);
			operator();
			setState(313);
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
	public static class ForallContext extends ParserRuleContext {
		public TerminalNode FORALL() { return getToken(HATPParser.FORALL, 0); }
		public TerminalNode OpenPar() { return getToken(HATPParser.OpenPar, 0); }
		public ArgumentsContext arguments() {
			return getRuleContext(ArgumentsContext.class,0);
		}
		public List<TerminalNode> Comma() { return getTokens(HATPParser.Comma); }
		public TerminalNode Comma(int i) {
			return getToken(HATPParser.Comma, i);
		}
		public TerminalNode ClosePar() { return getToken(HATPParser.ClosePar, 0); }
		public TerminalNode SEMICOLON() { return getToken(HATPParser.SEMICOLON, 0); }
		public List<TerminalNode> OpenCurly() { return getTokens(HATPParser.OpenCurly); }
		public TerminalNode OpenCurly(int i) {
			return getToken(HATPParser.OpenCurly, i);
		}
		public List<TerminalNode> CloseCurly() { return getTokens(HATPParser.CloseCurly); }
		public TerminalNode CloseCurly(int i) {
			return getToken(HATPParser.CloseCurly, i);
		}
		public List<ExpressionContext> expression() {
			return getRuleContexts(ExpressionContext.class);
		}
		public ExpressionContext expression(int i) {
			return getRuleContext(ExpressionContext.class,i);
		}
		public ForallContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_forall; }
	}

	public final ForallContext forall() throws RecognitionException {
		ForallContext _localctx = new ForallContext(_ctx, getState());
		enterRule(_localctx, 56, RULE_forall);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(315);
			match(FORALL);
			setState(316);
			match(OpenPar);
			setState(317);
			arguments();
			setState(318);
			match(Comma);
			{
			setState(319);
			match(OpenCurly);
			setState(321);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==IDENTIFIER) {
				{
				setState(320);
				expression();
				}
			}

			setState(323);
			match(CloseCurly);
			}
			setState(332);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==Comma) {
				{
				{
				setState(325);
				match(Comma);
				setState(326);
				match(OpenCurly);
				setState(327);
				expression();
				setState(328);
				match(CloseCurly);
				}
				}
				setState(334);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(335);
			match(ClosePar);
			setState(336);
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
	public static class CostContext extends ParserRuleContext {
		public TerminalNode COST() { return getToken(HATPParser.COST, 0); }
		public TerminalNode OpenCurly() { return getToken(HATPParser.OpenCurly, 0); }
		public TerminalNode IDENTIFIER() { return getToken(HATPParser.IDENTIFIER, 0); }
		public TerminalNode OpenClosePar() { return getToken(HATPParser.OpenClosePar, 0); }
		public TerminalNode CloseCurly() { return getToken(HATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(HATPParser.SEMICOLON, 0); }
		public CostContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_cost; }
	}

	public final CostContext cost() throws RecognitionException {
		CostContext _localctx = new CostContext(_ctx, getState());
		enterRule(_localctx, 58, RULE_cost);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(338);
			match(COST);
			setState(339);
			match(OpenCurly);
			setState(340);
			match(IDENTIFIER);
			setState(341);
			match(OpenClosePar);
			setState(342);
			match(CloseCurly);
			setState(343);
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
	public static class DurationContext extends ParserRuleContext {
		public TerminalNode DURATION() { return getToken(HATPParser.DURATION, 0); }
		public TerminalNode OpenCurly() { return getToken(HATPParser.OpenCurly, 0); }
		public TerminalNode IDENTIFIER() { return getToken(HATPParser.IDENTIFIER, 0); }
		public TerminalNode OpenClosePar() { return getToken(HATPParser.OpenClosePar, 0); }
		public TerminalNode CloseCurly() { return getToken(HATPParser.CloseCurly, 0); }
		public TerminalNode SEMICOLON() { return getToken(HATPParser.SEMICOLON, 0); }
		public DurationContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_duration; }
	}

	public final DurationContext duration() throws RecognitionException {
		DurationContext _localctx = new DurationContext(_ctx, getState());
		enterRule(_localctx, 60, RULE_duration);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(345);
			match(DURATION);
			setState(346);
			match(OpenCurly);
			setState(347);
			match(IDENTIFIER);
			setState(348);
			match(OpenClosePar);
			setState(349);
			match(CloseCurly);
			setState(350);
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

	public static final String _serializedATN =
		"\u0004\u00010\u0161\u0002\u0000\u0007\u0000\u0002\u0001\u0007\u0001\u0002"+
		"\u0002\u0007\u0002\u0002\u0003\u0007\u0003\u0002\u0004\u0007\u0004\u0002"+
		"\u0005\u0007\u0005\u0002\u0006\u0007\u0006\u0002\u0007\u0007\u0007\u0002"+
		"\b\u0007\b\u0002\t\u0007\t\u0002\n\u0007\n\u0002\u000b\u0007\u000b\u0002"+
		"\f\u0007\f\u0002\r\u0007\r\u0002\u000e\u0007\u000e\u0002\u000f\u0007\u000f"+
		"\u0002\u0010\u0007\u0010\u0002\u0011\u0007\u0011\u0002\u0012\u0007\u0012"+
		"\u0002\u0013\u0007\u0013\u0002\u0014\u0007\u0014\u0002\u0015\u0007\u0015"+
		"\u0002\u0016\u0007\u0016\u0002\u0017\u0007\u0017\u0002\u0018\u0007\u0018"+
		"\u0002\u0019\u0007\u0019\u0002\u001a\u0007\u001a\u0002\u001b\u0007\u001b"+
		"\u0002\u001c\u0007\u001c\u0002\u001d\u0007\u001d\u0002\u001e\u0007\u001e"+
		"\u0001\u0000\u0005\u0000@\b\u0000\n\u0000\f\u0000C\t\u0000\u0001\u0000"+
		"\u0005\u0000F\b\u0000\n\u0000\f\u0000I\t\u0000\u0001\u0000\u0005\u0000"+
		"L\b\u0000\n\u0000\f\u0000O\t\u0000\u0001\u0000\u0001\u0000\u0005\u0000"+
		"S\b\u0000\n\u0000\f\u0000V\t\u0000\u0001\u0000\u0001\u0000\u0005\u0000"+
		"Z\b\u0000\n\u0000\f\u0000]\t\u0000\u0001\u0000\u0001\u0000\u0001\u0001"+
		"\u0001\u0001\u0001\u0001\u0005\u0001d\b\u0001\n\u0001\f\u0001g\t\u0001"+
		"\u0001\u0001\u0001\u0001\u0001\u0002\u0001\u0002\u0001\u0002\u0005\u0002"+
		"n\b\u0002\n\u0002\f\u0002q\t\u0002\u0001\u0002\u0001\u0002\u0001\u0003"+
		"\u0001\u0003\u0001\u0003\u0005\u0003x\b\u0003\n\u0003\f\u0003{\t\u0003"+
		"\u0001\u0003\u0005\u0003~\b\u0003\n\u0003\f\u0003\u0081\t\u0003\u0001"+
		"\u0003\u0001\u0003\u0001\u0004\u0001\u0004\u0001\u0004\u0001\u0004\u0001"+
		"\u0004\u0001\u0004\u0005\u0004\u008b\b\u0004\n\u0004\f\u0004\u008e\t\u0004"+
		"\u0001\u0004\u0001\u0004\u0001\u0004\u0004\u0004\u0093\b\u0004\u000b\u0004"+
		"\f\u0004\u0094\u0001\u0004\u0004\u0004\u0098\b\u0004\u000b\u0004\f\u0004"+
		"\u0099\u0001\u0004\u0003\u0004\u009d\b\u0004\u0001\u0004\u0003\u0004\u00a0"+
		"\b\u0004\u0001\u0004\u0001\u0004\u0001\u0005\u0001\u0005\u0001\u0006\u0001"+
		"\u0006\u0001\u0006\u0005\u0006\u00a9\b\u0006\n\u0006\f\u0006\u00ac\t\u0006"+
		"\u0001\u0006\u0001\u0006\u0001\u0006\u0001\u0007\u0001\u0007\u0001\u0007"+
		"\u0001\u0007\u0005\u0007\u00b5\b\u0007\n\u0007\f\u0007\u00b8\t\u0007\u0001"+
		"\u0007\u0001\u0007\u0001\u0007\u0001\b\u0001\b\u0001\b\u0001\t\u0001\t"+
		"\u0001\n\u0001\n\u0001\u000b\u0001\u000b\u0001\f\u0001\f\u0001\f\u0001"+
		"\f\u0001\f\u0001\f\u0005\f\u00cc\b\f\n\f\f\f\u00cf\t\f\u0001\f\u0001\f"+
		"\u0001\f\u0001\f\u0004\f\u00d5\b\f\u000b\f\f\f\u00d6\u0001\f\u0001\f\u0001"+
		"\r\u0001\r\u0001\r\u0001\r\u0001\r\u0001\u000e\u0001\u000e\u0001\u000e"+
		"\u0001\u000e\u0005\u000e\u00e4\b\u000e\n\u000e\f\u000e\u00e7\t\u000e\u0001"+
		"\u000e\u0001\u000e\u0001\u000e\u0001\u000f\u0001\u000f\u0001\u000f\u0004"+
		"\u000f\u00ef\b\u000f\u000b\u000f\f\u000f\u00f0\u0001\u000f\u0001\u000f"+
		"\u0001\u000f\u0001\u0010\u0001\u0010\u0001\u0011\u0005\u0011\u00f9\b\u0011"+
		"\n\u0011\f\u0011\u00fc\t\u0011\u0001\u0011\u0001\u0011\u0001\u0012\u0001"+
		"\u0012\u0001\u0012\u0003\u0012\u0103\b\u0012\u0001\u0013\u0001\u0013\u0001"+
		"\u0014\u0001\u0014\u0001\u0015\u0001\u0015\u0001\u0016\u0001\u0016\u0001"+
		"\u0016\u0001\u0016\u0001\u0016\u0001\u0017\u0001\u0017\u0001\u0017\u0001"+
		"\u0017\u0001\u0017\u0001\u0018\u0001\u0018\u0001\u0018\u0001\u0018\u0001"+
		"\u0018\u0001\u0018\u0003\u0018\u011b\b\u0018\u0001\u0018\u0001\u0018\u0001"+
		"\u0018\u0001\u0019\u0001\u0019\u0001\u0019\u0001\u0019\u0001\u0019\u0001"+
		"\u0019\u0005\u0019\u0126\b\u0019\n\u0019\f\u0019\u0129\t\u0019\u0001\u0019"+
		"\u0001\u0019\u0001\u001a\u0001\u001a\u0001\u001a\u0001\u001a\u0001\u001a"+
		"\u0005\u001a\u0132\b\u001a\n\u001a\f\u001a\u0135\t\u001a\u0001\u001a\u0001"+
		"\u001a\u0001\u001b\u0001\u001b\u0001\u001b\u0001\u001c\u0001\u001c\u0001"+
		"\u001c\u0001\u001c\u0001\u001c\u0001\u001c\u0003\u001c\u0142\b\u001c\u0001"+
		"\u001c\u0001\u001c\u0001\u001c\u0001\u001c\u0001\u001c\u0001\u001c\u0001"+
		"\u001c\u0005\u001c\u014b\b\u001c\n\u001c\f\u001c\u014e\t\u001c\u0001\u001c"+
		"\u0001\u001c\u0001\u001c\u0001\u001d\u0001\u001d\u0001\u001d\u0001\u001d"+
		"\u0001\u001d\u0001\u001d\u0001\u001d\u0001\u001e\u0001\u001e\u0001\u001e"+
		"\u0001\u001e\u0001\u001e\u0001\u001e\u0001\u001e\u0001\u001e\u0001\u00fa"+
		"\u0000\u001f\u0000\u0002\u0004\u0006\b\n\f\u000e\u0010\u0012\u0014\u0016"+
		"\u0018\u001a\u001c\u001e \"$&(*,.02468:<\u0000\u0002\u0001\u0000$%\u0001"+
		"\u0000\u000f\u001a\u015f\u0000A\u0001\u0000\u0000\u0000\u0002`\u0001\u0000"+
		"\u0000\u0000\u0004j\u0001\u0000\u0000\u0000\u0006t\u0001\u0000\u0000\u0000"+
		"\b\u0084\u0001\u0000\u0000\u0000\n\u00a3\u0001\u0000\u0000\u0000\f\u00a5"+
		"\u0001\u0000\u0000\u0000\u000e\u00b0\u0001\u0000\u0000\u0000\u0010\u00bc"+
		"\u0001\u0000\u0000\u0000\u0012\u00bf\u0001\u0000\u0000\u0000\u0014\u00c1"+
		"\u0001\u0000\u0000\u0000\u0016\u00c3\u0001\u0000\u0000\u0000\u0018\u00c5"+
		"\u0001\u0000\u0000\u0000\u001a\u00da\u0001\u0000\u0000\u0000\u001c\u00df"+
		"\u0001\u0000\u0000\u0000\u001e\u00eb\u0001\u0000\u0000\u0000 \u00f5\u0001"+
		"\u0000\u0000\u0000\"\u00fa\u0001\u0000\u0000\u0000$\u00ff\u0001\u0000"+
		"\u0000\u0000&\u0104\u0001\u0000\u0000\u0000(\u0106\u0001\u0000\u0000\u0000"+
		"*\u0108\u0001\u0000\u0000\u0000,\u010a\u0001\u0000\u0000\u0000.\u010f"+
		"\u0001\u0000\u0000\u00000\u0114\u0001\u0000\u0000\u00002\u011f\u0001\u0000"+
		"\u0000\u00004\u012c\u0001\u0000\u0000\u00006\u0138\u0001\u0000\u0000\u0000"+
		"8\u013b\u0001\u0000\u0000\u0000:\u0152\u0001\u0000\u0000\u0000<\u0159"+
		"\u0001\u0000\u0000\u0000>@\u0003 \u0010\u0000?>\u0001\u0000\u0000\u0000"+
		"@C\u0001\u0000\u0000\u0000A?\u0001\u0000\u0000\u0000AB\u0001\u0000\u0000"+
		"\u0000BG\u0001\u0000\u0000\u0000CA\u0001\u0000\u0000\u0000DF\u0003\u0004"+
		"\u0002\u0000ED\u0001\u0000\u0000\u0000FI\u0001\u0000\u0000\u0000GE\u0001"+
		"\u0000\u0000\u0000GH\u0001\u0000\u0000\u0000HM\u0001\u0000\u0000\u0000"+
		"IG\u0001\u0000\u0000\u0000JL\u0003 \u0010\u0000KJ\u0001\u0000\u0000\u0000"+
		"LO\u0001\u0000\u0000\u0000MK\u0001\u0000\u0000\u0000MN\u0001\u0000\u0000"+
		"\u0000NP\u0001\u0000\u0000\u0000OM\u0001\u0000\u0000\u0000PT\u0003\u0006"+
		"\u0003\u0000QS\u0003 \u0010\u0000RQ\u0001\u0000\u0000\u0000SV\u0001\u0000"+
		"\u0000\u0000TR\u0001\u0000\u0000\u0000TU\u0001\u0000\u0000\u0000UW\u0001"+
		"\u0000\u0000\u0000VT\u0001\u0000\u0000\u0000W[\u0003\u0002\u0001\u0000"+
		"XZ\u0003 \u0010\u0000YX\u0001\u0000\u0000\u0000Z]\u0001\u0000\u0000\u0000"+
		"[Y\u0001\u0000\u0000\u0000[\\\u0001\u0000\u0000\u0000\\^\u0001\u0000\u0000"+
		"\u0000][\u0001\u0000\u0000\u0000^_\u0005\u0000\u0000\u0001_\u0001\u0001"+
		"\u0000\u0000\u0000`a\u0005\b\u0000\u0000ae\u0005)\u0000\u0000bd\u0003"+
		"\"\u0011\u0000cb\u0001\u0000\u0000\u0000dg\u0001\u0000\u0000\u0000ec\u0001"+
		"\u0000\u0000\u0000ef\u0001\u0000\u0000\u0000fh\u0001\u0000\u0000\u0000"+
		"ge\u0001\u0000\u0000\u0000hi\u0005*\u0000\u0000i\u0003\u0001\u0000\u0000"+
		"\u0000jk\u0005\t\u0000\u0000ko\u0005)\u0000\u0000ln\u0003\"\u0011\u0000"+
		"ml\u0001\u0000\u0000\u0000nq\u0001\u0000\u0000\u0000om\u0001\u0000\u0000"+
		"\u0000op\u0001\u0000\u0000\u0000pr\u0001\u0000\u0000\u0000qo\u0001\u0000"+
		"\u0000\u0000rs\u0005*\u0000\u0000s\u0005\u0001\u0000\u0000\u0000tu\u0005"+
		"\u0001\u0000\u0000uy\u0005)\u0000\u0000vx\u0003\b\u0004\u0000wv\u0001"+
		"\u0000\u0000\u0000x{\u0001\u0000\u0000\u0000yw\u0001\u0000\u0000\u0000"+
		"yz\u0001\u0000\u0000\u0000z\u007f\u0001\u0000\u0000\u0000{y\u0001\u0000"+
		"\u0000\u0000|~\u0003\u0018\f\u0000}|\u0001\u0000\u0000\u0000~\u0081\u0001"+
		"\u0000\u0000\u0000\u007f}\u0001\u0000\u0000\u0000\u007f\u0080\u0001\u0000"+
		"\u0000\u0000\u0080\u0082\u0001\u0000\u0000\u0000\u0081\u007f\u0001\u0000"+
		"\u0000\u0000\u0082\u0083\u0005*\u0000\u0000\u0083\u0007\u0001\u0000\u0000"+
		"\u0000\u0084\u0085\u0005\u0002\u0000\u0000\u0085\u0086\u0003\n\u0005\u0000"+
		"\u0086\u0087\u0005&\u0000\u0000\u0087\u008c\u0003\u0010\b\u0000\u0088"+
		"\u0089\u0005+\u0000\u0000\u0089\u008b\u0003\u0010\b\u0000\u008a\u0088"+
		"\u0001\u0000\u0000\u0000\u008b\u008e\u0001\u0000\u0000\u0000\u008c\u008a"+
		"\u0001\u0000\u0000\u0000\u008c\u008d\u0001\u0000\u0000\u0000\u008d\u008f"+
		"\u0001\u0000\u0000\u0000\u008e\u008c\u0001\u0000\u0000\u0000\u008f\u0090"+
		"\u0005\'\u0000\u0000\u0090\u0092\u0005)\u0000\u0000\u0091\u0093\u0003"+
		"\f\u0006\u0000\u0092\u0091\u0001\u0000\u0000\u0000\u0093\u0094\u0001\u0000"+
		"\u0000\u0000\u0094\u0092\u0001\u0000\u0000\u0000\u0094\u0095\u0001\u0000"+
		"\u0000\u0000\u0095\u0097\u0001\u0000\u0000\u0000\u0096\u0098\u0003\u000e"+
		"\u0007\u0000\u0097\u0096\u0001\u0000\u0000\u0000\u0098\u0099\u0001\u0000"+
		"\u0000\u0000\u0099\u0097\u0001\u0000\u0000\u0000\u0099\u009a\u0001\u0000"+
		"\u0000\u0000\u009a\u009c\u0001\u0000\u0000\u0000\u009b\u009d\u0003:\u001d"+
		"\u0000\u009c\u009b\u0001\u0000\u0000\u0000\u009c\u009d\u0001\u0000\u0000"+
		"\u0000\u009d\u009f\u0001\u0000\u0000\u0000\u009e\u00a0\u0003<\u001e\u0000"+
		"\u009f\u009e\u0001\u0000\u0000\u0000\u009f\u00a0\u0001\u0000\u0000\u0000"+
		"\u00a0\u00a1\u0001\u0000\u0000\u0000\u00a1\u00a2\u0005*\u0000\u0000\u00a2"+
		"\t\u0001\u0000\u0000\u0000\u00a3\u00a4\u0005,\u0000\u0000\u00a4\u000b"+
		"\u0001\u0000\u0000\u0000\u00a5\u00a6\u0005\u0003\u0000\u0000\u00a6\u00aa"+
		"\u0005)\u0000\u0000\u00a7\u00a9\u0003,\u0016\u0000\u00a8\u00a7\u0001\u0000"+
		"\u0000\u0000\u00a9\u00ac\u0001\u0000\u0000\u0000\u00aa\u00a8\u0001\u0000"+
		"\u0000\u0000\u00aa\u00ab\u0001\u0000\u0000\u0000\u00ab\u00ad\u0001\u0000"+
		"\u0000\u0000\u00ac\u00aa\u0001\u0000\u0000\u0000\u00ad\u00ae\u0005*\u0000"+
		"\u0000\u00ae\u00af\u0005\u001f\u0000\u0000\u00af\r\u0001\u0000\u0000\u0000"+
		"\u00b0\u00b1\u0005\u0004\u0000\u0000\u00b1\u00b6\u0005)\u0000\u0000\u00b2"+
		"\u00b5\u00038\u001c\u0000\u00b3\u00b5\u0003,\u0016\u0000\u00b4\u00b2\u0001"+
		"\u0000\u0000\u0000\u00b4\u00b3\u0001\u0000\u0000\u0000\u00b5\u00b8\u0001"+
		"\u0000\u0000\u0000\u00b6\u00b4\u0001\u0000\u0000\u0000\u00b6\u00b7\u0001"+
		"\u0000\u0000\u0000\u00b7\u00b9\u0001\u0000\u0000\u0000\u00b8\u00b6\u0001"+
		"\u0000\u0000\u0000\u00b9\u00ba\u0005*\u0000\u0000\u00ba\u00bb\u0005\u001f"+
		"\u0000\u0000\u00bb\u000f\u0001\u0000\u0000\u0000\u00bc\u00bd\u0003\u0012"+
		"\t\u0000\u00bd\u00be\u0003\u0014\n\u0000\u00be\u0011\u0001\u0000\u0000"+
		"\u0000\u00bf\u00c0\u0005,\u0000\u0000\u00c0\u0013\u0001\u0000\u0000\u0000"+
		"\u00c1\u00c2\u0005,\u0000\u0000\u00c2\u0015\u0001\u0000\u0000\u0000\u00c3"+
		"\u00c4\u0005,\u0000\u0000\u00c4\u0017\u0001\u0000\u0000\u0000\u00c5\u00c6"+
		"\u0005\u0005\u0000\u0000\u00c6\u00c7\u0005,\u0000\u0000\u00c7\u00c8\u0005"+
		"&\u0000\u0000\u00c8\u00cd\u0003\u0010\b\u0000\u00c9\u00ca\u0005+\u0000"+
		"\u0000\u00ca\u00cc\u0003\u0010\b\u0000\u00cb\u00c9\u0001\u0000\u0000\u0000"+
		"\u00cc\u00cf\u0001\u0000\u0000\u0000\u00cd\u00cb\u0001\u0000\u0000\u0000"+
		"\u00cd\u00ce\u0001\u0000\u0000\u0000\u00ce\u00d0\u0001\u0000\u0000\u0000"+
		"\u00cf\u00cd\u0001\u0000\u0000\u0000\u00d0\u00d1\u0005\'\u0000\u0000\u00d1"+
		"\u00d2\u0005)\u0000\u0000\u00d2\u00d4\u0003\u001e\u000f\u0000\u00d3\u00d5"+
		"\u0003\u001a\r\u0000\u00d4\u00d3\u0001\u0000\u0000\u0000\u00d5\u00d6\u0001"+
		"\u0000\u0000\u0000\u00d6\u00d4\u0001\u0000\u0000\u0000\u00d6\u00d7\u0001"+
		"\u0000\u0000\u0000\u00d7\u00d8\u0001\u0000\u0000\u0000\u00d8\u00d9\u0005"+
		"*\u0000\u0000\u00d9\u0019\u0001\u0000\u0000\u0000\u00da\u00db\u0005)\u0000"+
		"\u0000\u00db\u00dc\u0003\f\u0006\u0000\u00dc\u00dd\u0003\u001c\u000e\u0000"+
		"\u00dd\u00de\u0005*\u0000\u0000\u00de\u001b\u0001\u0000\u0000\u0000\u00df"+
		"\u00e0\u0005\u0007\u0000\u0000\u00e0\u00e5\u0005)\u0000\u0000\u00e1\u00e4"+
		"\u0003.\u0017\u0000\u00e2\u00e4\u00032\u0019\u0000\u00e3\u00e1\u0001\u0000"+
		"\u0000\u0000\u00e3\u00e2\u0001\u0000\u0000\u0000\u00e4\u00e7\u0001\u0000"+
		"\u0000\u0000\u00e5\u00e3\u0001\u0000\u0000\u0000\u00e5\u00e6\u0001\u0000"+
		"\u0000\u0000\u00e6\u00e8\u0001\u0000\u0000\u0000\u00e7\u00e5\u0001\u0000"+
		"\u0000\u0000\u00e8\u00e9\u0005*\u0000\u0000\u00e9\u00ea\u0005\u001f\u0000"+
		"\u0000\u00ea\u001d\u0001\u0000\u0000\u0000\u00eb\u00ec\u0005\u0006\u0000"+
		"\u0000\u00ec\u00ee\u0005)\u0000\u0000\u00ed\u00ef\u0003,\u0016\u0000\u00ee"+
		"\u00ed\u0001\u0000\u0000\u0000\u00ef\u00f0\u0001\u0000\u0000\u0000\u00f0"+
		"\u00ee\u0001\u0000\u0000\u0000\u00f0\u00f1\u0001\u0000\u0000\u0000\u00f1"+
		"\u00f2\u0001\u0000\u0000\u0000\u00f2\u00f3\u0005*\u0000\u0000\u00f3\u00f4"+
		"\u0005\u001f\u0000\u0000\u00f4\u001f\u0001\u0000\u0000\u0000\u00f5\u00f6"+
		"\u0007\u0000\u0000\u0000\u00f6!\u0001\u0000\u0000\u0000\u00f7\u00f9\t"+
		"\u0000\u0000\u0000\u00f8\u00f7\u0001\u0000\u0000\u0000\u00f9\u00fc\u0001"+
		"\u0000\u0000\u0000\u00fa\u00fb\u0001\u0000\u0000\u0000\u00fa\u00f8\u0001"+
		"\u0000\u0000\u0000\u00fb\u00fd\u0001\u0000\u0000\u0000\u00fc\u00fa\u0001"+
		"\u0000\u0000\u0000\u00fd\u00fe\u0005\u001f\u0000\u0000\u00fe#\u0001\u0000"+
		"\u0000\u0000\u00ff\u0102\u0003\u0014\n\u0000\u0100\u0101\u0005 \u0000"+
		"\u0000\u0101\u0103\u0003\u0016\u000b\u0000\u0102\u0100\u0001\u0000\u0000"+
		"\u0000\u0102\u0103\u0001\u0000\u0000\u0000\u0103%\u0001\u0000\u0000\u0000"+
		"\u0104\u0105\u0003$\u0012\u0000\u0105\'\u0001\u0000\u0000\u0000\u0106"+
		"\u0107\u0003$\u0012\u0000\u0107)\u0001\u0000\u0000\u0000\u0108\u0109\u0007"+
		"\u0001\u0000\u0000\u0109+\u0001\u0000\u0000\u0000\u010a\u010b\u0003&\u0013"+
		"\u0000\u010b\u010c\u0003*\u0015\u0000\u010c\u010d\u0003(\u0014\u0000\u010d"+
		"\u010e\u0005\u001f\u0000\u0000\u010e-\u0001\u0000\u0000\u0000\u010f\u0110"+
		"\u0003$\u0012\u0000\u0110\u0111\u0003*\u0015\u0000\u0111\u0112\u00030"+
		"\u0018\u0000\u0112\u0113\u0005\u001f\u0000\u0000\u0113/\u0001\u0000\u0000"+
		"\u0000\u0114\u0115\u0005\r\u0000\u0000\u0115\u0116\u0005&\u0000\u0000"+
		"\u0116\u0117\u0005,\u0000\u0000\u0117\u0118\u0005+\u0000\u0000\u0118\u011a"+
		"\u0005)\u0000\u0000\u0119\u011b\u0003,\u0016\u0000\u011a\u0119\u0001\u0000"+
		"\u0000\u0000\u011a\u011b\u0001\u0000\u0000\u0000\u011b\u011c\u0001\u0000"+
		"\u0000\u0000\u011c\u011d\u0005*\u0000\u0000\u011d\u011e\u0005\'\u0000"+
		"\u0000\u011e1\u0001\u0000\u0000\u0000\u011f\u0120\u00050\u0000\u0000\u0120"+
		"\u0121\u0005\"\u0000\u0000\u0121\u0127\u00034\u001a\u0000\u0122\u0126"+
		"\u00036\u001b\u0000\u0123\u0124\u0005+\u0000\u0000\u0124\u0126\u00036"+
		"\u001b\u0000\u0125\u0122\u0001\u0000\u0000\u0000\u0125\u0123\u0001\u0000"+
		"\u0000\u0000\u0126\u0129\u0001\u0000\u0000\u0000\u0127\u0125\u0001\u0000"+
		"\u0000\u0000\u0127\u0128\u0001\u0000\u0000\u0000\u0128\u012a\u0001\u0000"+
		"\u0000\u0000\u0129\u0127\u0001\u0000\u0000\u0000\u012a\u012b\u0005\u001f"+
		"\u0000\u0000\u012b3\u0001\u0000\u0000\u0000\u012c\u012d\u0005,\u0000\u0000"+
		"\u012d\u012e\u0005&\u0000\u0000\u012e\u0133\u0003\u0014\n\u0000\u012f"+
		"\u0130\u0005+\u0000\u0000\u0130\u0132\u0003\u0014\n\u0000\u0131\u012f"+
		"\u0001\u0000\u0000\u0000\u0132\u0135\u0001\u0000\u0000\u0000\u0133\u0131"+
		"\u0001\u0000\u0000\u0000\u0133\u0134\u0001\u0000\u0000\u0000\u0134\u0136"+
		"\u0001\u0000\u0000\u0000\u0135\u0133\u0001\u0000\u0000\u0000\u0136\u0137"+
		"\u0005\'\u0000\u0000\u01375\u0001\u0000\u0000\u0000\u0138\u0139\u0003"+
		"*\u0015\u0000\u0139\u013a\u00050\u0000\u0000\u013a7\u0001\u0000\u0000"+
		"\u0000\u013b\u013c\u0005\n\u0000\u0000\u013c\u013d\u0005&\u0000\u0000"+
		"\u013d\u013e\u0003\u0010\b\u0000\u013e\u013f\u0005+\u0000\u0000\u013f"+
		"\u0141\u0005)\u0000\u0000\u0140\u0142\u0003,\u0016\u0000\u0141\u0140\u0001"+
		"\u0000\u0000\u0000\u0141\u0142\u0001\u0000\u0000\u0000\u0142\u0143\u0001"+
		"\u0000\u0000\u0000\u0143\u0144\u0005*\u0000\u0000\u0144\u014c\u0001\u0000"+
		"\u0000\u0000\u0145\u0146\u0005+\u0000\u0000\u0146\u0147\u0005)\u0000\u0000"+
		"\u0147\u0148\u0003,\u0016\u0000\u0148\u0149\u0005*\u0000\u0000\u0149\u014b"+
		"\u0001\u0000\u0000\u0000\u014a\u0145\u0001\u0000\u0000\u0000\u014b\u014e"+
		"\u0001\u0000\u0000\u0000\u014c\u014a\u0001\u0000\u0000\u0000\u014c\u014d"+
		"\u0001\u0000\u0000\u0000\u014d\u014f\u0001\u0000\u0000\u0000\u014e\u014c"+
		"\u0001\u0000\u0000\u0000\u014f\u0150\u0005\'\u0000\u0000\u0150\u0151\u0005"+
		"\u001f\u0000\u0000\u01519\u0001\u0000\u0000\u0000\u0152\u0153\u0005\u000b"+
		"\u0000\u0000\u0153\u0154\u0005)\u0000\u0000\u0154\u0155\u0005,\u0000\u0000"+
		"\u0155\u0156\u0005(\u0000\u0000\u0156\u0157\u0005*\u0000\u0000\u0157\u0158"+
		"\u0005\u001f\u0000\u0000\u0158;\u0001\u0000\u0000\u0000\u0159\u015a\u0005"+
		"\f\u0000\u0000\u015a\u015b\u0005)\u0000\u0000\u015b\u015c\u0005,\u0000"+
		"\u0000\u015c\u015d\u0005(\u0000\u0000\u015d\u015e\u0005*\u0000\u0000\u015e"+
		"\u015f\u0005\u001f\u0000\u0000\u015f=\u0001\u0000\u0000\u0000\u001eAG"+
		"MT[eoy\u007f\u008c\u0094\u0099\u009c\u009f\u00aa\u00b4\u00b6\u00cd\u00d6"+
		"\u00e3\u00e5\u00f0\u00fa\u0102\u011a\u0125\u0127\u0133\u0141\u014c";
	public static final ATN _ATN =
		new ATNDeserializer().deserialize(_serializedATN.toCharArray());
	static {
		_decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
		for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
			_decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
		}
	}
}