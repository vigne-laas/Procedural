// Generated from /home/avigne/Projets/Procedural/catkin_ws/src/Procedural/include/procedural/old/reader/grammar/htn.g4 by ANTLR 4.13.1
import org.antlr.v4.runtime.atn.*;
import org.antlr.v4.runtime.dfa.DFA;
import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.misc.*;
import org.antlr.v4.runtime.tree.*;
import java.util.List;
import java.util.Iterator;
import java.util.ArrayList;

@SuppressWarnings({"all", "warnings", "unchecked", "unused", "cast", "CheckReturnValue"})
public class htnParser extends Parser {
	static { RuntimeMetaData.checkVersion("4.13.1", RuntimeMetaData.VERSION); }

	protected static final DFA[] _decisionToDFA;
	protected static final PredictionContextCache _sharedContextCache =
		new PredictionContextCache();
	public static final int
		T__0=1, T__1=2, T__2=3, T__3=4, T__4=5, T__5=6, HTN=7, ACTION=8, PRECONDITIONS=9, 
		EFFECTS=10, METHOD=11, GOAL=12, SUBTASK=13, TIMEPART=14, FACTDATABASE=15, 
		FORALL=16, COST=17, DURATION=18, SELECT=19, WS=20, ADD_IN_SET=21, REMOVE_FROM_SET=22, 
		EQUAL=23, TEST_EQUAL=24, TEST_DIFF=25, TEST_SET_IN=26, TEST_SET_NOT_IN=27, 
		SUP=28, SUP_EQUAL=29, INF=30, INF_EQUAL=31, SUP_TILD=32, PLUS=33, MINUS=34, 
		TIMES=35, SLASH=36, SEMICOLON=37, POINT=38, STRING=39, COLON=40, QUESTIONMARK=41, 
		COMMENT=42, LINE_COMMENT=43, IDENTIFIER=44, TYPE=45, VARNAME=46, ATTRIBUT=47, 
		NUMBER=48;
	public static final int
		RULE_hatp = 0, RULE_timepart = 1, RULE_factbase = 2, RULE_htn = 3, RULE_actions = 4, 
		RULE_preconditions = 5, RULE_effects = 6, RULE_arguments = 7, RULE_type = 8, 
		RULE_varname = 9, RULE_value = 10, RULE_methods = 11, RULE_subtask = 12, 
		RULE_goal = 13, RULE_comment = 14, RULE_ignore = 15, RULE_attribut = 16, 
		RULE_operator = 17, RULE_expression = 18, RULE_subselection = 19, RULE_selectcase = 20, 
		RULE_order = 21, RULE_function = 22, RULE_test = 23, RULE_forall = 24, 
		RULE_cost = 25, RULE_duration = 26;
	private static String[] makeRuleNames() {
		return new String[] {
			"hatp", "timepart", "factbase", "htn", "actions", "preconditions", "effects", 
			"arguments", "type", "varname", "value", "methods", "subtask", "goal", 
			"comment", "ignore", "attribut", "operator", "expression", "subselection", 
			"selectcase", "order", "function", "test", "forall", "cost", "duration"
		};
	}
	public static final String[] ruleNames = makeRuleNames();

	private static String[] makeLiteralNames() {
		return new String[] {
			null, "'{'", "'}'", "'('", "','", "')'", "'()'", "'HTN'", "'action'", 
			"'preconditions'", "'effects'", "'method'", "'goal'", "'subtasks'", "'timePart'", 
			"'factdatabase'", "'FORALL'", "'cost'", "'duration'", "'SELECT'", null, 
			"'<<='", "'=>>'", "'='", "'=='", "'!='", "'>>'", "'!>>'", "'>'", "'>='", 
			"'<'", "'<='", "'~>'", "'+'", "'-'", "'*'", "'/'", "';'", "'.'", "'\"'", 
			"':'", "'?'"
		};
	}
	private static final String[] _LITERAL_NAMES = makeLiteralNames();
	private static String[] makeSymbolicNames() {
		return new String[] {
			null, null, null, null, null, null, null, "HTN", "ACTION", "PRECONDITIONS", 
			"EFFECTS", "METHOD", "GOAL", "SUBTASK", "TIMEPART", "FACTDATABASE", "FORALL", 
			"COST", "DURATION", "SELECT", "WS", "ADD_IN_SET", "REMOVE_FROM_SET", 
			"EQUAL", "TEST_EQUAL", "TEST_DIFF", "TEST_SET_IN", "TEST_SET_NOT_IN", 
			"SUP", "SUP_EQUAL", "INF", "INF_EQUAL", "SUP_TILD", "PLUS", "MINUS", 
			"TIMES", "SLASH", "SEMICOLON", "POINT", "STRING", "COLON", "QUESTIONMARK", 
			"COMMENT", "LINE_COMMENT", "IDENTIFIER", "TYPE", "VARNAME", "ATTRIBUT", 
			"NUMBER"
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
	public String getGrammarFileName() { return "htn.g4"; }

	@Override
	public String[] getRuleNames() { return ruleNames; }

	@Override
	public String getSerializedATN() { return _serializedATN; }

	@Override
	public ATN getATN() { return _ATN; }

	public htnParser(TokenStream input) {
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
		public TerminalNode EOF() { return getToken(htnParser.EOF, 0); }
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
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterHatp(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitHatp(this);
		}
	}

	public final HatpContext hatp() throws RecognitionException {
		HatpContext _localctx = new HatpContext(_ctx, getState());
		enterRule(_localctx, 0, RULE_hatp);
		int _la;
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(57);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,0,_ctx);
			while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1 ) {
					{
					{
					setState(54);
					comment();
					}
					} 
				}
				setState(59);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,0,_ctx);
			}
			setState(63);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==FACTDATABASE) {
				{
				{
				setState(60);
				factbase();
				}
				}
				setState(65);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(69);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMENT || _la==LINE_COMMENT) {
				{
				{
				setState(66);
				comment();
				}
				}
				setState(71);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(72);
			htn();
			setState(76);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMENT || _la==LINE_COMMENT) {
				{
				{
				setState(73);
				comment();
				}
				}
				setState(78);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(79);
			timepart();
			setState(83);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMENT || _la==LINE_COMMENT) {
				{
				{
				setState(80);
				comment();
				}
				}
				setState(85);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(86);
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
		public TerminalNode TIMEPART() { return getToken(htnParser.TIMEPART, 0); }
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
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterTimepart(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitTimepart(this);
		}
	}

	public final TimepartContext timepart() throws RecognitionException {
		TimepartContext _localctx = new TimepartContext(_ctx, getState());
		enterRule(_localctx, 2, RULE_timepart);
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(88);
			match(TIMEPART);
			setState(89);
			match(T__0);
			setState(93);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,5,_ctx);
			while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1 ) {
					{
					{
					setState(90);
					ignore();
					}
					} 
				}
				setState(95);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,5,_ctx);
			}
			setState(96);
			match(T__1);
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
		public TerminalNode FACTDATABASE() { return getToken(htnParser.FACTDATABASE, 0); }
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
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterFactbase(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitFactbase(this);
		}
	}

	public final FactbaseContext factbase() throws RecognitionException {
		FactbaseContext _localctx = new FactbaseContext(_ctx, getState());
		enterRule(_localctx, 4, RULE_factbase);
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(98);
			match(FACTDATABASE);
			setState(99);
			match(T__0);
			setState(103);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,6,_ctx);
			while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1 ) {
					{
					{
					setState(100);
					ignore();
					}
					} 
				}
				setState(105);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,6,_ctx);
			}
			setState(106);
			match(T__1);
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
		public TerminalNode HTN() { return getToken(htnParser.HTN, 0); }
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
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterHtn(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitHtn(this);
		}
	}

	public final HtnContext htn() throws RecognitionException {
		HtnContext _localctx = new HtnContext(_ctx, getState());
		enterRule(_localctx, 6, RULE_htn);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(108);
			match(HTN);
			setState(109);
			match(T__0);
			setState(113);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==ACTION) {
				{
				{
				setState(110);
				actions();
				}
				}
				setState(115);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(119);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==METHOD) {
				{
				{
				setState(116);
				methods();
				}
				}
				setState(121);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(122);
			match(T__1);
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
		public TerminalNode ACTION() { return getToken(htnParser.ACTION, 0); }
		public TerminalNode IDENTIFIER() { return getToken(htnParser.IDENTIFIER, 0); }
		public List<ArgumentsContext> arguments() {
			return getRuleContexts(ArgumentsContext.class);
		}
		public ArgumentsContext arguments(int i) {
			return getRuleContext(ArgumentsContext.class,i);
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
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterActions(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitActions(this);
		}
	}

	public final ActionsContext actions() throws RecognitionException {
		ActionsContext _localctx = new ActionsContext(_ctx, getState());
		enterRule(_localctx, 8, RULE_actions);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(124);
			match(ACTION);
			setState(125);
			match(IDENTIFIER);
			setState(126);
			match(T__2);
			setState(127);
			arguments();
			setState(132);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==T__3) {
				{
				{
				setState(128);
				match(T__3);
				setState(129);
				arguments();
				}
				}
				setState(134);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(135);
			match(T__4);
			setState(136);
			match(T__0);
			setState(138); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(137);
				preconditions();
				}
				}
				setState(140); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( _la==PRECONDITIONS );
			setState(143); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(142);
				effects();
				}
				}
				setState(145); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( _la==EFFECTS );
			setState(148);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==COST) {
				{
				setState(147);
				cost();
				}
			}

			setState(151);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==DURATION) {
				{
				setState(150);
				duration();
				}
			}

			setState(153);
			match(T__1);
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
		public TerminalNode PRECONDITIONS() { return getToken(htnParser.PRECONDITIONS, 0); }
		public TerminalNode SEMICOLON() { return getToken(htnParser.SEMICOLON, 0); }
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
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterPreconditions(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitPreconditions(this);
		}
	}

	public final PreconditionsContext preconditions() throws RecognitionException {
		PreconditionsContext _localctx = new PreconditionsContext(_ctx, getState());
		enterRule(_localctx, 10, RULE_preconditions);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(155);
			match(PRECONDITIONS);
			setState(156);
			match(T__0);
			setState(160);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==IDENTIFIER) {
				{
				{
				setState(157);
				expression();
				}
				}
				setState(162);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(163);
			match(T__1);
			setState(164);
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
		public TerminalNode EFFECTS() { return getToken(htnParser.EFFECTS, 0); }
		public TerminalNode SEMICOLON() { return getToken(htnParser.SEMICOLON, 0); }
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
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterEffects(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitEffects(this);
		}
	}

	public final EffectsContext effects() throws RecognitionException {
		EffectsContext _localctx = new EffectsContext(_ctx, getState());
		enterRule(_localctx, 12, RULE_effects);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(166);
			match(EFFECTS);
			setState(167);
			match(T__0);
			setState(172);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==FORALL || _la==IDENTIFIER) {
				{
				setState(170);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case FORALL:
					{
					setState(168);
					forall();
					}
					break;
				case IDENTIFIER:
					{
					setState(169);
					expression();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(174);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(175);
			match(T__1);
			setState(176);
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
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterArguments(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitArguments(this);
		}
	}

	public final ArgumentsContext arguments() throws RecognitionException {
		ArgumentsContext _localctx = new ArgumentsContext(_ctx, getState());
		enterRule(_localctx, 14, RULE_arguments);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(178);
			type();
			setState(179);
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
		public TerminalNode IDENTIFIER() { return getToken(htnParser.IDENTIFIER, 0); }
		public TypeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterType(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitType(this);
		}
	}

	public final TypeContext type() throws RecognitionException {
		TypeContext _localctx = new TypeContext(_ctx, getState());
		enterRule(_localctx, 16, RULE_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(181);
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
		public TerminalNode IDENTIFIER() { return getToken(htnParser.IDENTIFIER, 0); }
		public VarnameContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_varname; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterVarname(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitVarname(this);
		}
	}

	public final VarnameContext varname() throws RecognitionException {
		VarnameContext _localctx = new VarnameContext(_ctx, getState());
		enterRule(_localctx, 18, RULE_varname);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(183);
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
		public TerminalNode IDENTIFIER() { return getToken(htnParser.IDENTIFIER, 0); }
		public ValueContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterValue(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitValue(this);
		}
	}

	public final ValueContext value() throws RecognitionException {
		ValueContext _localctx = new ValueContext(_ctx, getState());
		enterRule(_localctx, 20, RULE_value);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(185);
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
		public TerminalNode METHOD() { return getToken(htnParser.METHOD, 0); }
		public TerminalNode IDENTIFIER() { return getToken(htnParser.IDENTIFIER, 0); }
		public List<ArgumentsContext> arguments() {
			return getRuleContexts(ArgumentsContext.class);
		}
		public ArgumentsContext arguments(int i) {
			return getRuleContext(ArgumentsContext.class,i);
		}
		public GoalContext goal() {
			return getRuleContext(GoalContext.class,0);
		}
		public List<PreconditionsContext> preconditions() {
			return getRuleContexts(PreconditionsContext.class);
		}
		public PreconditionsContext preconditions(int i) {
			return getRuleContext(PreconditionsContext.class,i);
		}
		public List<SubtaskContext> subtask() {
			return getRuleContexts(SubtaskContext.class);
		}
		public SubtaskContext subtask(int i) {
			return getRuleContext(SubtaskContext.class,i);
		}
		public MethodsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_methods; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterMethods(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitMethods(this);
		}
	}

	public final MethodsContext methods() throws RecognitionException {
		MethodsContext _localctx = new MethodsContext(_ctx, getState());
		enterRule(_localctx, 22, RULE_methods);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(187);
			match(METHOD);
			setState(188);
			match(IDENTIFIER);
			setState(189);
			match(T__2);
			setState(190);
			arguments();
			setState(195);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==T__3) {
				{
				{
				setState(191);
				match(T__3);
				setState(192);
				arguments();
				}
				}
				setState(197);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(198);
			match(T__4);
			setState(199);
			match(T__0);
			setState(200);
			goal();
			setState(206); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(201);
				match(T__0);
				setState(202);
				preconditions();
				setState(203);
				subtask();
				setState(204);
				match(T__1);
				}
				}
				setState(208); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( _la==T__0 );
			setState(210);
			match(T__1);
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
		public TerminalNode SUBTASK() { return getToken(htnParser.SUBTASK, 0); }
		public TerminalNode SEMICOLON() { return getToken(htnParser.SEMICOLON, 0); }
		public List<SubselectionContext> subselection() {
			return getRuleContexts(SubselectionContext.class);
		}
		public SubselectionContext subselection(int i) {
			return getRuleContext(SubselectionContext.class,i);
		}
		public List<OrderContext> order() {
			return getRuleContexts(OrderContext.class);
		}
		public OrderContext order(int i) {
			return getRuleContext(OrderContext.class,i);
		}
		public SubtaskContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_subtask; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterSubtask(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitSubtask(this);
		}
	}

	public final SubtaskContext subtask() throws RecognitionException {
		SubtaskContext _localctx = new SubtaskContext(_ctx, getState());
		enterRule(_localctx, 24, RULE_subtask);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(212);
			match(SUBTASK);
			setState(213);
			match(T__0);
			setState(218);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==IDENTIFIER || _la==NUMBER) {
				{
				setState(216);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case IDENTIFIER:
					{
					setState(214);
					subselection();
					}
					break;
				case NUMBER:
					{
					setState(215);
					order();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(220);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(221);
			match(T__1);
			setState(222);
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
		public TerminalNode GOAL() { return getToken(htnParser.GOAL, 0); }
		public TerminalNode SEMICOLON() { return getToken(htnParser.SEMICOLON, 0); }
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
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterGoal(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitGoal(this);
		}
	}

	public final GoalContext goal() throws RecognitionException {
		GoalContext _localctx = new GoalContext(_ctx, getState());
		enterRule(_localctx, 26, RULE_goal);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(224);
			match(GOAL);
			setState(225);
			match(T__0);
			setState(227); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(226);
				expression();
				}
				}
				setState(229); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( _la==IDENTIFIER );
			setState(231);
			match(T__1);
			setState(232);
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
		public TerminalNode COMMENT() { return getToken(htnParser.COMMENT, 0); }
		public TerminalNode LINE_COMMENT() { return getToken(htnParser.LINE_COMMENT, 0); }
		public CommentContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_comment; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterComment(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitComment(this);
		}
	}

	public final CommentContext comment() throws RecognitionException {
		CommentContext _localctx = new CommentContext(_ctx, getState());
		enterRule(_localctx, 28, RULE_comment);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(234);
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
		public TerminalNode SEMICOLON() { return getToken(htnParser.SEMICOLON, 0); }
		public IgnoreContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_ignore; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterIgnore(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitIgnore(this);
		}
	}

	public final IgnoreContext ignore() throws RecognitionException {
		IgnoreContext _localctx = new IgnoreContext(_ctx, getState());
		enterRule(_localctx, 30, RULE_ignore);
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(239);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,22,_ctx);
			while ( _alt!=1 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1+1 ) {
					{
					{
					setState(236);
					matchWildcard();
					}
					} 
				}
				setState(241);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,22,_ctx);
			}
			setState(242);
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
		public TerminalNode POINT() { return getToken(htnParser.POINT, 0); }
		public ValueContext value() {
			return getRuleContext(ValueContext.class,0);
		}
		public AttributContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attribut; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterAttribut(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitAttribut(this);
		}
	}

	public final AttributContext attribut() throws RecognitionException {
		AttributContext _localctx = new AttributContext(_ctx, getState());
		enterRule(_localctx, 32, RULE_attribut);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(244);
			varname();
			setState(247);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==POINT) {
				{
				setState(245);
				match(POINT);
				setState(246);
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
	public static class OperatorContext extends ParserRuleContext {
		public TerminalNode ADD_IN_SET() { return getToken(htnParser.ADD_IN_SET, 0); }
		public TerminalNode REMOVE_FROM_SET() { return getToken(htnParser.REMOVE_FROM_SET, 0); }
		public TerminalNode EQUAL() { return getToken(htnParser.EQUAL, 0); }
		public TerminalNode TEST_EQUAL() { return getToken(htnParser.TEST_EQUAL, 0); }
		public TerminalNode TEST_DIFF() { return getToken(htnParser.TEST_DIFF, 0); }
		public TerminalNode TEST_SET_IN() { return getToken(htnParser.TEST_SET_IN, 0); }
		public TerminalNode TEST_SET_NOT_IN() { return getToken(htnParser.TEST_SET_NOT_IN, 0); }
		public TerminalNode SUP() { return getToken(htnParser.SUP, 0); }
		public TerminalNode SUP_EQUAL() { return getToken(htnParser.SUP_EQUAL, 0); }
		public TerminalNode INF() { return getToken(htnParser.INF, 0); }
		public TerminalNode INF_EQUAL() { return getToken(htnParser.INF_EQUAL, 0); }
		public TerminalNode SUP_TILD() { return getToken(htnParser.SUP_TILD, 0); }
		public OperatorContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_operator; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterOperator(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitOperator(this);
		}
	}

	public final OperatorContext operator() throws RecognitionException {
		OperatorContext _localctx = new OperatorContext(_ctx, getState());
		enterRule(_localctx, 34, RULE_operator);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(249);
			_la = _input.LA(1);
			if ( !((((_la) & ~0x3f) == 0 && ((1L << _la) & 8587837440L) != 0)) ) {
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
		public List<AttributContext> attribut() {
			return getRuleContexts(AttributContext.class);
		}
		public AttributContext attribut(int i) {
			return getRuleContext(AttributContext.class,i);
		}
		public OperatorContext operator() {
			return getRuleContext(OperatorContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(htnParser.SEMICOLON, 0); }
		public ExpressionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_expression; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterExpression(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitExpression(this);
		}
	}

	public final ExpressionContext expression() throws RecognitionException {
		ExpressionContext _localctx = new ExpressionContext(_ctx, getState());
		enterRule(_localctx, 36, RULE_expression);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(251);
			attribut();
			setState(252);
			operator();
			setState(253);
			attribut();
			setState(254);
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
		public TerminalNode SEMICOLON() { return getToken(htnParser.SEMICOLON, 0); }
		public SubselectionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_subselection; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterSubselection(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitSubselection(this);
		}
	}

	public final SubselectionContext subselection() throws RecognitionException {
		SubselectionContext _localctx = new SubselectionContext(_ctx, getState());
		enterRule(_localctx, 38, RULE_subselection);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(256);
			attribut();
			setState(257);
			operator();
			setState(258);
			selectcase();
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
	public static class SelectcaseContext extends ParserRuleContext {
		public TerminalNode SELECT() { return getToken(htnParser.SELECT, 0); }
		public TerminalNode IDENTIFIER() { return getToken(htnParser.IDENTIFIER, 0); }
		public ExpressionContext expression() {
			return getRuleContext(ExpressionContext.class,0);
		}
		public SelectcaseContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_selectcase; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterSelectcase(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitSelectcase(this);
		}
	}

	public final SelectcaseContext selectcase() throws RecognitionException {
		SelectcaseContext _localctx = new SelectcaseContext(_ctx, getState());
		enterRule(_localctx, 40, RULE_selectcase);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(261);
			match(SELECT);
			setState(262);
			match(T__2);
			setState(263);
			match(IDENTIFIER);
			setState(264);
			match(T__3);
			setState(265);
			match(T__0);
			setState(267);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==IDENTIFIER) {
				{
				setState(266);
				expression();
				}
			}

			setState(269);
			match(T__1);
			setState(270);
			match(T__4);
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
		public TerminalNode NUMBER() { return getToken(htnParser.NUMBER, 0); }
		public TerminalNode COLON() { return getToken(htnParser.COLON, 0); }
		public FunctionContext function() {
			return getRuleContext(FunctionContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(htnParser.SEMICOLON, 0); }
		public TestContext test() {
			return getRuleContext(TestContext.class,0);
		}
		public OrderContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_order; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterOrder(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitOrder(this);
		}
	}

	public final OrderContext order() throws RecognitionException {
		OrderContext _localctx = new OrderContext(_ctx, getState());
		enterRule(_localctx, 42, RULE_order);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(272);
			match(NUMBER);
			setState(273);
			match(COLON);
			setState(274);
			function();
			setState(276);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if ((((_la) & ~0x3f) == 0 && ((1L << _la) & 8587837440L) != 0)) {
				{
				setState(275);
				test();
				}
			}

			setState(278);
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
		public TerminalNode IDENTIFIER() { return getToken(htnParser.IDENTIFIER, 0); }
		public List<VarnameContext> varname() {
			return getRuleContexts(VarnameContext.class);
		}
		public VarnameContext varname(int i) {
			return getRuleContext(VarnameContext.class,i);
		}
		public FunctionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_function; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterFunction(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitFunction(this);
		}
	}

	public final FunctionContext function() throws RecognitionException {
		FunctionContext _localctx = new FunctionContext(_ctx, getState());
		enterRule(_localctx, 44, RULE_function);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(280);
			match(IDENTIFIER);
			setState(281);
			match(T__2);
			setState(282);
			varname();
			setState(287);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==T__3) {
				{
				{
				setState(283);
				match(T__3);
				setState(284);
				varname();
				}
				}
				setState(289);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(290);
			match(T__4);
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
	public static class TestContext extends ParserRuleContext {
		public OperatorContext operator() {
			return getRuleContext(OperatorContext.class,0);
		}
		public TerminalNode NUMBER() { return getToken(htnParser.NUMBER, 0); }
		public List<TestContext> test() {
			return getRuleContexts(TestContext.class);
		}
		public TestContext test(int i) {
			return getRuleContext(TestContext.class,i);
		}
		public TestContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_test; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterTest(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitTest(this);
		}
	}

	public final TestContext test() throws RecognitionException {
		TestContext _localctx = new TestContext(_ctx, getState());
		enterRule(_localctx, 46, RULE_test);
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(292);
			operator();
			setState(293);
			match(NUMBER);
			setState(298);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,27,_ctx);
			while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1 ) {
					{
					{
					setState(294);
					match(T__3);
					setState(295);
					test();
					}
					} 
				}
				setState(300);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,27,_ctx);
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
	public static class ForallContext extends ParserRuleContext {
		public TerminalNode FORALL() { return getToken(htnParser.FORALL, 0); }
		public ArgumentsContext arguments() {
			return getRuleContext(ArgumentsContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(htnParser.SEMICOLON, 0); }
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
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterForall(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitForall(this);
		}
	}

	public final ForallContext forall() throws RecognitionException {
		ForallContext _localctx = new ForallContext(_ctx, getState());
		enterRule(_localctx, 48, RULE_forall);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(301);
			match(FORALL);
			setState(302);
			match(T__2);
			setState(303);
			arguments();
			setState(304);
			match(T__3);
			{
			setState(305);
			match(T__0);
			setState(307);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==IDENTIFIER) {
				{
				setState(306);
				expression();
				}
			}

			setState(309);
			match(T__1);
			}
			setState(318);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==T__3) {
				{
				{
				setState(311);
				match(T__3);
				setState(312);
				match(T__0);
				setState(313);
				expression();
				setState(314);
				match(T__1);
				}
				}
				setState(320);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(321);
			match(T__4);
			setState(322);
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
		public TerminalNode COST() { return getToken(htnParser.COST, 0); }
		public TerminalNode IDENTIFIER() { return getToken(htnParser.IDENTIFIER, 0); }
		public TerminalNode SEMICOLON() { return getToken(htnParser.SEMICOLON, 0); }
		public CostContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_cost; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterCost(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitCost(this);
		}
	}

	public final CostContext cost() throws RecognitionException {
		CostContext _localctx = new CostContext(_ctx, getState());
		enterRule(_localctx, 50, RULE_cost);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(324);
			match(COST);
			setState(325);
			match(T__0);
			setState(326);
			match(IDENTIFIER);
			setState(327);
			match(T__5);
			setState(328);
			match(T__1);
			setState(329);
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
		public TerminalNode DURATION() { return getToken(htnParser.DURATION, 0); }
		public TerminalNode IDENTIFIER() { return getToken(htnParser.IDENTIFIER, 0); }
		public TerminalNode SEMICOLON() { return getToken(htnParser.SEMICOLON, 0); }
		public DurationContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_duration; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).enterDuration(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof htnListener ) ((htnListener)listener).exitDuration(this);
		}
	}

	public final DurationContext duration() throws RecognitionException {
		DurationContext _localctx = new DurationContext(_ctx, getState());
		enterRule(_localctx, 52, RULE_duration);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(331);
			match(DURATION);
			setState(332);
			match(T__0);
			setState(333);
			match(IDENTIFIER);
			setState(334);
			match(T__5);
			setState(335);
			match(T__1);
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

	public static final String _serializedATN =
		"\u0004\u00010\u0153\u0002\u0000\u0007\u0000\u0002\u0001\u0007\u0001\u0002"+
		"\u0002\u0007\u0002\u0002\u0003\u0007\u0003\u0002\u0004\u0007\u0004\u0002"+
		"\u0005\u0007\u0005\u0002\u0006\u0007\u0006\u0002\u0007\u0007\u0007\u0002"+
		"\b\u0007\b\u0002\t\u0007\t\u0002\n\u0007\n\u0002\u000b\u0007\u000b\u0002"+
		"\f\u0007\f\u0002\r\u0007\r\u0002\u000e\u0007\u000e\u0002\u000f\u0007\u000f"+
		"\u0002\u0010\u0007\u0010\u0002\u0011\u0007\u0011\u0002\u0012\u0007\u0012"+
		"\u0002\u0013\u0007\u0013\u0002\u0014\u0007\u0014\u0002\u0015\u0007\u0015"+
		"\u0002\u0016\u0007\u0016\u0002\u0017\u0007\u0017\u0002\u0018\u0007\u0018"+
		"\u0002\u0019\u0007\u0019\u0002\u001a\u0007\u001a\u0001\u0000\u0005\u0000"+
		"8\b\u0000\n\u0000\f\u0000;\t\u0000\u0001\u0000\u0005\u0000>\b\u0000\n"+
		"\u0000\f\u0000A\t\u0000\u0001\u0000\u0005\u0000D\b\u0000\n\u0000\f\u0000"+
		"G\t\u0000\u0001\u0000\u0001\u0000\u0005\u0000K\b\u0000\n\u0000\f\u0000"+
		"N\t\u0000\u0001\u0000\u0001\u0000\u0005\u0000R\b\u0000\n\u0000\f\u0000"+
		"U\t\u0000\u0001\u0000\u0001\u0000\u0001\u0001\u0001\u0001\u0001\u0001"+
		"\u0005\u0001\\\b\u0001\n\u0001\f\u0001_\t\u0001\u0001\u0001\u0001\u0001"+
		"\u0001\u0002\u0001\u0002\u0001\u0002\u0005\u0002f\b\u0002\n\u0002\f\u0002"+
		"i\t\u0002\u0001\u0002\u0001\u0002\u0001\u0003\u0001\u0003\u0001\u0003"+
		"\u0005\u0003p\b\u0003\n\u0003\f\u0003s\t\u0003\u0001\u0003\u0005\u0003"+
		"v\b\u0003\n\u0003\f\u0003y\t\u0003\u0001\u0003\u0001\u0003\u0001\u0004"+
		"\u0001\u0004\u0001\u0004\u0001\u0004\u0001\u0004\u0001\u0004\u0005\u0004"+
		"\u0083\b\u0004\n\u0004\f\u0004\u0086\t\u0004\u0001\u0004\u0001\u0004\u0001"+
		"\u0004\u0004\u0004\u008b\b\u0004\u000b\u0004\f\u0004\u008c\u0001\u0004"+
		"\u0004\u0004\u0090\b\u0004\u000b\u0004\f\u0004\u0091\u0001\u0004\u0003"+
		"\u0004\u0095\b\u0004\u0001\u0004\u0003\u0004\u0098\b\u0004\u0001\u0004"+
		"\u0001\u0004\u0001\u0005\u0001\u0005\u0001\u0005\u0005\u0005\u009f\b\u0005"+
		"\n\u0005\f\u0005\u00a2\t\u0005\u0001\u0005\u0001\u0005\u0001\u0005\u0001"+
		"\u0006\u0001\u0006\u0001\u0006\u0001\u0006\u0005\u0006\u00ab\b\u0006\n"+
		"\u0006\f\u0006\u00ae\t\u0006\u0001\u0006\u0001\u0006\u0001\u0006\u0001"+
		"\u0007\u0001\u0007\u0001\u0007\u0001\b\u0001\b\u0001\t\u0001\t\u0001\n"+
		"\u0001\n\u0001\u000b\u0001\u000b\u0001\u000b\u0001\u000b\u0001\u000b\u0001"+
		"\u000b\u0005\u000b\u00c2\b\u000b\n\u000b\f\u000b\u00c5\t\u000b\u0001\u000b"+
		"\u0001\u000b\u0001\u000b\u0001\u000b\u0001\u000b\u0001\u000b\u0001\u000b"+
		"\u0001\u000b\u0004\u000b\u00cf\b\u000b\u000b\u000b\f\u000b\u00d0\u0001"+
		"\u000b\u0001\u000b\u0001\f\u0001\f\u0001\f\u0001\f\u0005\f\u00d9\b\f\n"+
		"\f\f\f\u00dc\t\f\u0001\f\u0001\f\u0001\f\u0001\r\u0001\r\u0001\r\u0004"+
		"\r\u00e4\b\r\u000b\r\f\r\u00e5\u0001\r\u0001\r\u0001\r\u0001\u000e\u0001"+
		"\u000e\u0001\u000f\u0005\u000f\u00ee\b\u000f\n\u000f\f\u000f\u00f1\t\u000f"+
		"\u0001\u000f\u0001\u000f\u0001\u0010\u0001\u0010\u0001\u0010\u0003\u0010"+
		"\u00f8\b\u0010\u0001\u0011\u0001\u0011\u0001\u0012\u0001\u0012\u0001\u0012"+
		"\u0001\u0012\u0001\u0012\u0001\u0013\u0001\u0013\u0001\u0013\u0001\u0013"+
		"\u0001\u0013\u0001\u0014\u0001\u0014\u0001\u0014\u0001\u0014\u0001\u0014"+
		"\u0001\u0014\u0003\u0014\u010c\b\u0014\u0001\u0014\u0001\u0014\u0001\u0014"+
		"\u0001\u0015\u0001\u0015\u0001\u0015\u0001\u0015\u0003\u0015\u0115\b\u0015"+
		"\u0001\u0015\u0001\u0015\u0001\u0016\u0001\u0016\u0001\u0016\u0001\u0016"+
		"\u0001\u0016\u0005\u0016\u011e\b\u0016\n\u0016\f\u0016\u0121\t\u0016\u0001"+
		"\u0016\u0001\u0016\u0001\u0017\u0001\u0017\u0001\u0017\u0001\u0017\u0005"+
		"\u0017\u0129\b\u0017\n\u0017\f\u0017\u012c\t\u0017\u0001\u0018\u0001\u0018"+
		"\u0001\u0018\u0001\u0018\u0001\u0018\u0001\u0018\u0003\u0018\u0134\b\u0018"+
		"\u0001\u0018\u0001\u0018\u0001\u0018\u0001\u0018\u0001\u0018\u0001\u0018"+
		"\u0001\u0018\u0005\u0018\u013d\b\u0018\n\u0018\f\u0018\u0140\t\u0018\u0001"+
		"\u0018\u0001\u0018\u0001\u0018\u0001\u0019\u0001\u0019\u0001\u0019\u0001"+
		"\u0019\u0001\u0019\u0001\u0019\u0001\u0019\u0001\u001a\u0001\u001a\u0001"+
		"\u001a\u0001\u001a\u0001\u001a\u0001\u001a\u0001\u001a\u0001\u001a\u0001"+
		"\u00ef\u0000\u001b\u0000\u0002\u0004\u0006\b\n\f\u000e\u0010\u0012\u0014"+
		"\u0016\u0018\u001a\u001c\u001e \"$&(*,.024\u0000\u0002\u0001\u0000*+\u0001"+
		"\u0000\u0015 \u0155\u00009\u0001\u0000\u0000\u0000\u0002X\u0001\u0000"+
		"\u0000\u0000\u0004b\u0001\u0000\u0000\u0000\u0006l\u0001\u0000\u0000\u0000"+
		"\b|\u0001\u0000\u0000\u0000\n\u009b\u0001\u0000\u0000\u0000\f\u00a6\u0001"+
		"\u0000\u0000\u0000\u000e\u00b2\u0001\u0000\u0000\u0000\u0010\u00b5\u0001"+
		"\u0000\u0000\u0000\u0012\u00b7\u0001\u0000\u0000\u0000\u0014\u00b9\u0001"+
		"\u0000\u0000\u0000\u0016\u00bb\u0001\u0000\u0000\u0000\u0018\u00d4\u0001"+
		"\u0000\u0000\u0000\u001a\u00e0\u0001\u0000\u0000\u0000\u001c\u00ea\u0001"+
		"\u0000\u0000\u0000\u001e\u00ef\u0001\u0000\u0000\u0000 \u00f4\u0001\u0000"+
		"\u0000\u0000\"\u00f9\u0001\u0000\u0000\u0000$\u00fb\u0001\u0000\u0000"+
		"\u0000&\u0100\u0001\u0000\u0000\u0000(\u0105\u0001\u0000\u0000\u0000*"+
		"\u0110\u0001\u0000\u0000\u0000,\u0118\u0001\u0000\u0000\u0000.\u0124\u0001"+
		"\u0000\u0000\u00000\u012d\u0001\u0000\u0000\u00002\u0144\u0001\u0000\u0000"+
		"\u00004\u014b\u0001\u0000\u0000\u000068\u0003\u001c\u000e\u000076\u0001"+
		"\u0000\u0000\u00008;\u0001\u0000\u0000\u000097\u0001\u0000\u0000\u0000"+
		"9:\u0001\u0000\u0000\u0000:?\u0001\u0000\u0000\u0000;9\u0001\u0000\u0000"+
		"\u0000<>\u0003\u0004\u0002\u0000=<\u0001\u0000\u0000\u0000>A\u0001\u0000"+
		"\u0000\u0000?=\u0001\u0000\u0000\u0000?@\u0001\u0000\u0000\u0000@E\u0001"+
		"\u0000\u0000\u0000A?\u0001\u0000\u0000\u0000BD\u0003\u001c\u000e\u0000"+
		"CB\u0001\u0000\u0000\u0000DG\u0001\u0000\u0000\u0000EC\u0001\u0000\u0000"+
		"\u0000EF\u0001\u0000\u0000\u0000FH\u0001\u0000\u0000\u0000GE\u0001\u0000"+
		"\u0000\u0000HL\u0003\u0006\u0003\u0000IK\u0003\u001c\u000e\u0000JI\u0001"+
		"\u0000\u0000\u0000KN\u0001\u0000\u0000\u0000LJ\u0001\u0000\u0000\u0000"+
		"LM\u0001\u0000\u0000\u0000MO\u0001\u0000\u0000\u0000NL\u0001\u0000\u0000"+
		"\u0000OS\u0003\u0002\u0001\u0000PR\u0003\u001c\u000e\u0000QP\u0001\u0000"+
		"\u0000\u0000RU\u0001\u0000\u0000\u0000SQ\u0001\u0000\u0000\u0000ST\u0001"+
		"\u0000\u0000\u0000TV\u0001\u0000\u0000\u0000US\u0001\u0000\u0000\u0000"+
		"VW\u0005\u0000\u0000\u0001W\u0001\u0001\u0000\u0000\u0000XY\u0005\u000e"+
		"\u0000\u0000Y]\u0005\u0001\u0000\u0000Z\\\u0003\u001e\u000f\u0000[Z\u0001"+
		"\u0000\u0000\u0000\\_\u0001\u0000\u0000\u0000][\u0001\u0000\u0000\u0000"+
		"]^\u0001\u0000\u0000\u0000^`\u0001\u0000\u0000\u0000_]\u0001\u0000\u0000"+
		"\u0000`a\u0005\u0002\u0000\u0000a\u0003\u0001\u0000\u0000\u0000bc\u0005"+
		"\u000f\u0000\u0000cg\u0005\u0001\u0000\u0000df\u0003\u001e\u000f\u0000"+
		"ed\u0001\u0000\u0000\u0000fi\u0001\u0000\u0000\u0000ge\u0001\u0000\u0000"+
		"\u0000gh\u0001\u0000\u0000\u0000hj\u0001\u0000\u0000\u0000ig\u0001\u0000"+
		"\u0000\u0000jk\u0005\u0002\u0000\u0000k\u0005\u0001\u0000\u0000\u0000"+
		"lm\u0005\u0007\u0000\u0000mq\u0005\u0001\u0000\u0000np\u0003\b\u0004\u0000"+
		"on\u0001\u0000\u0000\u0000ps\u0001\u0000\u0000\u0000qo\u0001\u0000\u0000"+
		"\u0000qr\u0001\u0000\u0000\u0000rw\u0001\u0000\u0000\u0000sq\u0001\u0000"+
		"\u0000\u0000tv\u0003\u0016\u000b\u0000ut\u0001\u0000\u0000\u0000vy\u0001"+
		"\u0000\u0000\u0000wu\u0001\u0000\u0000\u0000wx\u0001\u0000\u0000\u0000"+
		"xz\u0001\u0000\u0000\u0000yw\u0001\u0000\u0000\u0000z{\u0005\u0002\u0000"+
		"\u0000{\u0007\u0001\u0000\u0000\u0000|}\u0005\b\u0000\u0000}~\u0005,\u0000"+
		"\u0000~\u007f\u0005\u0003\u0000\u0000\u007f\u0084\u0003\u000e\u0007\u0000"+
		"\u0080\u0081\u0005\u0004\u0000\u0000\u0081\u0083\u0003\u000e\u0007\u0000"+
		"\u0082\u0080\u0001\u0000\u0000\u0000\u0083\u0086\u0001\u0000\u0000\u0000"+
		"\u0084\u0082\u0001\u0000\u0000\u0000\u0084\u0085\u0001\u0000\u0000\u0000"+
		"\u0085\u0087\u0001\u0000\u0000\u0000\u0086\u0084\u0001\u0000\u0000\u0000"+
		"\u0087\u0088\u0005\u0005\u0000\u0000\u0088\u008a\u0005\u0001\u0000\u0000"+
		"\u0089\u008b\u0003\n\u0005\u0000\u008a\u0089\u0001\u0000\u0000\u0000\u008b"+
		"\u008c\u0001\u0000\u0000\u0000\u008c\u008a\u0001\u0000\u0000\u0000\u008c"+
		"\u008d\u0001\u0000\u0000\u0000\u008d\u008f\u0001\u0000\u0000\u0000\u008e"+
		"\u0090\u0003\f\u0006\u0000\u008f\u008e\u0001\u0000\u0000\u0000\u0090\u0091"+
		"\u0001\u0000\u0000\u0000\u0091\u008f\u0001\u0000\u0000\u0000\u0091\u0092"+
		"\u0001\u0000\u0000\u0000\u0092\u0094\u0001\u0000\u0000\u0000\u0093\u0095"+
		"\u00032\u0019\u0000\u0094\u0093\u0001\u0000\u0000\u0000\u0094\u0095\u0001"+
		"\u0000\u0000\u0000\u0095\u0097\u0001\u0000\u0000\u0000\u0096\u0098\u0003"+
		"4\u001a\u0000\u0097\u0096\u0001\u0000\u0000\u0000\u0097\u0098\u0001\u0000"+
		"\u0000\u0000\u0098\u0099\u0001\u0000\u0000\u0000\u0099\u009a\u0005\u0002"+
		"\u0000\u0000\u009a\t\u0001\u0000\u0000\u0000\u009b\u009c\u0005\t\u0000"+
		"\u0000\u009c\u00a0\u0005\u0001\u0000\u0000\u009d\u009f\u0003$\u0012\u0000"+
		"\u009e\u009d\u0001\u0000\u0000\u0000\u009f\u00a2\u0001\u0000\u0000\u0000"+
		"\u00a0\u009e\u0001\u0000\u0000\u0000\u00a0\u00a1\u0001\u0000\u0000\u0000"+
		"\u00a1\u00a3\u0001\u0000\u0000\u0000\u00a2\u00a0\u0001\u0000\u0000\u0000"+
		"\u00a3\u00a4\u0005\u0002\u0000\u0000\u00a4\u00a5\u0005%\u0000\u0000\u00a5"+
		"\u000b\u0001\u0000\u0000\u0000\u00a6\u00a7\u0005\n\u0000\u0000\u00a7\u00ac"+
		"\u0005\u0001\u0000\u0000\u00a8\u00ab\u00030\u0018\u0000\u00a9\u00ab\u0003"+
		"$\u0012\u0000\u00aa\u00a8\u0001\u0000\u0000\u0000\u00aa\u00a9\u0001\u0000"+
		"\u0000\u0000\u00ab\u00ae\u0001\u0000\u0000\u0000\u00ac\u00aa\u0001\u0000"+
		"\u0000\u0000\u00ac\u00ad\u0001\u0000\u0000\u0000\u00ad\u00af\u0001\u0000"+
		"\u0000\u0000\u00ae\u00ac\u0001\u0000\u0000\u0000\u00af\u00b0\u0005\u0002"+
		"\u0000\u0000\u00b0\u00b1\u0005%\u0000\u0000\u00b1\r\u0001\u0000\u0000"+
		"\u0000\u00b2\u00b3\u0003\u0010\b\u0000\u00b3\u00b4\u0003\u0012\t\u0000"+
		"\u00b4\u000f\u0001\u0000\u0000\u0000\u00b5\u00b6\u0005,\u0000\u0000\u00b6"+
		"\u0011\u0001\u0000\u0000\u0000\u00b7\u00b8\u0005,\u0000\u0000\u00b8\u0013"+
		"\u0001\u0000\u0000\u0000\u00b9\u00ba\u0005,\u0000\u0000\u00ba\u0015\u0001"+
		"\u0000\u0000\u0000\u00bb\u00bc\u0005\u000b\u0000\u0000\u00bc\u00bd\u0005"+
		",\u0000\u0000\u00bd\u00be\u0005\u0003\u0000\u0000\u00be\u00c3\u0003\u000e"+
		"\u0007\u0000\u00bf\u00c0\u0005\u0004\u0000\u0000\u00c0\u00c2\u0003\u000e"+
		"\u0007\u0000\u00c1\u00bf\u0001\u0000\u0000\u0000\u00c2\u00c5\u0001\u0000"+
		"\u0000\u0000\u00c3\u00c1\u0001\u0000\u0000\u0000\u00c3\u00c4\u0001\u0000"+
		"\u0000\u0000\u00c4\u00c6\u0001\u0000\u0000\u0000\u00c5\u00c3\u0001\u0000"+
		"\u0000\u0000\u00c6\u00c7\u0005\u0005\u0000\u0000\u00c7\u00c8\u0005\u0001"+
		"\u0000\u0000\u00c8\u00ce\u0003\u001a\r\u0000\u00c9\u00ca\u0005\u0001\u0000"+
		"\u0000\u00ca\u00cb\u0003\n\u0005\u0000\u00cb\u00cc\u0003\u0018\f\u0000"+
		"\u00cc\u00cd\u0005\u0002\u0000\u0000\u00cd\u00cf\u0001\u0000\u0000\u0000"+
		"\u00ce\u00c9\u0001\u0000\u0000\u0000\u00cf\u00d0\u0001\u0000\u0000\u0000"+
		"\u00d0\u00ce\u0001\u0000\u0000\u0000\u00d0\u00d1\u0001\u0000\u0000\u0000"+
		"\u00d1\u00d2\u0001\u0000\u0000\u0000\u00d2\u00d3\u0005\u0002\u0000\u0000"+
		"\u00d3\u0017\u0001\u0000\u0000\u0000\u00d4\u00d5\u0005\r\u0000\u0000\u00d5"+
		"\u00da\u0005\u0001\u0000\u0000\u00d6\u00d9\u0003&\u0013\u0000\u00d7\u00d9"+
		"\u0003*\u0015\u0000\u00d8\u00d6\u0001\u0000\u0000\u0000\u00d8\u00d7\u0001"+
		"\u0000\u0000\u0000\u00d9\u00dc\u0001\u0000\u0000\u0000\u00da\u00d8\u0001"+
		"\u0000\u0000\u0000\u00da\u00db\u0001\u0000\u0000\u0000\u00db\u00dd\u0001"+
		"\u0000\u0000\u0000\u00dc\u00da\u0001\u0000\u0000\u0000\u00dd\u00de\u0005"+
		"\u0002\u0000\u0000\u00de\u00df\u0005%\u0000\u0000\u00df\u0019\u0001\u0000"+
		"\u0000\u0000\u00e0\u00e1\u0005\f\u0000\u0000\u00e1\u00e3\u0005\u0001\u0000"+
		"\u0000\u00e2\u00e4\u0003$\u0012\u0000\u00e3\u00e2\u0001\u0000\u0000\u0000"+
		"\u00e4\u00e5\u0001\u0000\u0000\u0000\u00e5\u00e3\u0001\u0000\u0000\u0000"+
		"\u00e5\u00e6\u0001\u0000\u0000\u0000\u00e6\u00e7\u0001\u0000\u0000\u0000"+
		"\u00e7\u00e8\u0005\u0002\u0000\u0000\u00e8\u00e9\u0005%\u0000\u0000\u00e9"+
		"\u001b\u0001\u0000\u0000\u0000\u00ea\u00eb\u0007\u0000\u0000\u0000\u00eb"+
		"\u001d\u0001\u0000\u0000\u0000\u00ec\u00ee\t\u0000\u0000\u0000\u00ed\u00ec"+
		"\u0001\u0000\u0000\u0000\u00ee\u00f1\u0001\u0000\u0000\u0000\u00ef\u00f0"+
		"\u0001\u0000\u0000\u0000\u00ef\u00ed\u0001\u0000\u0000\u0000\u00f0\u00f2"+
		"\u0001\u0000\u0000\u0000\u00f1\u00ef\u0001\u0000\u0000\u0000\u00f2\u00f3"+
		"\u0005%\u0000\u0000\u00f3\u001f\u0001\u0000\u0000\u0000\u00f4\u00f7\u0003"+
		"\u0012\t\u0000\u00f5\u00f6\u0005&\u0000\u0000\u00f6\u00f8\u0003\u0014"+
		"\n\u0000\u00f7\u00f5\u0001\u0000\u0000\u0000\u00f7\u00f8\u0001\u0000\u0000"+
		"\u0000\u00f8!\u0001\u0000\u0000\u0000\u00f9\u00fa\u0007\u0001\u0000\u0000"+
		"\u00fa#\u0001\u0000\u0000\u0000\u00fb\u00fc\u0003 \u0010\u0000\u00fc\u00fd"+
		"\u0003\"\u0011\u0000\u00fd\u00fe\u0003 \u0010\u0000\u00fe\u00ff\u0005"+
		"%\u0000\u0000\u00ff%\u0001\u0000\u0000\u0000\u0100\u0101\u0003 \u0010"+
		"\u0000\u0101\u0102\u0003\"\u0011\u0000\u0102\u0103\u0003(\u0014\u0000"+
		"\u0103\u0104\u0005%\u0000\u0000\u0104\'\u0001\u0000\u0000\u0000\u0105"+
		"\u0106\u0005\u0013\u0000\u0000\u0106\u0107\u0005\u0003\u0000\u0000\u0107"+
		"\u0108\u0005,\u0000\u0000\u0108\u0109\u0005\u0004\u0000\u0000\u0109\u010b"+
		"\u0005\u0001\u0000\u0000\u010a\u010c\u0003$\u0012\u0000\u010b\u010a\u0001"+
		"\u0000\u0000\u0000\u010b\u010c\u0001\u0000\u0000\u0000\u010c\u010d\u0001"+
		"\u0000\u0000\u0000\u010d\u010e\u0005\u0002\u0000\u0000\u010e\u010f\u0005"+
		"\u0005\u0000\u0000\u010f)\u0001\u0000\u0000\u0000\u0110\u0111\u00050\u0000"+
		"\u0000\u0111\u0112\u0005(\u0000\u0000\u0112\u0114\u0003,\u0016\u0000\u0113"+
		"\u0115\u0003.\u0017\u0000\u0114\u0113\u0001\u0000\u0000\u0000\u0114\u0115"+
		"\u0001\u0000\u0000\u0000\u0115\u0116\u0001\u0000\u0000\u0000\u0116\u0117"+
		"\u0005%\u0000\u0000\u0117+\u0001\u0000\u0000\u0000\u0118\u0119\u0005,"+
		"\u0000\u0000\u0119\u011a\u0005\u0003\u0000\u0000\u011a\u011f\u0003\u0012"+
		"\t\u0000\u011b\u011c\u0005\u0004\u0000\u0000\u011c\u011e\u0003\u0012\t"+
		"\u0000\u011d\u011b\u0001\u0000\u0000\u0000\u011e\u0121\u0001\u0000\u0000"+
		"\u0000\u011f\u011d\u0001\u0000\u0000\u0000\u011f\u0120\u0001\u0000\u0000"+
		"\u0000\u0120\u0122\u0001\u0000\u0000\u0000\u0121\u011f\u0001\u0000\u0000"+
		"\u0000\u0122\u0123\u0005\u0005\u0000\u0000\u0123-\u0001\u0000\u0000\u0000"+
		"\u0124\u0125\u0003\"\u0011\u0000\u0125\u012a\u00050\u0000\u0000\u0126"+
		"\u0127\u0005\u0004\u0000\u0000\u0127\u0129\u0003.\u0017\u0000\u0128\u0126"+
		"\u0001\u0000\u0000\u0000\u0129\u012c\u0001\u0000\u0000\u0000\u012a\u0128"+
		"\u0001\u0000\u0000\u0000\u012a\u012b\u0001\u0000\u0000\u0000\u012b/\u0001"+
		"\u0000\u0000\u0000\u012c\u012a\u0001\u0000\u0000\u0000\u012d\u012e\u0005"+
		"\u0010\u0000\u0000\u012e\u012f\u0005\u0003\u0000\u0000\u012f\u0130\u0003"+
		"\u000e\u0007\u0000\u0130\u0131\u0005\u0004\u0000\u0000\u0131\u0133\u0005"+
		"\u0001\u0000\u0000\u0132\u0134\u0003$\u0012\u0000\u0133\u0132\u0001\u0000"+
		"\u0000\u0000\u0133\u0134\u0001\u0000\u0000\u0000\u0134\u0135\u0001\u0000"+
		"\u0000\u0000\u0135\u0136\u0005\u0002\u0000\u0000\u0136\u013e\u0001\u0000"+
		"\u0000\u0000\u0137\u0138\u0005\u0004\u0000\u0000\u0138\u0139\u0005\u0001"+
		"\u0000\u0000\u0139\u013a\u0003$\u0012\u0000\u013a\u013b\u0005\u0002\u0000"+
		"\u0000\u013b\u013d\u0001\u0000\u0000\u0000\u013c\u0137\u0001\u0000\u0000"+
		"\u0000\u013d\u0140\u0001\u0000\u0000\u0000\u013e\u013c\u0001\u0000\u0000"+
		"\u0000\u013e\u013f\u0001\u0000\u0000\u0000\u013f\u0141\u0001\u0000\u0000"+
		"\u0000\u0140\u013e\u0001\u0000\u0000\u0000\u0141\u0142\u0005\u0005\u0000"+
		"\u0000\u0142\u0143\u0005%\u0000\u0000\u01431\u0001\u0000\u0000\u0000\u0144"+
		"\u0145\u0005\u0011\u0000\u0000\u0145\u0146\u0005\u0001\u0000\u0000\u0146"+
		"\u0147\u0005,\u0000\u0000\u0147\u0148\u0005\u0006\u0000\u0000\u0148\u0149"+
		"\u0005\u0002\u0000\u0000\u0149\u014a\u0005%\u0000\u0000\u014a3\u0001\u0000"+
		"\u0000\u0000\u014b\u014c\u0005\u0012\u0000\u0000\u014c\u014d\u0005\u0001"+
		"\u0000\u0000\u014d\u014e\u0005,\u0000\u0000\u014e\u014f\u0005\u0006\u0000"+
		"\u0000\u014f\u0150\u0005\u0002\u0000\u0000\u0150\u0151\u0005%\u0000\u0000"+
		"\u01515\u0001\u0000\u0000\u0000\u001e9?ELS]gqw\u0084\u008c\u0091\u0094"+
		"\u0097\u00a0\u00aa\u00ac\u00c3\u00d0\u00d8\u00da\u00e5\u00ef\u00f7\u010b"+
		"\u0114\u011f\u012a\u0133\u013e";
	public static final ATN _ATN =
		new ATNDeserializer().deserialize(_serializedATN.toCharArray());
	static {
		_decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
		for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
			_decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
		}
	}
}