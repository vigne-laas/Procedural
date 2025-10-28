// Generated from /home/avigne/Projets/Procedural/catkin_ws/src/Procedural/include/procedural/old/reader/grammar/htn.g4 by ANTLR 4.13.1
import org.antlr.v4.runtime.tree.ParseTreeListener;

/**
 * This interface defines a complete listener for a parse tree produced by
 * {@link htnParser}.
 */
public interface htnListener extends ParseTreeListener {
	/**
	 * Enter a parse tree produced by {@link htnParser#hatp}.
	 * @param ctx the parse tree
	 */
	void enterHatp(htnParser.HatpContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#hatp}.
	 * @param ctx the parse tree
	 */
	void exitHatp(htnParser.HatpContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#timepart}.
	 * @param ctx the parse tree
	 */
	void enterTimepart(htnParser.TimepartContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#timepart}.
	 * @param ctx the parse tree
	 */
	void exitTimepart(htnParser.TimepartContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#factbase}.
	 * @param ctx the parse tree
	 */
	void enterFactbase(htnParser.FactbaseContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#factbase}.
	 * @param ctx the parse tree
	 */
	void exitFactbase(htnParser.FactbaseContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#htn}.
	 * @param ctx the parse tree
	 */
	void enterHtn(htnParser.HtnContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#htn}.
	 * @param ctx the parse tree
	 */
	void exitHtn(htnParser.HtnContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#actions}.
	 * @param ctx the parse tree
	 */
	void enterActions(htnParser.ActionsContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#actions}.
	 * @param ctx the parse tree
	 */
	void exitActions(htnParser.ActionsContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#preconditions}.
	 * @param ctx the parse tree
	 */
	void enterPreconditions(htnParser.PreconditionsContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#preconditions}.
	 * @param ctx the parse tree
	 */
	void exitPreconditions(htnParser.PreconditionsContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#effects}.
	 * @param ctx the parse tree
	 */
	void enterEffects(htnParser.EffectsContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#effects}.
	 * @param ctx the parse tree
	 */
	void exitEffects(htnParser.EffectsContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#arguments}.
	 * @param ctx the parse tree
	 */
	void enterArguments(htnParser.ArgumentsContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#arguments}.
	 * @param ctx the parse tree
	 */
	void exitArguments(htnParser.ArgumentsContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#type}.
	 * @param ctx the parse tree
	 */
	void enterType(htnParser.TypeContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#type}.
	 * @param ctx the parse tree
	 */
	void exitType(htnParser.TypeContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#varname}.
	 * @param ctx the parse tree
	 */
	void enterVarname(htnParser.VarnameContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#varname}.
	 * @param ctx the parse tree
	 */
	void exitVarname(htnParser.VarnameContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#value}.
	 * @param ctx the parse tree
	 */
	void enterValue(htnParser.ValueContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#value}.
	 * @param ctx the parse tree
	 */
	void exitValue(htnParser.ValueContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#methods}.
	 * @param ctx the parse tree
	 */
	void enterMethods(htnParser.MethodsContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#methods}.
	 * @param ctx the parse tree
	 */
	void exitMethods(htnParser.MethodsContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#subtask}.
	 * @param ctx the parse tree
	 */
	void enterSubtask(htnParser.SubtaskContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#subtask}.
	 * @param ctx the parse tree
	 */
	void exitSubtask(htnParser.SubtaskContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#goal}.
	 * @param ctx the parse tree
	 */
	void enterGoal(htnParser.GoalContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#goal}.
	 * @param ctx the parse tree
	 */
	void exitGoal(htnParser.GoalContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#comment}.
	 * @param ctx the parse tree
	 */
	void enterComment(htnParser.CommentContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#comment}.
	 * @param ctx the parse tree
	 */
	void exitComment(htnParser.CommentContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#ignore}.
	 * @param ctx the parse tree
	 */
	void enterIgnore(htnParser.IgnoreContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#ignore}.
	 * @param ctx the parse tree
	 */
	void exitIgnore(htnParser.IgnoreContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#attribut}.
	 * @param ctx the parse tree
	 */
	void enterAttribut(htnParser.AttributContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#attribut}.
	 * @param ctx the parse tree
	 */
	void exitAttribut(htnParser.AttributContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#operator}.
	 * @param ctx the parse tree
	 */
	void enterOperator(htnParser.OperatorContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#operator}.
	 * @param ctx the parse tree
	 */
	void exitOperator(htnParser.OperatorContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#expression}.
	 * @param ctx the parse tree
	 */
	void enterExpression(htnParser.ExpressionContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#expression}.
	 * @param ctx the parse tree
	 */
	void exitExpression(htnParser.ExpressionContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#subselection}.
	 * @param ctx the parse tree
	 */
	void enterSubselection(htnParser.SubselectionContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#subselection}.
	 * @param ctx the parse tree
	 */
	void exitSubselection(htnParser.SubselectionContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#selectcase}.
	 * @param ctx the parse tree
	 */
	void enterSelectcase(htnParser.SelectcaseContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#selectcase}.
	 * @param ctx the parse tree
	 */
	void exitSelectcase(htnParser.SelectcaseContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#order}.
	 * @param ctx the parse tree
	 */
	void enterOrder(htnParser.OrderContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#order}.
	 * @param ctx the parse tree
	 */
	void exitOrder(htnParser.OrderContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#function}.
	 * @param ctx the parse tree
	 */
	void enterFunction(htnParser.FunctionContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#function}.
	 * @param ctx the parse tree
	 */
	void exitFunction(htnParser.FunctionContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#test}.
	 * @param ctx the parse tree
	 */
	void enterTest(htnParser.TestContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#test}.
	 * @param ctx the parse tree
	 */
	void exitTest(htnParser.TestContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#forall}.
	 * @param ctx the parse tree
	 */
	void enterForall(htnParser.ForallContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#forall}.
	 * @param ctx the parse tree
	 */
	void exitForall(htnParser.ForallContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#cost}.
	 * @param ctx the parse tree
	 */
	void enterCost(htnParser.CostContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#cost}.
	 * @param ctx the parse tree
	 */
	void exitCost(htnParser.CostContext ctx);
	/**
	 * Enter a parse tree produced by {@link htnParser#duration}.
	 * @param ctx the parse tree
	 */
	void enterDuration(htnParser.DurationContext ctx);
	/**
	 * Exit a parse tree produced by {@link htnParser#duration}.
	 * @param ctx the parse tree
	 */
	void exitDuration(htnParser.DurationContext ctx);
}