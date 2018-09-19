/**
 * UCT implementation with some variations, based in paper "Bandit based Monte-Carlo Planning" and in the ICAPS 2010 
 * presentation "Monte-Carlo Planning: Basic Principles and Recent Progress"
 * 
 * @author Daniel Dias (dbdias at ime.usp.br) and Luis Rocha ( ludygrv at cecm.usp.br )
 * @version 2011-11-13
 *
 * The original UCT paper is here:
 *    L. Kocsis and C. Szepesvári.
 *    Bandit based Monte-Carlo Planning.
 *    ECML 2006.
 * 
 * The ICAPS 2010 presentation is here:
 *    A. Fern
 *    Monte-Carlo Planning: Basic Principles and Recent Progress
 *    http://videolectures.net/icaps2010_fern_mcpbprp/
 **/
package rddl.explanations.algo;

import java.math.*;
import java.util.*;

import org.apache.commons.math3.random.RandomDataGenerator;
import rddl.*;
import rddl.RDDL.*;
import rddl.policy.*;
import rddl.translate.RDDL2Format;
import util.*;

/**
 * Implements the UCT algorithm.
 */
public class UCT2 extends EnumerableStatePolicy {

	protected boolean SHOW_P1 = true;

	/**
	 * Default constructor.
	 */
	public UCT2() { }
	
	/**
	 * Initialize this class with the instance name to be solved by this algorithm. 
	 * @param instance_name Instance name to be solved by this algorithm
	 */
	public UCT2(String instance_name) {
		super(instance_name);
	}


	public UCT2(List<String> stateVariableNames, int remainingHorizons ,
				double discountFactor, List<CString> actions, INSTANCE instance ,
				RDDL2Format translation, RandomDataGenerator rand, RDDL rddl, String instance_name,
				RandomRolloutPolicy roll_policy){

		super(stateVariableNames,remainingHorizons, discountFactor, actions, instance, translation, rand, rddl, instance_name);
		this.policy = roll_policy;


	}
	
	private HashMap<BigInteger, HashMap<CString, Double>> averageRewardsPerState = new HashMap<BigInteger, HashMap<CString,Double>>();		
	private HashMap<BigInteger, HashMap<CString, Integer>> pullsPerState = new HashMap<BigInteger, HashMap<CString,Integer>>();
	
	private HashMap<BigInteger, Integer> statesOccurrences = new HashMap<BigInteger, Integer>();		
	private HashMap<BigInteger, Integer> statesUsedActions = new HashMap<BigInteger, Integer>();
	private HashSet<BigInteger> innerStates = new HashSet<BigInteger>();
	private HashMap<Pair<BigInteger,CString>, HashSet<BigInteger> > stateActionState = new HashMap<>();
	private RandomRolloutPolicy policy = null;
	private HashMap<BigInteger,State> stateMapping = new HashMap<>();
	private HashSet<BigInteger> goalStates = new HashSet<>();
	//private int search_horizon = 40;
	private boolean _debug_flag = false;

	private final double exploreConst = 40d;
	//private final double DELTA = 0.9d;
	//private final double EPSILON = 0.1d;
	private final double ROUND_TIME_LIMIT = 180; //seconds
	//private final String TIME_DIST = "quad"; 
	private final int HORIZON_MAX = 100; //seconds
	//private int W = 0;
	
	/**
	 * Compute the action used in a getAction call.
	 */
	private double actionTime(long h)
	{
		double s=0.0;
		for(int i=0;i< this.getTotalHorizons() ;i++) { s += i*i;}
		return h*h*ROUND_TIME_LIMIT/s;
	}
	
	/**
	 * Returns the best action given a state.
	 */
	@Override
	protected String getBestAction(State s0) {
		long count =0;
		long start_time = System.currentTimeMillis();
		long now_time = start_time;
		long h = this.getRemainingHorizons();
		double time_lim = 1000*actionTime(h);
		//now_time - start_time < time_lim

		while(count < 100)
		{
			State s = cloneState(s0);
			System.out.println("--------- This is were we start at root Node!!!!! :: "+ stateActionState.keySet().toString());
			double rew = this.UCT_search(s, this.getRemainingHorizons());
			now_time = System.currentTimeMillis();
			if (_debug_flag) System.out.println("At time: "+Long.toString(now_time-start_time)+"Reward"+Double.toString(rew));
			count++;
		}
		//BigInteger s0N = this.getStateDepthLabel(s0,search_horizon);
		//System.out.println("Inicial = "+ s0N.toString());
		//System.out.println( averageRewardsPerState.size());		
		//System.out.println( averageRewardsPerState.get(s0N));

		String best = getUCTAction(s0, this.getRemainingHorizons());



		System.out.println("Nvezes = " + Long.toString(count) + " best = "+best);
		return best;
	}



	private String getUCTAction(State s, Integer h) {
		BigInteger stateAsNumber = this.getStateDepthLabel(s,h);
		
		CString bestAction = null;
		double bestActionReward = Double.NEGATIVE_INFINITY;
		HashMap<CString, Double> averageRewardsPerAction = this.averageRewardsPerState.get(stateAsNumber);
		
		for (CString action : averageRewardsPerAction.keySet()) {
			// should we check if the action is Appliable ? s.checkStateActionConstraints(action_list);
			if (averageRewardsPerAction.containsKey(action) == false) break;
			double averageReward = averageRewardsPerAction.get(action);
			if ( averageReward > bestActionReward) {
				bestAction = action;
				bestActionReward = averageReward;
			}
		}				
		System.out.println("Best Reward = "+ Double.toString(bestActionReward));
		return bestAction._string;			
	}

	private boolean crossingDomainGoalState(State s){

		HashMap<PVAR_NAME,HashMap<ArrayList<LCONST>,Object>> state_value = s._state;

		HashMap<ArrayList<LCONST>,Object> lconst_value = state_value.get(new PVAR_NAME("agent-at"));
		LCONST x3 = new OBJECT_VAL("$x3");
		LCONST y3 = new OBJECT_VAL("$y3");
		ArrayList<LCONST> temp_lconsts = new ArrayList<>();
		temp_lconsts.add(x3); temp_lconsts.add(y3);
		if(lconst_value.containsKey(temp_lconsts)){
			return true;
		}else{
			return false;
		}


	}
		
	/**
	 * Execute and expands the Monte Carlo Search Tree.
	 */
	protected double UCT_search(State s, int search_horizon) {


		double reward = 0.0;
		//This is for converting state to a number.
		BigInteger stateAsNumber = this.getStateDepthLabel(s,search_horizon);
		if (search_horizon <= 0) return reward;

		//Checking if the current state is a goal state!.
		if(crossingDomainGoalState(s)){
			goalStates.add(stateAsNumber);
		}
		//Storing the state value in stateMapping!!!!!...
		if(!stateMapping.containsKey(stateAsNumber)) stateMapping.put(stateAsNumber,cloneState(s));

		try {
			//This function generates legal actions for a state.
			Map<String,ArrayList<PVAR_INST_DEF>> action_map = 
					ActionGenerator.getLegalBoolActionMap(s);

			// Incrementing states visited.
			int occur = 1;
			if (statesOccurrences.containsKey(stateAsNumber) )
				occur += statesOccurrences.get(stateAsNumber);
			statesOccurrences.put(stateAsNumber,occur);
			/////////////////////////////////////////////////////

			CString action;
			int act_ind=0;
			//Maybe something do with looping... Need to think about it.
			if (innerStates.contains(stateAsNumber) ){									
				action = UCBgetAction(stateAsNumber,action_map);
			}
			else {
				if (!statesUsedActions.containsKey(stateAsNumber))
					statesUsedActions.put(stateAsNumber,0);			
				act_ind = statesUsedActions.get(stateAsNumber);
				action = this.getLegalActions(action_map).get(act_ind);
			}
			
			ArrayList<PVAR_INST_DEF> actionList;
			actionList = action_map.get(action._string);
			/////////////////////////////////////////////////////



			int act_occur = 1;
			//First time.
			if (!this.pullsPerState.containsKey(stateAsNumber)){
				HashMap<CString, Integer> new_actionOccurrences = new HashMap<CString, Integer>();
				this.pullsPerState.put(stateAsNumber, new_actionOccurrences);
			}
			HashMap<CString, Integer> actionOccurrences = this.pullsPerState.get(stateAsNumber);
			if (actionOccurrences.containsKey(action))  act_occur += actionOccurrences.get(action);
			actionOccurrences.put(action,act_occur);
			if(SHOW_P1){

				System.out.println("Current State   ::: " + stateAsNumber.toString());
				System.out.println("Legal Actions   ::: " + action_map.toString());
				System.out.println("Action Chosen   ::: " + actionList.toString());
				System.out.println("Inner Node      ::: " + innerStates.contains(stateAsNumber));
				System.out.println("###############");
			}
			/////////////////////////////////////////////////////

			s.checkStateActionConstraints(actionList);
			s.computeNextState(actionList,_random);

			// Calculate one step reward
			reward = ((Number) s._reward.sample(new HashMap<LVAR,LCONST>(), s, this._random)).doubleValue();

			s.advanceNextState(false);

			if(crossingDomainGoalState(s)){
				goalStates.add(stateAsNumber);
			}


			BigInteger nextState = this.getStateDepthLabel(s,search_horizon-1);


			if(!stateMapping.containsKey(nextState)) stateMapping.put(nextState,cloneState(s));




			Pair<BigInteger,CString> state_action = new Pair(stateAsNumber,action);

			//<Integer, BigInteger> count_nextstate = new Pair(act_occur,nextState);
			if(stateActionState.containsKey(state_action)){
				stateActionState.get(state_action).add(nextState);
			}else{
				HashSet<BigInteger> next_state_occur = new HashSet<>();
				next_state_occur.add(nextState);
				stateActionState.put(state_action,next_state_occur);

			}

			if (innerStates.contains(stateAsNumber)){
				reward += this.getDiscountFactor() * this.UCT_search(s,search_horizon -1);
			} else{
				reward += this.getDiscountFactor() * policy.rollOut(s,10,_random);
			}
						
			if (averageRewardsPerState.containsKey(stateAsNumber) == false){
				HashMap<CString,Double> new_avg_rew = new HashMap<CString,Double>();
				averageRewardsPerState.put(stateAsNumber,new_avg_rew);
			}
			HashMap<CString,Double> avg_rew = averageRewardsPerState.get(stateAsNumber);
			if (avg_rew.containsKey(action) == false) {
				avg_rew.put(action, reward);
				act_ind++;
				statesUsedActions.put(stateAsNumber,act_ind);
				if (act_ind >= getLegalActions(action_map).size())
					innerStates.add(stateAsNumber);
			}
			//Updating the average.
			else avg_rew.put(action, avg_rew.get(action) + ( (reward - avg_rew.get(action))/act_occur) );
			
		}
		catch (EvalException e) {
			e.printStackTrace();
			System.exit(-1);
		}
		return reward;
	}
	
	/**
	 * Apply the UCB algorithm to choose an action.
	 */
	private CString UCBgetAction(BigInteger sN, Map<String,ArrayList<PVAR_INST_DEF>> action_map){
		CString bestAction = null;
		double bestScore = Double.NEGATIVE_INFINITY;
		HashMap<CString, Double> averageRewardsPerAction = this.averageRewardsPerState.get(sN);
		
		for (String action : action_map.keySet()) {
			// should we check if the action is Appliable ? s.checkStateActionConstraints(action_list);
			CString caction= new CString(action);
			double score = averageRewardsPerAction.get(new CString(action));
			score += exploreConst * Math.sqrt( Math.log(statesOccurrences.get(sN)) / pullsPerState.get(sN).get(caction) );
			if ( score > bestScore) {
				bestAction =caction ;
				bestScore = score;
			}
		}
		return bestAction;			
	}
	
	/**
	 * Clone a state to use in simulation.
	 */
	private State cloneState(State currentState) {
		
		State s = new State();
		
		s._hmPVariables = new HashMap<RDDL.PVAR_NAME, RDDL.PVARIABLE_DEF>(currentState._hmPVariables);
		s._hmTypes = new HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF>(currentState._hmTypes);
		s._hmCPFs = new HashMap<RDDL.PVAR_NAME, RDDL.CPF_DEF>(currentState._hmCPFs);
		
		s._hmObject2Consts = new HashMap<RDDL.TYPE_NAME, ArrayList<LCONST>>(currentState._hmObject2Consts);
		
		s._alStateNames = new ArrayList<RDDL.PVAR_NAME>(currentState._alStateNames);
		s._alActionNames = new ArrayList<RDDL.PVAR_NAME>(currentState._alActionNames);
		s._tmIntermNames = new TreeMap<Pair, RDDL.PVAR_NAME>(currentState._tmIntermNames);
		s._alIntermNames = new ArrayList<RDDL.PVAR_NAME>(currentState._alIntermNames);
		s._alObservNames = new ArrayList<RDDL.PVAR_NAME>(currentState._alObservNames);
		s._alNonFluentNames = new ArrayList<RDDL.PVAR_NAME>(currentState._alNonFluentNames);
		
		s._hmTypeMap = new HashMap<String, ArrayList<PVAR_NAME>>();
		for (String key : currentState._hmTypeMap.keySet()) {
			ArrayList<PVAR_NAME> value = currentState._hmTypeMap.get(key);
			s._hmTypeMap.put(key, new ArrayList<RDDL.PVAR_NAME>(value));
		}
		
		s._state = new HashMap<RDDL.PVAR_NAME, HashMap<ArrayList<LCONST>,Object>>();
		for (PVAR_NAME key : currentState._state.keySet()) {
			HashMap<ArrayList<LCONST>,Object> value = currentState._state.get(key);
			s._state.put(key, new HashMap<ArrayList<LCONST>, Object>(value));
		} 
		
		s._nonfluents = new HashMap<RDDL.PVAR_NAME, HashMap<ArrayList<LCONST>,Object>>();
		for (PVAR_NAME key : currentState._nonfluents.keySet()) {
			HashMap<ArrayList<LCONST>,Object> value = currentState._nonfluents.get(key);
			s._nonfluents.put(key, new HashMap<ArrayList<LCONST>, Object>(value));
		}
		
		s._actions = new HashMap<RDDL.PVAR_NAME, HashMap<ArrayList<LCONST>,Object>>();
		for (PVAR_NAME key : currentState._actions.keySet()) {
			HashMap<ArrayList<LCONST>,Object> value = currentState._actions.get(key);
			s._actions.put(key, new HashMap<ArrayList<LCONST>, Object>(value));
		} 
		
		s._interm = new HashMap<RDDL.PVAR_NAME, HashMap<ArrayList<LCONST>,Object>>();
		for (PVAR_NAME key : currentState._interm.keySet()) {
			HashMap<ArrayList<LCONST>,Object> value = currentState._interm.get(key);
			s._interm.put(key, new HashMap<ArrayList<LCONST>, Object>(value));
		}
		
		s._observ = new HashMap<RDDL.PVAR_NAME, HashMap<ArrayList<LCONST>,Object>>();
		for (PVAR_NAME key : currentState._observ.keySet()) {
			HashMap<ArrayList<LCONST>,Object> value = currentState._observ.get(key);
			s._observ.put(key, new HashMap<ArrayList<LCONST>, Object>(value));
		}
		
		s._alActionPreconditions = currentState._alActionPreconditions;
		s._alStateInvariants = currentState._alStateInvariants;
		s._reward = currentState._reward;
		s._nMaxNondefActions = currentState._nMaxNondefActions;
		
		s._nextState = new HashMap<RDDL.PVAR_NAME, HashMap<ArrayList<LCONST>,Object>>();
		for (PVAR_NAME key : currentState._nextState.keySet()) {
			HashMap<ArrayList<LCONST>,Object> value = currentState._nextState.get(key);
			s._nextState.put(key, new HashMap<ArrayList<LCONST>, Object>(value));
		}
		
		return s;
	}
	
	BigInteger getStateDepthLabel(State s, Integer d)
	{
		BigInteger sh = this.getStateLabel(s);
		sh = sh.multiply(BigInteger.valueOf(HORIZON_MAX));
		//System.out.println("State Part:"+sh.toString()+"depthpart"+BigInteger.valueOf(d).toString() );
		sh = sh.add(BigInteger.valueOf(d));
		
		return sh;
	}

	/**
	 * Initialize all class attributes.
	 */
	@Override
	public void roundInit(double timeLeft, int horizon, int roundNumber, int totalRounds) {
		super.roundInit(timeLeft, horizon, roundNumber, totalRounds);
		
		this.setRandSeed(System.currentTimeMillis());
		policy = new RandomRolloutPolicy(this);
	}
}
