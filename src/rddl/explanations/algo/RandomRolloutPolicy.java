package rddl.explanations.algo;


import java.math.*;
import java.util.*;

import org.apache.commons.math3.random.RandomDataGenerator;
import rddl.*;
import rddl.RDDL.*;
import rddl.policy.*;
import rddl.translate.RDDL2Format;
import util.CString;

public class RandomRolloutPolicy extends EnumerableStatePolicy {

    protected boolean SHOW_P1 = false;


    public RandomRolloutPolicy(EnumerableStatePolicy policy) {
        this(policy, null);
    }



    public RandomRolloutPolicy(List<String> stateVariableNames, int remainingHorizons ,
                         double discountFactor, List<CString> actions, INSTANCE instance ,
                         RDDL2Format translation, RandomDataGenerator rand, RDDL rddl, String instance_name, HashMap<BigInteger, String> internalPolicy ) {
        super(stateVariableNames,remainingHorizons, discountFactor, actions, instance, translation, rand, rddl, instance_name);
        if(internalPolicy!=null){
            this.internalPolicy = new HashMap<BigInteger, String>(internalPolicy);

        }else{
            this.internalPolicy = new HashMap<BigInteger, String>();
        }

    }






    private RandomRolloutPolicy(EnumerableStatePolicy policy, HashMap<BigInteger, String> internalPolicy) {
        super(policy);

        if (internalPolicy != null)
            this.internalPolicy = new HashMap<BigInteger, String>(internalPolicy);
        else
            this.internalPolicy = new HashMap<BigInteger, String>();
    }

    private HashMap<BigInteger, String> internalPolicy = null;

    public void changePolicy(State s, String action) {
        BigInteger stateAsInteger = this.getStateLabel(s);
        this.internalPolicy.put(stateAsInteger, action);
    }

    @Override
    public String getBestAction(State s) throws EvalException {
        BigInteger stateAsInteger = this.getStateLabel(s);

        Map<String,ArrayList<PVAR_INST_DEF>> action_map = ActionGenerator.getLegalBoolActionMap(s);

        if (!this.internalPolicy.containsKey(stateAsInteger)) {
            ArrayList<String> actions = new ArrayList<String>(action_map.keySet());
            this.internalPolicy.put(stateAsInteger, actions.get(this._random.nextInt(0, action_map.size() - 1)));
        }

        return this.internalPolicy.get(stateAsInteger);
    }

    protected RandomRolloutPolicy copy() {
        return new RandomRolloutPolicy(this, this.internalPolicy);
    }

    protected String getRandomAction(Map<String,ArrayList<PVAR_INST_DEF>> action_map , RandomDataGenerator rand){

        ArrayList<String> list_actions = new ArrayList<>(action_map.keySet());
        if(list_actions.size()==1){
            return list_actions.get(0);
        }

        int i = rand.nextInt(0,list_actions.size()-1);

        return list_actions.get(i);

    }

    public double rollOut(State s, int horizon , RandomDataGenerator rand) {
        /* Executing a RollOut will modify the state, you must create a copy before calling this method */
        double accum_reward = 0.0d;
        double cur_discount = 1.0;
        if(SHOW_P1){
            System.out.println("-------Random RollOut policy Started");
        }
        try {
            for (int t = horizon; t > 0; t--) {
                Map<String,ArrayList<PVAR_INST_DEF>> action_map = ActionGenerator.getLegalBoolActionMap(s);
                String selectedAction = this.getRandomAction(action_map,rand);

                ArrayList<PVAR_INST_DEF> action_list = action_map.get(selectedAction);

                s.checkStateActionConstraints(action_list);
                s.computeNextState(action_list, this._random);

                double reward = ((Number) s._reward.sample(new HashMap<LVAR,LCONST>(), s, this._random)).doubleValue();
                if(SHOW_P1){
                    System.out.println("--Action Choosen " + String.valueOf(t) + " :::" + selectedAction.toString());
                    System.out.println("--immediate Reward " + String.valueOf(t) + " ::: " + String.valueOf(reward));
                }

                accum_reward += cur_discount * reward;
                cur_discount *= this.getDiscountFactor();

                // Done with this iteration, advance to next round
                s.advanceNextState(false);
            }
        } catch (EvalException e) {
            e.printStackTrace();
            System.exit(-1);
        }
        return accum_reward;
    }

}

