package rddl.explanations.algo;

import org.apache.commons.math3.random.RandomDataGenerator;
import rddl.EvalException;
import rddl.RDDL;
import rddl.State;
import rddl.RDDL.INSTANCE;
import rddl.RDDL.*;
import rddl.parser.ParseException;
import rddl.parser.parser.*;
import java.util.ArrayList;
import rddl.explanations.algo.UCT2.*;
import rddl.translate.RDDL2Format;
import util.CString;

public class UCTPlanner {


    public UCTPlanner(){}


    public static void initialzeRDDL(String rddl_filename, String instance_name) throws EvalException {
        String RDDL_FILENAME = rddl_filename;
        String instanceName = instance_name;

        //////////These are domain value types initialized.
        RDDL rddl = null;
        State      state = null;
        INSTANCE   instance =  null;
        NONFLUENTS nonFluents = null;
        DOMAIN     domain = null;
        Integer randomSeed = 1 ;

        rddl = new RDDL(RDDL_FILENAME);
        if (!rddl._tmInstanceNodes.containsKey(instanceName)) {
            System.out.println("Instance name '" + instanceName + "\nPossible choices: " + rddl._tmInstanceNodes.keySet());
            System.exit(1);
        }
        state = new State();
        instance = rddl._tmInstanceNodes.get(instanceName);
        if (instance._sNonFluents != null) {
            nonFluents = rddl._tmNonFluentNodes.get(instance._sNonFluents);
        }
        domain = rddl._tmDomainNodes.get(instance._sDomain);
        if (nonFluents != null && !instance._sDomain.equals(nonFluents._sDomain)) {
            System.err.println("Domain name of instance and fluents do not match: " +
                    instance._sDomain + " vs. " + nonFluents._sDomain);
            System.exit(1);
        }
        state.init(domain._hmObjects, nonFluents != null ? nonFluents._hmObjects : null, instance._hmObjects,
                domain._hmTypes, domain._hmPVariables, domain._hmCPF,
                instance._alInitState, nonFluents == null ? new ArrayList<RDDL.PVAR_INST_DEF>() : nonFluents._alNonFluents, instance._alNonFluents,
                domain._alStateConstraints, domain._alActionPreconditions, domain._alStateInvariants,
                domain._exprReward, instance._nNonDefActions);


        //This get state variable names
        ArrayList<CString> actions = new ArrayList<>();
        actions.add(new CString("noop"));
        ArrayList<String> stateVaribleNames = new ArrayList<>();
        for(PVAR_NAME key : domain._hmPVariables.keySet()){
            PVARIABLE_DEF var_def = domain._hmPVariables.get(key);
            if( ( var_def instanceof PVARIABLE_STATE_DEF) ){
                if( ! ((PVARIABLE_STATE_DEF) var_def)._bNonFluent){
                    //stateVaribleNames.add(key._sPVarName);
                    ArrayList<ArrayList<LCONST>> gfluents = state.generateAtoms(key);
                    for (ArrayList<LCONST> gfluent : gfluents) {
                        String variableName = RDDL2Format.CleanFluentName(key._sPVarName + gfluent);
                        stateVaribleNames.add(variableName);
                    }
                }

            }else if(var_def instanceof PVARIABLE_ACTION_DEF){
                ArrayList<ArrayList<LCONST>> gfluents = state.generateAtoms(key);
                for (ArrayList<LCONST> gfluent : gfluents) {
                    String variableName = RDDL2Format.CleanFluentName(key._sPVarName + gfluent);
                    actions.add(new CString(variableName) );
                }
            }

        }

        RandomDataGenerator rand = new RandomDataGenerator();

        RandomRolloutPolicy random_roll_policy = new RandomRolloutPolicy(stateVaribleNames,instance._nHorizon,
                instance._dDiscount,actions,instance,null,rand, rddl,instance_name,null);
        UCT2 uct_planner  = new UCT2(stateVaribleNames,instance._nHorizon,
                                    instance._dDiscount,actions,instance,null,rand, rddl,instance_name,random_roll_policy);



       String action_val = uct_planner.getBestAction(state);

        System.out.println("Initializing is Done....");





    }

    public static void main(String[] args) throws Exception{
        initialzeRDDL(args[0],args[1]);




    }



}
