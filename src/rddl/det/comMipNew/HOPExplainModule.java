package rddl.det.comMipNew;


import rddl.RDDL;
import rddl.State;

import java.util.ArrayList;
import java.util.HashMap;

public class HOPExplainModule extends HOPPlannerNew {


    public  HOPExplainModule(String rddl_filename, String n_futures, String n_lookahead, String inst_name,
                            String gurobi_timeout, String future_gen_type, String hindsight_strat, String rand_seed) throws Exception{

        super(rddl_filename,n_futures,n_lookahead,inst_name,gurobi_timeout,future_gen_type,hindsight_strat,rand_seed);
        System.out.println("dkfjkdjfkdjkfjd");

        //Need to think about the parameter for explaination.


    }




    public void runOneStepExplaination(State s) throws Exception{
            if(DO_NPWL_PWL){
                checkNonLinearExpressions(state);
            }
            ArrayList<RDDL.PVAR_INST_DEF> actions = null;
            try {
                actions = getActions(s);
                System.out.println("The Action Taken is >>" + actions.toString());
                s.checkStateActionConstraints(actions);
            }catch(Exception e1){
                // This is the case when actions are infeasible or gurobi has infeasiblity.
                actions = baseLineAction(s);
            }
            //Computing the next state!!>
            state.computeNextState(actions,_random);
            try{
                state.checkStateActionConstraints(actions);
            } catch (Exception e){

                System.out.println("State or Action Contraint Voilated ");
                throw e;
            }


            //Need to get information on each state in future and their actions.






            final double immediate_reward = ((Number)domain._exprReward.sample(new HashMap<RDDL.LVAR,RDDL.LCONST>(),state,_random)).doubleValue();






    }




    public void warpperRunExplaination() throws Exception{


        int horizon  = instance._nHorizon;
        double cur_discount = instance._dDiscount;
        state.init(domain._hmObjects, nonFluents != null ? nonFluents._hmObjects : null, instance._hmObjects,
                domain._hmTypes, domain._hmPVariables, domain._hmCPF,
                instance._alInitState, nonFluents == null ? new ArrayList<RDDL.PVAR_INST_DEF>() : nonFluents._alNonFluents, instance._alNonFluents,
                domain._alStateConstraints, domain._alActionPreconditions, domain._alStateInvariants,
                domain._exprReward, instance._nNonDefActions);

        runOneStepExplaination(state);

    }





    public static void main(String[] args) throws Exception{

        //need to implement a calling function!!!..
        // 1. rddl_file_path
        // 2. rddl_instance_name
        // 3. n_Futures
        // 4. n_lookahead
        // 5. gurobi_timeout
        // 6. future_gen_type
        // 7. hindsight_strat
        // 8. num_of_rounds
        // 9. rand_seed

        String rddl_file_path = args[0];
        String rddl_instance_name = args[1];
        String n_Futures = args[2];
        String n_lookahead = args[3];
        String gurobi_timeout = args[4];
        String future_gen_type = args[5];
        String hindsight_strat = args[6];
        String num_of_rounds   = args[7];
        String rand_seed = args[8];

        HOPExplainModule hop_exp = new HOPExplainModule(rddl_file_path,n_Futures,n_lookahead,rddl_instance_name,gurobi_timeout,future_gen_type,hindsight_strat,rand_seed);
        hop_exp.warpperRunExplaination();

    }









}
