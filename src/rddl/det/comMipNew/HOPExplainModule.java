package rddl.det.comMipNew;


import com.sun.xml.internal.ws.api.pipe.helper.PipeAdapter;
import gurobi.*;
import rddl.EvalException;
import rddl.RDDL;
import rddl.State;
import util.Pair;

import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class HOPExplainModule extends HOPPlannerNew {

    protected String STATE_KEY = "_State";
    protected String ACTION_KEY = "_Action";
    protected String INTERM_KEY = "_Interm";
    protected String REWARD_KEY = "_Reward";
    protected String PLAN_PATH  = "./Explanation_Output/HOP_Plans/";
    protected String VISUAL_CROSSING_DOMAIN_FOLDER = "./Explanation_Output/HOP_Visual_Plans/";


    public  HOPExplainModule(String rddl_filename, String n_futures, String n_lookahead, String inst_name,
                            String gurobi_timeout, String future_gen_type, String hindsight_strat, String rand_seed,String plan_file_name) throws Exception{

        super(rddl_filename,n_futures,n_lookahead,inst_name,gurobi_timeout,future_gen_type,hindsight_strat,rand_seed);
        System.out.println("dkfjkdjfkdjkfjd");
        this.PLAN_PATH = this.PLAN_PATH + plan_file_name;

        //Need to think about the parameter for explaination.


    }



    public void runOneStepExplaination(State s) throws Exception{
            if(DO_NPWL_PWL){
                checkNonLinearExpressions(state);
            }



            for(int i=0 ; i<10 ; i++){
                ArrayList<RDDL.PVAR_INST_DEF> actions = null;
                try {
                    actions = getHOPPlans(s);
                    System.out.println("The Action Taken is >>" + actions.toString());
                    s.checkStateActionConstraints(actions);
                }catch(Exception e1){
                    // This is the case when actions are infeasible or gurobi has infeasiblity.
                    actions = baseLineAction(s);
                }
                //Computing the next state!!>
                s.computeNextState(actions,_random);
                try{
                    s.checkStateActionConstraints(actions);
                } catch (Exception e){

                    System.out.println("State or Action Contraint Voilated ");
                    throw e;
                }
                s.advanceNextState();
                final double immediate_reward = ((Number)domain._exprReward.sample(new HashMap<RDDL.LVAR,RDDL.LCONST>(),state,_random)).doubleValue();


            }


            //Need to get information on each state in future and their actions.

    }



    public ArrayList<RDDL.PVAR_INST_DEF> getHOPPlans(State s) throws Exception{
        //This Function should output the plans for state "S";
        HashMap<RDDL.PVAR_NAME, HashMap<ArrayList<RDDL.LCONST>, Object>> subs = getSubsWithDefaults( s );

        if( static_grb_model == null ){
            System.out.println("----Initializing Gurobi model----");
            firstTimeModel(subs);
        }
        try{
            translateCPTs( subs, static_grb_model , false );
            translateReward( static_grb_model , false);
            translateInitialState( static_grb_model, subs );

            int exit_code = -1;
            try{
                System.out.println("----STARTING OPTIMIZATION----");
                grb_env.set( GRB.DoubleParam.TimeLimit, TIME_LIMIT_MINS*60 );
                exit_code = goOptimize( static_grb_model);
            }
            catch( GRBException exc ){
                int error_code = exc.getErrorCode();
            }
            finally{
                System.out.println("Exit code " + exit_code );
            }
            Map<RDDL.EXPR, Double > ret  = outputResults( static_grb_model);
            modelSummary( static_grb_model );
            ArrayList<RDDL.PVAR_INST_DEF> ret1 = getRootActions(ret,s);
            if(exit_code ==3 ){
                try {
                    throw new Exception("Model Infeasiblity Excepiton");
                }catch(Exception e){
                    cleanUp(static_grb_model);
                    throw e;
                }
            }
            /////////////////////////////////
            Pair<ArrayList<HashMap<String,HashMap<RDDL.LCONST,Object>>>,ArrayList<ArrayList<String>>> plans_info = decomGRBPlans(static_grb_model);
            ArrayList<HashMap<String,HashMap<RDDL.LCONST,Object>>> all_future_plans = plans_info._o1;
            ArrayList<ArrayList<String>> all_futures_str = plans_info._o2;
            vizPlansToFolder(all_future_plans,"VIZ_CONSENSUS_PLANS_F_5_L_5_N_");
            printPlanToFile(all_futures_str,"CONSENSUS_PLANS_F_5_L_5.csv");
            cleanUp(static_grb_model);
            return ret1;

        } catch (Exception e) {
            e.printStackTrace();
            throw e ;
        }



    }


    public void  vizPlansToFolder(ArrayList<HashMap<String,HashMap<RDDL.LCONST,Object>>> plans,String file_prefix) throws Exception{
        CrossingWorldUI print_obj =new CrossingWorldUI();
        for(int i=0 ; i<plans.size();i++){
            String vizPlan = print_obj.runPlan(plans.get(i),this.lookahead);

            try (Writer writer = new BufferedWriter(new OutputStreamWriter(
                    new FileOutputStream(VISUAL_CROSSING_DOMAIN_FOLDER+file_prefix+"_"+String.valueOf(i)+".txt"), "utf-8"))) {
                    writer.write(vizPlan);
            }catch (Exception e ){
                System.out.println("File Not Written : " + VISUAL_CROSSING_DOMAIN_FOLDER+file_prefix+"_"+String.valueOf(i)+".txt");
                throw e;
            }

        }

    }




    public Pair<ArrayList<HashMap<String,HashMap<RDDL.LCONST,Object>>>,ArrayList<ArrayList<String>>> decomGRBPlans(final GRBModel static_grb_model) throws Exception{
        HashMap<Pair<RDDL.LCONST,RDDL.LCONST>,Map<RDDL.EXPR,Double>> ret1 = extractALLInfoGurobi(static_grb_model);
        HashMap<Pair<RDDL.LCONST,RDDL.LCONST>,Double> reward_ret =extractRewardInfo(static_grb_model);
        ArrayList<ArrayList<String>> all_futures = new ArrayList<>();
        ArrayList<HashMap<String,HashMap<RDDL.LCONST,Object>>> plans_list = new ArrayList<>();
        for(int i=0 ; i<future_TERMS.size();i++){
            HashMap<String,HashMap<RDDL.LCONST,Object>> plan =getAFuturePlan(ret1, reward_ret,i);
            ArrayList<String> output =printPlanToArrayList(plan);
            all_futures.add(output);
            plans_list.add(plan);
        }

        return new Pair(plans_list,all_futures);
    }




    public void printPlanToFile(ArrayList<ArrayList<String>> plans, String file_name) throws Exception{
        ArrayList<String> concat_list = new ArrayList<>();
        for(int i=0 ; i<plans.size();i++){
            for(int j=0 ;j<plans.get(i).size();j++){
                try{
                    String val = concat_list.get(j);
                    concat_list.set(j,val+","+plans.get(i).get(j));
                }catch (IndexOutOfBoundsException e){
                    concat_list.add(j,plans.get(i).get(j));
                }
            }
        }

        try (Writer writer = new BufferedWriter(new OutputStreamWriter(
                new FileOutputStream(PLAN_PATH+file_name), "utf-8"))) {
            for(int i=0;i<concat_list.size();i++){
                writer.write(concat_list.get(i)+"\n");
            }
        }
    }


    public HashMap<String,HashMap<RDDL.LCONST,Object>> getAFuturePlan(HashMap<Pair<RDDL.LCONST,RDDL.LCONST>,
            Map<RDDL.EXPR,Double>> gurobi_output,HashMap<Pair<RDDL.LCONST,RDDL.LCONST>,Double>reward_ret,int future_index)  throws Exception{
        /*
        * This will return a HashMap with _state_value , _action_value, _reward_value
        *
        * */
        HashMap<String,HashMap<RDDL.LCONST,Object>> time_mapping = new HashMap<>();
        time_mapping.put(STATE_KEY,new HashMap<>());
        time_mapping.put(ACTION_KEY,new HashMap<>());
        time_mapping.put(REWARD_KEY,new HashMap<>());
        time_mapping.put(INTERM_KEY,new HashMap<>());
        //This if for State,Action and Interm values
        for(int i=0 ; i<TIME_TERMS.size();i++){
             RDDL.LCONST future_term = future_TERMS.get(future_index);
             RDDL.LCONST time_term = TIME_TERMS.get(i);
             Pair<RDDL.LCONST,RDDL.LCONST> key =  new Pair<>(future_term,time_term);
             gurobi_output.get(key);
             for(RDDL.EXPR e : gurobi_output.get(key).keySet()){
                 Double val = gurobi_output.get(key).get(e);

                 if( (e instanceof RDDL.PVAR_EXPR) && (val!=0.0)){
                     RDDL.PVAR_NAME temp_pvar =  ((RDDL.PVAR_EXPR) e)._pName;
                     ArrayList<RDDL.LTERM> lconst_list = ((RDDL.PVAR_EXPR) e)._alTerms;
                     if(hm_variables.get(temp_pvar) instanceof RDDL.PVARIABLE_STATE_DEF){
                         if(!time_mapping.get("_State").containsKey(time_term)){
                             HashMap<RDDL.EXPR,Double>temp_hash = new HashMap<>();
                             temp_hash.put(e,val);
                             time_mapping.get("_State").put(time_term,temp_hash);
                         }else{
                             ((HashMap<RDDL.EXPR,Double>) time_mapping.get("_State").get(time_term)).put(e,val);
                         }
                     }else if(hm_variables.get(temp_pvar) instanceof RDDL.PVARIABLE_ACTION_DEF){
                         if(!time_mapping.get("_Action").containsKey(time_term)){
                             HashMap<RDDL.EXPR,Double>temp_hash = new HashMap<>();
                             temp_hash.put(e,val);
                             time_mapping.get("_Action").put(time_term,temp_hash);
                         }else{
                             ((HashMap<RDDL.EXPR,Double>) time_mapping.get("_Action").get(time_term)).put(e,val);
                         }
                     }else if(hm_variables.get(temp_pvar) instanceof RDDL.PVARIABLE_INTERM_DEF){
                         if(!time_mapping.get("_Interm").containsKey(time_term)){
                             HashMap<RDDL.EXPR,Double>temp_hash = new HashMap<>();
                             temp_hash.put(e,val);
                             time_mapping.get("_Interm").put(time_term,temp_hash);
                         }else{
                             ((HashMap<RDDL.EXPR,Double>) time_mapping.get("_Interm").get(time_term)).put(e,val);
                         }

                     }else{
                      throw new Exception("This case is not handled in this block");
                     }


                 }
             }

             Pair key_val = new Pair<>(future_term,time_term);
             Double reward_val = reward_ret.get(key_val);
             time_mapping.get("_Reward").put(time_term,reward_val);
        }

        //Need to get the immediate Reward





        return time_mapping;
    }




    public void getExoAndEndo(State s) throws Exception {
        /*
            This function classifies the state variables as Endogenous State variables and Exogeneous State Variables
        */

        HashMap<RDDL.PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = s._hmPVariables;


        for(RDDL.PVAR_NAME key : hm_variables.keySet()){

            RDDL.PVARIABLE_DEF pvar_def = hm_variables.get(key);

            //This is to get the pvariable names
            if( pvar_def instanceof RDDL.PVARIABLE_STATE_DEF  ){
                if(!((RDDL.PVARIABLE_STATE_DEF) pvar_def)._bNonFluent ){



                }

            }






        }













    }





    protected HashMap<Pair<RDDL.LCONST,RDDL.LCONST>,Map<RDDL.EXPR,Double>> extractALLInfoGurobi(final GRBModel grb_model ) throws Exception {

        System.out.println("------This is output results for GRB MODEL -------");
//		DecimalFormat df = new DecimalFormat("#.##########");
//		df.setRoundingMode( RoundingMode.DOWN );

        HashMap<Pair<RDDL.LCONST,RDDL.LCONST>,Map<RDDL.EXPR,Double>> organize_data = new HashMap<>();
        if( grb_model.get( GRB.IntAttr.SolCount ) == 0 ){
            return null;
        }

        Map<RDDL.EXPR, Double > ret = new HashMap<RDDL.EXPR, Double >();

        HashMap<RDDL.PVAR_NAME, ArrayList<ArrayList<RDDL.LCONST>>> src = new HashMap<>();
        src.putAll( rddl_action_vars );
        src.putAll( rddl_interm_vars );
        src.putAll( rddl_state_vars );

        src.forEach( new BiConsumer<RDDL.PVAR_NAME, ArrayList<ArrayList<RDDL.LCONST> > >( ) {

            @Override
            public void accept(RDDL.PVAR_NAME pvar,
                               ArrayList<ArrayList<RDDL.LCONST>> u) {
                u.forEach( new Consumer< ArrayList<RDDL.LCONST> >( ) {
                    @Override
                    public void accept(ArrayList<RDDL.LCONST> terms ) {
                        future_TERMS.forEach( new Consumer<RDDL.LCONST>() {
                            @Override
                            public void accept(RDDL.LCONST future_term) {

                                TIME_TERMS.forEach(new Consumer<RDDL.LCONST>(){
                                    @Override
                                    public void accept(RDDL.LCONST time_term){
                                        try {
                                            RDDL.EXPR action_var = new RDDL.PVAR_EXPR( pvar._sPVarName, terms )
                                                    .addTerm(TIME_PREDICATE, constants, objects, hmtypes, hm_variables )
                                                    .addTerm(future_PREDICATE, constants, objects, hmtypes, hm_variables )
                                                    .substitute( Collections.singletonMap( TIME_PREDICATE, time_term ), constants, objects, hmtypes, hm_variables )
                                                    .substitute( Collections.singletonMap( future_PREDICATE, future_term ) , constants, objects, hmtypes, hm_variables );

                                            //Please ignore the commented code, this was for testing
//                                    for(EXPR key : EXPR.grb_cache.keySet()){
//                                        if(key.toString().equals("(roll($d1, $time0, $future0) ^ roll($d2, $time0, $future0) ^ roll($d3, $time0, $future0) ^ roll($d4, $time0, $future0) ^ roll($d5, $time0, $future0))")){
//                                            System.out.println("dfdkjfkdj");
//                                        }
//                                        if(key.toString().equals("((current-phase($time0, $future0) == @roll1) => (roll($d1, $time0, $future0) ^ roll($d2, $time0, $future0) ^ roll($d3, $time0, $future0) ^ roll($d4, $time0, $future0) ^ roll($d5, $time0, $future0)))")){
//                                            System.out.println("dkjfdkjfd");
//
//                                        }
//                                        if(key.toString().equals("(current-phase($time0, $future0) == @roll1)")){
//                                            System.out.println("djfkdjfkd");
//                                        }
//
//                                        if(key.toString().equals("~(current-phase($time0, $future0) == @roll1)")){
//                                            System.out.println("dkjfkdjf");
//                                        }
//
//                                        if(key.toString().equals("current-phase($time0, $future0)")){
//                                            System.out.println("kdjfkdjkf");
//                                        }
//
//                                        if(key.toString().equals("@roll1")){
//                                            System.out.println("dkfjkdkfdj");
//                                        }
//
//                                        if(key.toString().equals("(die-value($d1, $time0, $future0) == @1)")){
//                                            System.out.println("kdjkjdkfjkdjf");
//                                        }
//
//                                        if(key.toString().equals("die-value($d1, $time0, $future0)")){
//                                            System.out.println("dkfkdjkfjdk");
//                                        }
//
//                                        if(key.toString().equals("(agent-at($x3, $y3, $time0, $future0) == 0)")){
//                                            System.out.println("djkfjdfkd");
//                                        }
//
//                                    }


                                            //System.out.println(action_var);
                                            GRBVar grb_var = RDDL.EXPR.grb_cache.get( action_var );
                                            assert( grb_var != null );
                                            double actual = grb_var.get( GRB.DoubleAttr.X );

                                            //NOTE : uncomment this part if having issues with constrained actions
                                            // such as if you get -1E-11 instead of 0,
                                            //and you are expecting a positive action >= 0
                                            String interm_val = State._df.format( actual );
                                            //System.out.println( actual + " rounded to " + interm_val );
                                            Pair<RDDL.LCONST,RDDL.LCONST> key_val = new Pair<>(future_term,time_term);
                                            HashMap<RDDL.EXPR,Double> opti_values = new HashMap<>();
                                            opti_values.put(action_var,Double.valueOf(interm_val));
                                            if(!organize_data.containsKey(key_val)){
                                                organize_data.put(key_val,opti_values);
                                            }else{
                                                (organize_data.get(key_val)).put(action_var,Double.valueOf(  interm_val ));
                                            }


                                            ret.put( action_var, Double.valueOf(  interm_val ) );
                                        } catch (GRBException e) {
                                            e.printStackTrace();
                                            ////System.exit(1);
                                        }
                                        catch (Exception e){
                                            e.printStackTrace();
                                        }
                                    }
                                });
                            }
                        });
                    }
                });
            }

        });

        // This is for getting reward value from Gurobi






        System.out.println( "Maximum (unscaled) bound violation : " +  + grb_model.get( GRB.DoubleAttr.BoundVio	) );
        System.out.println("Sum of (unscaled) constraint violations : " + grb_model.get( GRB.DoubleAttr.ConstrVioSum ) );
        System.out.println("Maximum integrality violation : "+ grb_model.get( GRB.DoubleAttr.IntVio ) );
        System.out.println("Sum of integrality violations : " + grb_model.get( GRB.DoubleAttr.IntVioSum ) );
        System.out.println("Objective value : " + grb_model.get( GRB.DoubleAttr.ObjVal ) );

        return organize_data;
    }


    public HashMap<Pair<RDDL.LCONST,RDDL.LCONST>,Double> extractRewardInfo(GRBModel grbModel) throws Exception{

        HashMap<Pair<RDDL.LCONST,RDDL.LCONST>,Double> reward_mapping = new HashMap<>();
        RDDL.EXPR stationary_clear = rddl_state._reward;
        if( replace_reward_pwl != null ){
            System.out.println("--Replacing NPWL Reward to PWL Reward");
            stationary_clear = replace_reward_pwl;
        }else if( !stationary_clear._bDet ){
            stationary_clear = future_gen.getFuture(stationary_clear,
                    this._random, constants, objects, hmtypes, hm_variables);
        }


        Pair<String,String> key = new Pair<String, String>(stationary_clear.toString(),
                Collections.EMPTY_MAP.toString());
        if( substitute_expression_cache.containsKey( key ) ){
            stationary_clear = substitute_expression_cache.get(key);
        }else{
            stationary_clear = stationary_clear.substitute( Collections.EMPTY_MAP,
                    constants, objects, hmtypes, hm_variables );
            substitute_expression_cache.put(key, stationary_clear);
        }

        final RDDL.EXPR non_stationary = stationary_clear.addTerm( TIME_PREDICATE ,
                constants, objects, hmtypes, hm_variables )
                .addTerm( future_PREDICATE, constants, objects, hmtypes, hm_variables );

        GRBLinExpr all_sum = new GRBLinExpr();
        //System.out.println(non_stationary);

        ArrayList<Integer> time_terms_indices = new ArrayList<Integer>( TIME_TERMS.size() );
        for( int i = 0 ; i < TIME_TERMS.size(); ++i ){
            time_terms_indices.add( i );
        }

        future_TERMS.stream().forEach( new Consumer<RDDL.LCONST>() {
            @Override
            public void accept(final RDDL.LCONST future_term) {

                time_terms_indices.stream().forEach( new Consumer<Integer>() {
                    @Override
                    public void accept(final Integer index){
                        try {
                            final RDDL.LCONST time_term = TIME_TERMS.get(index);
                            //add discounting
                            final double cur_disc = Math.pow(rddl_instance._dDiscount, index);

                            final RDDL.EXPR subs_tf = non_stationary.substitute(Collections.singletonMap(TIME_PREDICATE, time_term),
                                    constants, objects, hmtypes, hm_variables )
                                    .substitute(Collections.singletonMap(future_PREDICATE, future_term), constants, objects, hmtypes, hm_variables );
                            //System.out.println( subs_tf );//"Reward_" + time_term + "_" + future_term );
                            GRBVar grb_var = RDDL.EXPR.grb_cache.get( subs_tf );
                            assert( grb_var != null );
                            //Not using grb_cache, beacuse it is throwing as null even though the expression is present.  May be due to memory flushing.
                            double actual = grbModel.getVarByName(RDDL.EXPR.name_map.get(subs_tf.toString())).get(GRB.DoubleAttr.X);
                            Pair key_val = new Pair<>(future_term,time_term);
                            reward_mapping.put(key_val,Double.valueOf(actual));


                        }
                        catch (Exception e){
                            e.printStackTrace();
                        }
                    }
                });
            }
        });



    return reward_mapping;
    }


    public ArrayList<String> printPlanToArrayList(HashMap<String,HashMap<RDDL.LCONST,Object>> plan) throws Exception{
        ArrayList<String> output = new ArrayList<>();

        for(int i=0;i<TIME_TERMS.size();i++){
            String str_state = "";
            String str_action = "NOOP";
            String str_reward = "";
            str_state = plan.get(STATE_KEY).get(TIME_TERMS.get(i)).toString().replace(",",":");
            str_reward = plan.get(REWARD_KEY).get(TIME_TERMS.get(i)).toString().replace(",",":");

            try{
                str_action = plan.get(ACTION_KEY).get(TIME_TERMS.get(i)).toString().replace(",",":");

            }catch (Exception e){
                System.out.println("No Action Variable");
            }

            output.add(str_state);
            output.add(str_action);
            output.add(str_reward);

        }
        return output;

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
        String plan_filename  = args[9];

        HOPExplainModule hop_exp = new HOPExplainModule(rddl_file_path,n_Futures,n_lookahead,rddl_instance_name,gurobi_timeout,future_gen_type,hindsight_strat,rand_seed,plan_filename);
        hop_exp.warpperRunExplaination();

    }









}
