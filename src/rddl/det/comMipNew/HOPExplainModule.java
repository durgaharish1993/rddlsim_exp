package rddl.det.comMipNew;


import com.sun.xml.internal.ws.api.pipe.helper.PipeAdapter;
import gurobi.*;
import rddl.EvalException;
import rddl.RDDL;
import rddl.State;
import rddl.policy.RandomPolicy;
import util.Pair;

import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.util.*;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import static rddl.RDDL.EXPR.name_map;

public class HOPExplainModule extends HOPPlannerNew {

    protected String STATE_KEY = "_State";
    protected String ACTION_KEY = "_Action";
    protected String INTERM_KEY = "_Interm";
    protected String REWARD_KEY = "_Reward";
    protected String PLAN_PATH  = "./Explanation_Output/HOP_Plans/";
    protected String VISUAL_CROSSING_DOMAIN_FOLDER = "./Explanation_Output/HOP_Visual_Plans/";
    protected String PREFIX_VIZ_FILE = null;
    protected boolean DO_SOMETHING_ELSE_GUROBI = true;
    protected Writer writer_obj = null;

    public  HOPExplainModule(String rddl_filename, String n_futures, String n_lookahead, String inst_name,
                            String gurobi_timeout, String future_gen_type, String hindsight_strat, String rand_seed,String plan_file_name, String prefix_path, String explanation_objectives_path) throws Exception{

        super(rddl_filename,n_futures,n_lookahead,inst_name,gurobi_timeout,future_gen_type,hindsight_strat,rand_seed);
        System.out.println("dkfjkdjfkdjkfjd");
        this.PLAN_PATH = this.PLAN_PATH + plan_file_name;
        this.PREFIX_VIZ_FILE = prefix_path;

        this.writer_obj = new BufferedWriter(new OutputStreamWriter(
                new FileOutputStream(explanation_objectives_path), "utf-8"));


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
    }



    public ArrayList<RDDL.PVAR_INST_DEF> getHOPPlans(State s) throws Exception{
        //This Function should output the plans for state "S";
        HashMap<RDDL.PVAR_NAME, HashMap<ArrayList<RDDL.LCONST>, Object>> subs = getSubsWithDefaults( s );

        if( static_grb_model == null ){
            System.out.println("----Initializing Gurobi model----");
            firstTimeModel(subs);
        }
        try {
            translateCPTs(subs, static_grb_model, false);
            translateReward(static_grb_model, false);
            translateInitialState(static_grb_model, subs);
        } catch (Exception e ){
            System.out.println("--- There is problem with the translating the domain to gurobi");
            throw e ;

        }

        try{
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
            vizPlansToFolder(all_future_plans,PREFIX_VIZ_FILE);
            printPlanToFile(all_futures_str,null);
            double obj_val = static_grb_model.get(GRB.DoubleAttr.ObjVal);


            ArrayList<RDDL.PVAR_INST_DEF> ret2 = null;
            if(DO_SOMETHING_ELSE_GUROBI){
                HashSet<RDDL.PVAR_INST_DEF> action_not_same = getActionNotEqualGurobi(s,0);
                for(int k =0 ; k< future_TERMS.size() ; k++){
                    static_grb_model = addBoolDiffAction(static_grb_model,action_not_same,k);
                }
                static_grb_model.reset();
                try{
                    System.out.println("----STARTING OPTIMIZATION----");
                    grb_env.set( GRB.DoubleParam.TimeLimit, TIME_LIMIT_MINS*60 );
                    exit_code = goOptimize( static_grb_model);
                }
                catch( GRBException exc ){
                    int error_code = exc.getErrorCode();
                }

                double obj_val1 = static_grb_model.get(GRB.DoubleAttr.ObjVal);

                ret  = outputResults( static_grb_model);
                modelSummary( static_grb_model );
                ret2 = getRootActions(ret,s);
                Pair<ArrayList<HashMap<String,HashMap<RDDL.LCONST,Object>>>,ArrayList<ArrayList<String>>> plans_info1 = decomGRBPlans(static_grb_model);
                ArrayList<HashMap<String,HashMap<RDDL.LCONST,Object>>> all_future_plans1 = plans_info1._o1;
                ArrayList<ArrayList<String>> all_futures_str1 = plans_info1._o2;
                vizPlansToFolder(all_future_plans1,"Second_plan_F_1_L_5_");
                printPlanToFile(all_futures_str1,"./Explanation_Output/HOP_Plans/second_plan_F_1_L_5.csv");


                String cur_state_val = s._state.toString().replace(",",":");
                String optimal_action = ret1.toString().replace(",",":");
                String not_optimal_action = ret2.toString().replace(",",":");
                String obj_string_1 = String.valueOf(obj_val);
                String obj_string_2 = String.valueOf(obj_val1);
                writer_obj.write(cur_state_val+","+optimal_action+"," + obj_string_1 + "," + not_optimal_action + "," + obj_string_2 +"\n");
                writer_obj.flush();

            }


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
            String vizPlan = "";
            try{
                vizPlan = print_obj.runPlan(plans.get(i),this.lookahead);
            } catch (Exception e){
                System.out.println("--The Plan Could not produce Visualization --");

            }

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


    public void printPlanToFile(ArrayList<ArrayList<String>> plans, String file_path) throws Exception{
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

        if(file_path==null){
            try (Writer writer = new BufferedWriter(new OutputStreamWriter(
                    new FileOutputStream(PLAN_PATH), "utf-8"))) {
                for(int i=0;i<concat_list.size();i++){
                    writer.write(concat_list.get(i)+"\n");
                }
            }

        }else{
            try (Writer writer = new BufferedWriter(new OutputStreamWriter(
                    new FileOutputStream(file_path), "utf-8"))) {
                for(int i=0;i<concat_list.size();i++){
                    writer.write(concat_list.get(i)+"\n");
                }
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


    public HashSet<RDDL.PVAR_INST_DEF> getActionNotEqualGurobi(State s,int future_val) throws Exception{
        RandomPolicy random_policy = new RandomPolicy(s);
        ArrayList<ArrayList<RDDL.PVAR_INST_DEF>> legalActions = new ArrayList<>();
        HashMap<RDDL.PVAR_NAME, ArrayList<ArrayList<RDDL.LCONST>>> src = new HashMap<>();
        src.putAll(rddl_action_vars);
        Map<RDDL.EXPR, Double > ret = new HashMap<RDDL.EXPR, Double >();
        RDDL.LCONST time_term  = new RDDL.OBJECT_VAL("time0");
        RDDL.LCONST future_term = new RDDL.OBJECT_VAL("future"+String.valueOf(future_val));
        for(RDDL.PVAR_NAME key : src.keySet()){
            ArrayList<ArrayList<RDDL.LCONST>> vals = src.get(key);
            for(int i=0 ; i<vals.size();i++){
                ArrayList<RDDL.LCONST> terms = vals.get(i);
                RDDL.EXPR action_var = new RDDL.PVAR_EXPR(key._sPVarName,terms)
                        .addTerm(TIME_PREDICATE, constants, objects, hmtypes, hm_variables )
                        .addTerm(future_PREDICATE, constants, objects, hmtypes, hm_variables )
                        .substitute( Collections.singletonMap( TIME_PREDICATE, time_term ), constants, objects, hmtypes, hm_variables )
                        .substitute( Collections.singletonMap( future_PREDICATE, future_term ) , constants, objects, hmtypes, hm_variables );
                try{
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
                    ret.put( action_var, Double.valueOf(  interm_val ) );
                } catch (GRBException e) {
                    e.printStackTrace();
                    ////System.exit(1);
                }
            }
        }
        ArrayList<RDDL.PVAR_INST_DEF> gurobi_action = new ArrayList<>();
        for(RDDL.EXPR key : ret.keySet()){
            Double val = ret.get(key);
            if(key instanceof RDDL.PVAR_EXPR){
                RDDL.PVAR_NAME p_name = ((RDDL.PVAR_EXPR) key)._pName;
                ArrayList<RDDL.LTERM> terms = new ArrayList<RDDL.LTERM>(((RDDL.PVAR_EXPR) key)._alTerms.subList(0,2));
                gurobi_action.add(new RDDL.PVAR_INST_DEF(p_name._sPVarName, val >0.95 ? Boolean.TRUE : Boolean.FALSE,terms));

            }else{
                throw new Exception("There should be a PVAR_EXPR");
            }
        }

        HashSet gurobi_action_set = new HashSet(gurobi_action);
        ArrayList<RDDL.PVAR_INST_DEF> rand_actions = random_policy.getActions(s);
        HashSet<RDDL.PVAR_INST_DEF> random_action_set = new HashSet(rand_actions);
        int MAX_TRAILS = 20;
        int num_of_trails  = 0 ;
        while(random_action_set.equals(gurobi_action_set) && num_of_trails<MAX_TRAILS){
            random_action_set = new HashSet<RDDL.PVAR_INST_DEF>(random_policy.getActions(s));
            num_of_trails +=1;
        }

        return random_action_set;
    }


    public GRBModel addBoolDiffAction(GRBModel grbModel, HashSet<RDDL.PVAR_INST_DEF> random_action_set, int future_val) throws Exception{
        //This function adds a different action to the root node and try returns the newly added constraint.
        RDDL.OBJECT_VAL time_term = new RDDL.OBJECT_VAL("time0");
        RDDL.OBJECT_VAL future_term = new RDDL.OBJECT_VAL( "future" + String.valueOf(future_val));
        for(RDDL.PVAR_INST_DEF key1 : random_action_set){
            RDDL.EXPR act_val  = new RDDL.PVAR_EXPR(key1._sPredName._sPVarName,key1._alTerms).addTerm(TIME_PREDICATE, constants, objects, hmtypes, hm_variables )
                    .addTerm(future_PREDICATE, constants, objects, hmtypes, hm_variables )
                    .substitute( Collections.singletonMap( TIME_PREDICATE, time_term ), constants, objects, hmtypes, hm_variables )
                    .substitute( Collections.singletonMap( future_PREDICATE, future_term ) , constants, objects, hmtypes, hm_variables );
            GRBVar grb_var = RDDL.EXPR.grb_cache.get(act_val);

            if(key1._oValue instanceof Boolean){
                Boolean action_value = (Boolean)key1._oValue ? true : false;
                RDDL.BOOL_CONST_EXPR expr_val = new RDDL.BOOL_CONST_EXPR(action_value);
                GRBVar x2_var_rhs = expr_val.getGRBConstr(GRB.EQUAL,grbModel,constants,objects,type_map,hmtypes,hm_variables);
                GRBConstr a =grbModel.addConstr(grb_var,GRB.EQUAL,x2_var_rhs, name_map.get(grb_var.toString()) +"="+ name_map.get(expr_val.toString()));
                to_remove_constr.add(a);
            }else{
                throw new Exception("This is not boolean, Not implemented");
            }

        }

        return grbModel;


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
            //This handles when there is no State variables.
            try{
                str_state = plan.get(STATE_KEY).get(TIME_TERMS.get(i)).toString().replace(",",":");
            } catch (Exception e){
                System.out.println("--No State Variable--");
            }
            //This handles action when not present
            try{
                str_action = plan.get(ACTION_KEY).get(TIME_TERMS.get(i)).toString().replace(",",":");

            }catch (Exception e){
                System.out.println("--No Action Variable--");
            }
            str_reward = plan.get(REWARD_KEY).get(TIME_TERMS.get(i)).toString().replace(",",":");
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
        // 10.Plans writing csv File
        // 11.Prefix for writing Viz Representation.


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
        String prefix_path = args[10];
        String explain_obj = args[11];

        HOPExplainModule hop_exp = new HOPExplainModule(rddl_file_path,n_Futures,n_lookahead,rddl_instance_name,gurobi_timeout,future_gen_type,hindsight_strat,rand_seed,plan_filename,prefix_path,explain_obj);
        hop_exp.warpperRunExplaination();

    }









}
