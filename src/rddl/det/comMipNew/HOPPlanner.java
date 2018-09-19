package rddl.det.comMipNew;

import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Array;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.NumberFormat;
import java.util.*;
import java.util.Map.Entry;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;

//import org.apache.commons.math3.random.RandomDataGenerator;

import gurobi.*;
import gurobi.GRB.StringAttr;
import org.apache.commons.math3.random.RandomDataGenerator;
import org.rosuda.JRI.Rengine;
import rddl.EvalException;
import rddl.RDDL;
import rddl.RDDL.BOOL_CONST_EXPR;
import rddl.RDDL.CPF_DEF;
import rddl.RDDL.EXPR;
import rddl.RDDL.INT_CONST_EXPR;
import rddl.RDDL.LCONST;
import rddl.RDDL.LVAR;
import rddl.RDDL.PVAR_EXPR;
import rddl.RDDL.PVAR_INST_DEF;
import rddl.RDDL.PVAR_NAME;
import rddl.RDDL.REAL_CONST_EXPR;
import rddl.RDDL.ENUM_VAL;
import rddl.State;
import rddl.policy.RandomPolicy;
//import rddl.det.comMip.HOPTranslate;
import rddl.policy.Policy;
import util.Pair;

import rddl.det.comMipNew.EarthForPlanner.*;

public class HOPPlanner extends Policy {

    private static final boolean SHOW_TIMING = false;
    protected static final boolean OUTPUT_LP_FILE = false;
    public static final String  OUTPUT_FILE = "model_toy.lp";
    public static final String OUTPUT_NAMEMAP_FILE = "model_toy_namemap.txt";
    public Boolean OUTPUT_NAME_MAP_FILE = true;

    protected boolean SHOW_LEVEL_1 = false;
    protected boolean SHOW_LEVEL_2 = false;
    protected boolean SHOW_GUROBI_ADD = false;
    protected boolean SHOW_PWL_NON_PWL = false;
    protected boolean SHOW_TIME_ZERO_GUROBI_ACTION = false;
    //protected boolean DO_GUROBI_INITIALIZATION = true;
    //DO_GUROBI_INITIALIZATION
    ///////////////////////////////////////////////////////////

    //this is to reduce the overhead of substituting expressions
    protected final Map<Pair<String,String>,EXPR>
            substitute_expression_cache = new HashMap<>();
    protected final boolean SUBSTITUTE_CACHE_USE = true;

    private static final int GRB_INFUNBDINFO = 1;
    private static final int GRB_DUALREDUCTIONS = 0;
    private static final double GRB_MIPGAP = 0.01;//0.5; //0.01;
    private static final double GRB_HEURISTIC = 0.2;
    private static final int GRB_IISMethod = 1;
    protected static final LVAR TIME_PREDICATE = new LVAR( "?time" );
    protected static final LVAR future_PREDICATE = new LVAR( "?future" );
    private static final RDDL.TYPE_NAME TIME_TYPE = new RDDL.TYPE_NAME( "time" );
    private static final boolean GRB_LOGGING_ON = false;
    protected static final boolean RECOVER_INFEASIBLE = true;
    protected RDDL rddl_obj;
    //protected int lookahead;
    protected State rddl_state;
    protected RDDL.DOMAIN rddl_domain;
    protected RDDL.INSTANCE rddl_instance;
    protected RDDL.NONFLUENTS rddl_nonfluents;
    protected String instance_name;
    protected String domain_name;
    private String GRB_log;
    protected HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>> rddl_state_vars;
    protected HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>> rddl_action_vars;
    protected HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>> rddl_observ_vars;
    protected HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>> rddl_interm_vars;
    protected List<String> string_state_vars;
    protected List<String> string_action_vars;
    protected List<String> string_observ_vars;
    protected List<String> string_interm_vars;
    protected GRBEnv grb_env;
    protected GRBModel static_grb_model = null;
    private HashMap<PVAR_NAME, RDDL.TYPE_NAME> pred_type = new HashMap<>();
    protected ArrayList<ArrayList<PVAR_INST_DEF>> ret_list = new ArrayList<ArrayList<PVAR_INST_DEF>>();

    protected HashMap<RDDL.TYPE_NAME, RDDL.OBJECTS_DEF> objects;
    protected Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
    protected HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes ;
    protected HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables;
    protected ArrayList< LCONST > TIME_TERMS = new ArrayList<>();
    protected HashMap<PVAR_NAME,  Character> type_map = new HashMap<>();
    //these are saved between invocations of getActions()
    //these are never removed
    protected List<EXPR> saved_expr = new ArrayList<>();
    protected List<GRBConstr> saved_constr = new ArrayList<>();


    //This is the range of actions to be taken, This will get initalized when the domain gets initalized and will get updated for picking random Actions.
    protected HashMap<PVAR_NAME,ArrayList<RDDL.TYPE_NAME>> object_type_name  = new HashMap<>();

    //This stores the Expression which are not PWL.
    protected HashMap<PVAR_NAME,Object> rddl_state_default = new HashMap<>();


    //NPWL
    protected HashMap<PVAR_NAME,ArrayList<EXPR>> replace_cpf_pwl = new HashMap<>();
    protected EXPR replace_reward_pwl = null;

    protected double running_R_api = 0.0;
    //these are removed between invocations of getActions()
    protected List<EXPR> to_remove_expr = new ArrayList<RDDL.EXPR>();
    protected List<GRBConstr> to_remove_constr = new ArrayList<>();
    protected EarthForPlanner earth_obj =new EarthForPlanner();



    public static enum FUTURE_SAMPLING{
        SAMPLE {
            @Override
            public EXPR getFuture(EXPR e, RandomDataGenerator rand, Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants, Map<RDDL.TYPE_NAME, RDDL.OBJECTS_DEF> objects, HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes, HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables )   {
                try {
                    return e.sampleDeterminization(rand,constants,objects, hmtypes, hm_variables);
                } catch (Exception e1) {
                    e1.printStackTrace();
                }return null;

            }
        }, MEAN {
            @Override
            public EXPR getFuture(EXPR e, RandomDataGenerator rand, Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants, Map<RDDL.TYPE_NAME, RDDL.OBJECTS_DEF> objects,  HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes, HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables  ) {
                try {
                    return e.getMean(constants, objects, hmtypes, hm_variables );
                } catch (Exception e1) {
                    e1.printStackTrace();
                }
                return null;
            }
        };

        public abstract EXPR getFuture( final EXPR e ,
                                        final RandomDataGenerator rand,
                                        final Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants,
                                        final Map<RDDL.TYPE_NAME, RDDL.OBJECTS_DEF> objects,
                                        final HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes,
                                        final HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables  );
    }

    protected int num_futures;
    protected int lookahead;
    protected FUTURE_SAMPLING future_gen;

    private static final RDDL.TYPE_NAME
            future_TYPE = new RDDL.TYPE_NAME( "future" );
    protected ArrayList< LCONST > future_TERMS = new ArrayList<>();
    protected enum HINDSIGHT_STRATEGY {
        ROOT, ALL_ACTIONS, CONSENSUS
    }
    protected HINDSIGHT_STRATEGY hindsight_method;
    protected HashMap< HashMap<EXPR, Object>, Integer >
            all_votes = new HashMap<>();

    /*COMPETITION PARAMETERS*/

    protected Boolean DO_GUROBI_INITIALIZATION = false;
    protected Boolean DO_NPWL_PWL = true;

    protected double TIME_LIMIT_MINS = 10;

    protected Policy random_policy = null;

    public HOPPlanner(String n_futures, String n_lookahead, String inst_name,
                      String gurobi_timeout, String future_gen_type, String hindsight_strat, String rand_seed,
                      RDDL rddl_object, State s) throws Exception {

        this.num_futures      = Integer.parseInt(n_futures);
        this.lookahead        = Integer.parseInt(n_lookahead);
        this.future_gen       = FUTURE_SAMPLING.valueOf(future_gen_type);
        this.hindsight_method = HINDSIGHT_STRATEGY.valueOf(hindsight_strat);
        this.TIME_LIMIT_MINS  = Double.valueOf(gurobi_timeout);
        initializeCompetitionRDDL(rddl_object, inst_name, s);
        this.setRandSeed(Long.parseLong(rand_seed));
        this.random_policy = new RandomPolicy();

        this.objects = new HashMap<>( rddl_instance._hmObjects );
        if( rddl_nonfluents != null && rddl_nonfluents._hmObjects != null ){
            objects.putAll( rddl_nonfluents._hmObjects );
        }

        getConstants();

        for( Entry<PVAR_NAME, RDDL.PVARIABLE_DEF> entry : rddl_state._hmPVariables.entrySet() ){

            final RDDL.TYPE_NAME rddl_type = entry.getValue()._typeRange;
            RDDL.TYPE_DEF tdef = rddl_state._hmTypes.get(rddl_type);

            char grb_type = 'Z';

            if(tdef instanceof RDDL.ENUM_TYPE_DEF){
                grb_type = GRB.INTEGER;

            }else{
                grb_type = rddl_type.equals( RDDL.TYPE_NAME.BOOL_TYPE ) ? GRB.BINARY :
                        rddl_type.equals( RDDL.TYPE_NAME.INT_TYPE ) ? GRB.INTEGER : GRB.CONTINUOUS;
            }

            assert(grb_type != 'Z');

            this.object_type_name.put(entry.getKey(), entry.getValue()._alParamTypes);
            //object_val_mapping.put(object_type_name.get(entry.getKey()),  rddl_state._hmObject2Consts.get(object_type_name.get(entry.getKey()))   );

            this.type_map.put( entry.getKey(), grb_type );

        }
    }

    protected void initializeCompetitionRDDL(final RDDL rddl_object,
                                             String instanceName, State s) throws Exception{
        //rddl.Policy members
        this._rddl = this.rddl_obj;
        this._sInstanceName = instanceName;
        //this class members
        this.rddl_obj      = rddl_object;
        this.instance_name = instanceName; //instance_rddl._tmInstanceNodes.keySet().iterator().next() ;
        //domain_rddl._tmDomainNodes.keySet().iterator().next();

        rddl_instance      = rddl_obj._tmInstanceNodes.get( instance_name );
        this.domain_name   = rddl_instance._sDomain;
        if (rddl_instance == null){
            throw new Exception("Instance '" + instance_name +
                    "' not found, choices are " + rddl_obj._tmInstanceNodes.keySet());
        }

        rddl_nonfluents = null;
        if (rddl_instance._sNonFluents != null){
            rddl_nonfluents = rddl_obj._tmNonFluentNodes.get(rddl_instance._sNonFluents);
        }

        rddl_domain = rddl_obj._tmDomainNodes.get(rddl_instance._sDomain);
        if ( rddl_domain == null){
            throw new Exception("Could not get domain '" +
                    rddl_instance._sDomain + "' for instance '" + instance_name + "'");
        }

        if (rddl_nonfluents != null && !rddl_instance._sDomain.equals(rddl_nonfluents._sDomain)){
            throw new Exception("Domain name of instance and fluents do not match: " +
                    rddl_instance._sDomain + " vs. " + rddl_nonfluents._sDomain);
        }

        this.rddl_state = new rddl.State();
        this.rddl_state = s;
        this.rddl_state_vars  = collectGroundings( rddl_state._alStateNames );
        this.rddl_action_vars = collectGroundings( rddl_state._alActionNames );
        this.rddl_observ_vars = collectGroundings( rddl_state._alObservNames );
        this.rddl_interm_vars = collectGroundings( rddl_state._alIntermNames );

        this.string_state_vars  = cleanMap( rddl_state_vars );
        this.string_action_vars = cleanMap( rddl_action_vars );
        this.string_observ_vars = cleanMap( rddl_observ_vars );
        this.string_interm_vars = cleanMap( rddl_interm_vars );

        for(Map.Entry<PVAR_NAME, RDDL.PVARIABLE_DEF> entry : rddl_state._hmPVariables.entrySet()){
            PVAR_NAME temp_pvar = entry.getKey();
            if(entry.getValue() instanceof RDDL.PVARIABLE_STATE_DEF){
                Object val = ((RDDL.PVARIABLE_STATE_DEF) entry.getValue())._oDefValue;
                rddl_state_default.put(temp_pvar,val);
            }
        }
    }


    protected void getConstants() throws EvalException {
        System.out.println("------This is getConstants(Translate.java) ");
        ArrayList<PVAR_INST_DEF> all_consts = new ArrayList<PVAR_INST_DEF>();
        for( final PVAR_NAME p : rddl_state._alNonFluentNames ){
            ArrayList<ArrayList<LCONST>> atoms = rddl_state.generateAtoms( p );
            Object def = rddl_state.getDefaultValue(p);
            for( final ArrayList<LCONST> atom : atoms ){
                all_consts.add( new PVAR_INST_DEF(p._sPVarName, def, atom) );
            }
        }
        if( rddl_nonfluents != null && rddl_nonfluents._alNonFluents != null ){
            all_consts.addAll( rddl_nonfluents._alNonFluents );
        }

        constants.putAll( getConsts( all_consts ) );
        //This is overriding the default values ...
        hmtypes = rddl_state._hmTypes;
        hm_variables = rddl_state._hmPVariables;
        if(rddl_state._nonfluents!=null){
            for(PVAR_NAME pname : rddl_state._nonfluents.keySet()){
                HashMap< ArrayList<LCONST>, Object>  val = rddl_state._nonfluents.get(pname);
                for(ArrayList<LCONST> array_lconst : val.keySet()){
                    constants.get(pname).put(array_lconst,val.get(array_lconst));
                }
            }
        }
        //constants.putAll( getConsts(  rddl_nonfluents._alNonFluents ) );
    }

    protected HashMap< PVAR_NAME, HashMap< ArrayList<LCONST>, Object> >
    getConsts(ArrayList<PVAR_INST_DEF> consts) {
        HashMap<PVAR_NAME, HashMap<ArrayList<LCONST>, Object>> ret
                = new HashMap< PVAR_NAME, HashMap< ArrayList<LCONST>, Object> >();
        for( final PVAR_INST_DEF p : consts ){
            if( ret.get(p._sPredName) == null ){
                ret.put( p._sPredName, new HashMap<ArrayList<LCONST>, Object>() );
            }
            HashMap<ArrayList<LCONST>, Object> inner_map = ret.get( p._sPredName );
            inner_map.put( p._alTerms, p._oValue );
            ret.put( p._sPredName, inner_map );//unnecessary
        }
        return ret;
    }



    public HashMap< PVAR_NAME, ArrayList<ArrayList<LCONST>> > collectGroundings( final ArrayList<PVAR_NAME> preds )
            throws EvalException {
        HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>> ret
                = new  HashMap<RDDL.PVAR_NAME, ArrayList<ArrayList<LCONST>>>();

        for( PVAR_NAME p : preds ){
            ArrayList<ArrayList<LCONST>> gfluents = rddl_state.generateAtoms(p);
            ret.put(p, gfluents);
            RDDL.PVARIABLE_DEF def = rddl_state._hmPVariables.get(p);
            pred_type.put( p, def._typeRange );
        }
        return ret ;
    }


    protected List<String> cleanMap( final HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>> map ) {
        List<String> ret = new ArrayList<String>();
        map.forEach( (a,b) -> b.forEach( m -> ret.add( CleanFluentName( a.toString() + m ) ) ) );
        return ret;
    }

    public static String CleanFluentName(String s) {
        s = s.replace("[", "__");
        s = s.replace("]", "");
        s = s.replace(", ", "_");
        s = s.replace(',','_');
        s = s.replace(' ','_');
        s = s.replace('-','_');
        s = s.replace("()","");
        s = s.replace("(", "__");
        s = s.replace(")", "");
        s = s.replace("$", "");
        if (s.endsWith("__"))
            s = s.substring(0, s.length() - 2);
        if (s.endsWith("__'")) {
            s = s.substring(0, s.length() - 3);
            s = s + "\'"; // Really need to escape it?  Don't think so.
        }
        return s;
    }

    private void initializeGRB( ) throws GRBException {
        this.GRB_log = GRB_LOGGING_ON ? domain_name + "__" + instance_name + ".grb" : "";

        this.grb_env = new GRBEnv(GRB_log);
        grb_env.set( GRB.DoubleParam.TimeLimit, TIME_LIMIT_MINS*60 );
        grb_env.set( GRB.DoubleParam.MIPGap, GRB_MIPGAP );
        grb_env.set( GRB.DoubleParam.Heuristics, GRB_HEURISTIC );
        grb_env.set( GRB.IntParam.InfUnbdInfo , GRB_INFUNBDINFO );
        grb_env.set( GRB.IntParam.DualReductions, GRB_DUALREDUCTIONS );
        grb_env.set( GRB.IntParam.IISMethod, GRB_IISMethod );
        grb_env.set( GRB.IntParam.NumericFocus, 3);
        grb_env.set( GRB.IntParam.MIPFocus, 1);
        grb_env.set( GRB.DoubleParam.FeasibilityTol, 1e-6 );// Math.pow(10,  -(State._df.getMaximumFractionDigits() ) ) ); 1e-6
        grb_env.set( GRB.DoubleParam.IntFeasTol,  1e-6);//Math.pow(10,  -(State._df.getMaximumFractionDigits() ) ) ); //Math.pow( 10 , -(1+State._df.getMaximumFractionDigits() ) ) );
        grb_env.set( GRB.DoubleParam.FeasRelaxBigM, RDDL.EXPR.M);
        grb_env.set( GRB.IntParam.Threads, 1 );
        grb_env.set( GRB.IntParam.Quad, 1 );
        grb_env.set( GRB.IntParam.Method, 1 );
        grb_env.set( GRB.DoubleParam.NodefileStart, 0.5 );

        //grb_env.set(GRB.IntParam.Presolve,0);
        //grb_env.set(DoubleParam.OptimalityTol, 1e-2);
        grb_env.set(GRB.IntParam.NumericFocus, 3);
//		grb_env.set( IntParam.SolutionLimit, 5);

        System.out.println("current nodefile directly " + grb_env.get( GRB.StringParam.NodefileDir ) );

        this.static_grb_model = new GRBModel( grb_env );
        //max
        static_grb_model.set( GRB.IntAttr.ModelSense, -1);
        static_grb_model.update();

    }

    @Override
    public ArrayList<PVAR_INST_DEF> getActions(State s) throws EvalException {
        HashMap<PVAR_NAME, HashMap<ArrayList<LCONST>, Object>> subs = getSubsWithDefaults( s );

        if( static_grb_model == null ){
            System.out.println("----Initializing Gurobi model----");
            firstTimeModel( );
        }
        try {
            Pair<Map<EXPR, Double>,Integer> out_put = doPlan( subs, RECOVER_INFEASIBLE );
            Map<EXPR, Double> ret_expr = out_put._o1;
            //This is the exit code.
            int exit_code = out_put._o2;
            //ArrayList<ArrayList<PVAR_INST_DEF>> ret_list = new ArrayList<ArrayList<PVAR_INST_DEF>>()
            ret_list.clear();
            ArrayList<PVAR_INST_DEF> ret = getRootActions(ret_expr,s);
            //System.out.println("####################################################");

            if(exit_code ==3 ){
                try {
                    throw new Exception("Model Infeasiblity Excepiton");
                }catch(Exception e){
                    cleanUp(static_grb_model);
                    throw e;
                }
            }

            cleanUp(static_grb_model);
            return ret;
        } catch (Exception e) {
            e.printStackTrace();
        }
        return null;
    }



    private void firstTimeModel( ) {
        try {
            System.out.println("1. initializeGRB \n" +
                    "2. addExtraPredicates \n" +
                    "3. addAllVariables \n" +
                    "4. translateConstraints");
            initializeGRB( );
            addExtraPredicates();
            addAllVariables();
            translateConstraints( this.static_grb_model );
            translateReward( this.static_grb_model , true );
        } catch (Exception e) {
            e.printStackTrace();
        }

    }



    public Pair<Map< EXPR, Double >,Integer> doPlan(HashMap<PVAR_NAME, HashMap<ArrayList<LCONST>, Object>> subs ,
                                                    final boolean recover ) throws Exception {

        System.out.println("----In doPlan-------");

        System.out.println("----Translating CPTs with random Futures----");
        translateCPTs( subs, static_grb_model , false );

        System.out.println("----Translating reward with random Futures----");
        translateReward( static_grb_model , false);

        System.out.println("--------------Initial State-------------");
        translateInitialState( static_grb_model, subs );

        int exit_code = -1;
        try{
            System.out.println("----STARTING OPTIMIZATION----");
            grb_env.set( GRB.DoubleParam.TimeLimit, TIME_LIMIT_MINS*60 );
            if(OUTPUT_NAME_MAP_FILE){
                outputNAMEMAPFile(static_grb_model);
            }
            exit_code = goOptimize( static_grb_model);
        }

        catch( GRBException exc ){
            int error_code = exc.getErrorCode();
            System.out.println("Error code : " + error_code );
            if( recover ){//error_code == GRB.ERROR_OUT_OF_MEMORY && recover ){
//				cleanUp(static_grb_model);
                handleOOM( static_grb_model );
                return doPlan( subs, RECOVER_INFEASIBLE );//can cause infinite loop if set to true
            }else{
                System.out.println("The Solution is Infeasible");
                throw exc;
            }
        }
        finally{
            System.out.println("Exit code " + exit_code );
        }

        Map< EXPR, Double > ret  = outputResults( static_grb_model);
        //Map< EXPR, Double > ret1 = outputAllResults(static_grb_model);

        if( OUTPUT_LP_FILE ) {
            outputLPFile( static_grb_model );
        }
        //This prints the model Summary
        modelSummary( static_grb_model );
        System.out.println("----> Cleaning up the Gurobi Model");
        cleanUp( static_grb_model );
        return new Pair<>(ret,exit_code);
    }


    protected void addExtraPredicates() {
        System.out.println("----Adding Extra Predicates");
        removeExtraPredicates();
        for( int t = 0 ; t < lookahead; ++t ){
            //[$time0,$time1,$time2,$time3]
            TIME_TERMS.add( new RDDL.OBJECT_VAL( "time" + t ) );
        }
        this.objects.put( TIME_TYPE,
                new RDDL.OBJECTS_DEF(  TIME_TYPE._STypeName, TIME_TERMS ) );

        if( future_gen.equals( FUTURE_SAMPLING.MEAN ) ){
            this.num_futures = 1;
        }
        for( int f = 0 ; f < this.num_futures; ++f ){
            this.future_TERMS.add( new RDDL.OBJECT_VAL( "future" + f ) );
        }
        this.objects.put( future_TYPE,
                new RDDL.OBJECTS_DEF(  future_TYPE._STypeName, future_TERMS ) );
    }


    protected void removeExtraPredicates() {
        System.out.println("-----removeExtraPredicates (Overrided)----");

        TIME_TERMS.clear();
        this.objects.remove( TIME_TYPE );
        if( future_TERMS != null ){
            future_TERMS.clear();
        }
        if( this.objects != null ){
            this.objects.remove( future_TYPE );
        }
    }



    protected HashMap<PVAR_NAME, HashMap<ArrayList<LCONST>, Object>> getSubsWithDefaults(final State state) throws EvalException {

        final HashMap<PVAR_NAME, HashMap<ArrayList<LCONST>, Object>> ret
                = new HashMap< PVAR_NAME, HashMap< ArrayList<LCONST>, Object> >();

        for( final PVAR_NAME stateVar : state._alStateNames ){
            if( !ret.containsKey(stateVar) ){
                ret.put( stateVar, new HashMap<>() );
            }
            final ArrayList<ArrayList<LCONST>> possible_terms
                    = state.generateAtoms(stateVar);
            if( possible_terms.isEmpty() ){
                possible_terms.add(new ArrayList<LCONST>());
            }
            for( ArrayList<LCONST> term_assign : possible_terms ){
                if( state._state.containsKey(stateVar)
                        && state._state.get(stateVar).containsKey(term_assign) ){
                    ret.get( stateVar ).put( term_assign,
                            state._state.get(stateVar).get(term_assign) );
                }else{
                    ret.get(stateVar).put(term_assign,
                            state.getDefaultValue(stateVar) );
                }
            }
        }
        return ret;
    }




    protected void addAllVariables( ) throws Exception{
        System.out.println("-----addAllVaribles----");
        HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>> src
                = new HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>>();

        src.putAll( rddl_state_vars ); src.putAll( rddl_action_vars ); src.putAll( rddl_interm_vars ); src.putAll( rddl_observ_vars );

        src.forEach( new BiConsumer<PVAR_NAME, ArrayList<ArrayList<LCONST>> >() {
            @Override
            public void accept(PVAR_NAME pvar, ArrayList<ArrayList<LCONST>> u) {
                u.stream().forEach( new Consumer<ArrayList<LCONST>>() {
                    @Override
                    public void accept(ArrayList<LCONST> terms) {
                        try{
                            EXPR pvar_expr = new PVAR_EXPR(pvar._sPVarName, terms ).addTerm(TIME_PREDICATE, constants, objects, hmtypes,hm_variables )
                                    .addTerm( future_PREDICATE, constants, objects,  hmtypes,hm_variables  );
                            TIME_TERMS.stream().forEach( new Consumer<LCONST>() {
                                @Override
                                public void accept(LCONST time_term ) {
                                    try{
                                        EXPR this_t = pvar_expr.substitute( Collections.singletonMap( TIME_PREDICATE, time_term),
                                                constants, objects,  hmtypes,hm_variables  );
                                        future_TERMS.stream().forEach( new Consumer< LCONST >() {
                                            @Override
                                            public void accept(LCONST future_term) {
                                                //System.out.println(this_t.toString());
                                                try{
                                                    EXPR this_tf = this_t.substitute(
                                                            Collections.singletonMap( future_PREDICATE, future_term ),
                                                            constants, objects, hmtypes, hm_variables  );
                                                    synchronized( static_grb_model ){
                                                        try {
                                                            GRBVar gvar = this_tf.getGRBConstr( GRB.EQUAL, static_grb_model, constants, objects, type_map, hmtypes, hm_variables);
                                                        } catch (Exception e) {
                                                            e.printStackTrace();
                                                        }
                                                        saved_expr.add( this_tf );
                                                    }
                                                }
                                                catch (Exception e){
                                                    e.printStackTrace();
                                                }
                                            }
                                        });
                                    } catch (Exception e) {e.printStackTrace(); }

                                }
                            });
                        } catch (Exception e) {e.printStackTrace(); }
                    }
                });
            }
        });
    }

    protected void translateConstraints(final GRBModel grb_model) throws Exception {
        GRBExpr old_obj = grb_model.getObjective();

        System.out.println("----Translating Constraints(Overrided)----");
        ArrayList<RDDL.BOOL_EXPR> constraints = new ArrayList<RDDL.BOOL_EXPR>();
        constraints.addAll( rddl_state._alActionPreconditions );
        constraints.addAll( rddl_state._alStateInvariants );

        constraints.stream().forEach( new Consumer<RDDL.BOOL_EXPR>() {
            @Override
            public void accept(RDDL.BOOL_EXPR e) {
                try{
                    System.out.println(e.toString());
                    EXPR temp_expr = null;
                    EXPR non_stationary_e  = null;
                    if(SUBSTITUTE_CACHE_USE){
                        Pair<String, String> key
                                = new Pair(e.toString(), Collections.EMPTY_MAP.toString());
                        //Substitute_expression_cache --> stores substituted_expressions --> Key is a Pair<Expression,subs> --> value is substitutued Expression.
                        if(substitute_expression_cache.containsKey(key)){
                            temp_expr = substitute_expression_cache.get(key);
                        }else{
                            //((RDDL.CONN_EXPR) ((RDDL.QUANT_EXPR) e)._expr)._alSubNodes.get(1).substitute(Collections.EMPTY_MAP,constants,objects);
                            temp_expr = e.substitute(Collections.EMPTY_MAP, constants, objects, hmtypes, hm_variables );
                            substitute_expression_cache.put(key, temp_expr);
                        }
                        assert(temp_expr != null);
                        non_stationary_e = temp_expr.addTerm(TIME_PREDICATE, constants, objects, hmtypes, hm_variables )
                                .addTerm(future_PREDICATE, constants, objects, hmtypes, hm_variables );

                    }else{
                        non_stationary_e = e.substitute(Collections.EMPTY_MAP,constants,objects, hmtypes, hm_variables )
                                .addTerm(TIME_PREDICATE,constants,objects, hmtypes, hm_variables )
                                .addTerm(future_PREDICATE,constants,objects, hmtypes, hm_variables );
                    }
                    final EXPR non_stationary_e1 = non_stationary_e;

                    TIME_TERMS.stream().forEach(new Consumer<LCONST>() {
                        @Override
                        public void accept(LCONST time_term) {
                            future_TERMS.stream().forEach(new Consumer<LCONST>() {
                                @Override
                                public void accept(LCONST future_term) {

                                    try {
                                        EXPR this_tf = non_stationary_e1
                                                .substitute(Collections.singletonMap(TIME_PREDICATE, time_term), constants, objects, hmtypes, hm_variables )
                                                .substitute(Collections.singletonMap(future_PREDICATE, future_term), constants, objects, hmtypes, hm_variables );

//                                        System.out.println(">>>>" + this_tf.toString());
                                        synchronized (grb_model) {

                                            if (SHOW_GUROBI_ADD)
                                                System.out.println(this_tf);
                                            GRBVar constrained_var = this_tf.getGRBConstr(
                                                    GRB.EQUAL, grb_model, constants, objects,
                                                    type_map, hmtypes, hm_variables);

                                            String nam = RDDL.EXPR.getGRBName(this_tf);
                                            GRBConstr this_constr = grb_model.addConstr(
                                                    constrained_var, GRB.EQUAL, 1.0, nam);
                                            saved_expr.add(this_tf);
                                            saved_constr.add(this_constr);
                                            //saved_vars.add( constrained_var );
                                        }
                                    } catch (GRBException e) {
                                        e.printStackTrace();
                                    } catch (Exception e1) {
                                        e1.printStackTrace();
                                    }
                                }
                            });
                        }
                    });
                }catch (Exception e1){
                    e1.printStackTrace();
                }
            }
        });

        ArrayList<RDDL.BOOL_EXPR> hindsight_constraint = getHindSightConstraintExpr(hindsight_method);
        System.out.println("----hindsight constraint----"  + hindsight_constraint);

        hindsight_constraint.stream().forEach( new Consumer<RDDL.BOOL_EXPR>() {

            @Override
            public void accept( RDDL.BOOL_EXPR t) {
                synchronized( grb_model ){
                    GRBVar gvar = null;
                    try {
                        gvar = t.getGRBConstr( GRB.EQUAL, grb_model, constants,
                                objects, type_map, hmtypes, hm_variables);
                        GRBConstr this_constr = grb_model.addConstr( gvar,
                                GRB.EQUAL, 1.0, RDDL.EXPR.getGRBName(t) );
                        saved_expr.add( t ); // saved_vars.add( gvar );
                        saved_constr.add(this_constr);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        });

        grb_model.setObjective(old_obj);
        grb_model.update();

    }

    protected ArrayList<RDDL.BOOL_EXPR> getHindSightConstraintExpr(
            HINDSIGHT_STRATEGY hindsight_method )  {
        ArrayList<RDDL.BOOL_EXPR> ret = new ArrayList<RDDL.BOOL_EXPR>();
        System.out.println("----HindSightConstraintExpr (HOPTranslate.java)--");

        switch( hindsight_method ){
            case ALL_ACTIONS :
                rddl_action_vars.forEach( new BiConsumer<PVAR_NAME, ArrayList<ArrayList<LCONST>>> () {
                    public void accept( PVAR_NAME pvar , ArrayList<ArrayList<LCONST>> u) {
                        u.forEach( new Consumer< ArrayList<LCONST>>() {
                            @Override
                            public void accept(ArrayList<LCONST> terms) {
                                try {
                                    PVAR_EXPR pvar_expr = new PVAR_EXPR(pvar._sPVarName, terms);
                                    EXPR with_tf = pvar_expr.addTerm(TIME_PREDICATE, constants, objects, hmtypes, hm_variables )
                                            .addTerm(future_PREDICATE, constants, objects, hmtypes, hm_variables );

                                    for (final LCONST time : TIME_TERMS) {
                                        EXPR this_t = with_tf.substitute(Collections.singletonMap(TIME_PREDICATE, time), constants, objects, hmtypes, hm_variables );
                                        EXPR ref_expr = this_t.substitute(Collections.singletonMap(future_PREDICATE, future_TERMS.get(0)), constants, objects, hmtypes, hm_variables );

                                        for (final LCONST future : future_TERMS) {
                                            EXPR addedd = this_t.substitute(Collections.singletonMap(future_PREDICATE, future), constants, objects, hmtypes, hm_variables );
                                            try {
                                                ret.add(new RDDL.COMP_EXPR(ref_expr, addedd, RDDL.COMP_EXPR.EQUAL));
                                            } catch (Exception e) {
                                                e.printStackTrace();
                                            }
                                        }

                                    }
                                } catch (Exception e){e.printStackTrace();}
                            }
                        });
                    };
                });
                break;
            case CONSENSUS :
                //nothing to add
                break;
            case ROOT :
                System.out.println("----->ROOT");
                rddl_action_vars.forEach( new BiConsumer<PVAR_NAME, ArrayList<ArrayList<LCONST>>> () {
                    public void accept( PVAR_NAME pvar , ArrayList<ArrayList<LCONST>> u) {
                        u.forEach( new Consumer< ArrayList<LCONST>>() {
                            @Override
                            public void accept(ArrayList<LCONST> terms) {
                                try {
                                    PVAR_EXPR pvar_expr = new PVAR_EXPR(pvar._sPVarName, terms);
                                    EXPR with_tf = pvar_expr.addTerm(TIME_PREDICATE, constants, objects, hmtypes, hm_variables )
                                            .addTerm(future_PREDICATE, constants, objects, hmtypes, hm_variables );

                                    EXPR this_t = with_tf.substitute(Collections.singletonMap(TIME_PREDICATE,
                                            TIME_TERMS.get(0)), constants, objects, hmtypes, hm_variables );
                                    EXPR ref_expr = this_t.substitute(Collections.singletonMap(future_PREDICATE,
                                            future_TERMS.get(0)), constants, objects, hmtypes, hm_variables );

                                    for (final LCONST future : future_TERMS) {

                                        EXPR addedd = this_t.substitute(
                                                Collections.singletonMap(future_PREDICATE, future),
                                                constants, objects, hmtypes, hm_variables );
                                        if (SHOW_GUROBI_ADD)
                                            System.out.println(addedd.toString());
                                        try {
                                            ret.add(new RDDL.COMP_EXPR(ref_expr,
                                                    addedd, RDDL.COMP_EXPR.EQUAL));
                                        } catch (Exception e) {
                                            e.printStackTrace();
                                        }
                                    }

                                }catch (Exception e){e.printStackTrace();}
                            }
                        });
                    };
                });
                break;

            default: try{
                throw new Exception("Unknown hindsight strategy " + hindsight_method );
            }	catch( Exception exc ){
                exc.printStackTrace();
                //System.exit(1);
            }
        }

        System.out.println("----End HindSightConstraintExpr----");
        return ret;
    }


    protected void translateReward( final GRBModel grb_model,
                                    final boolean first_time ) throws Exception {
        System.out.println("---- Translate Reward ----");
        grb_model.setObjective( new GRBLinExpr() );
        grb_model.update();

        EXPR stationary_clear = rddl_state._reward;
        if ( !first_time && stationary_clear._bDet &&
                replace_reward_pwl == null ) {
            System.out.println("---Skipping reward---" );
            return;
        }else if( replace_reward_pwl != null ){
            stationary_clear = replace_reward_pwl;
        }else if( !stationary_clear._bDet ){
            stationary_clear = future_gen.getFuture(stationary_clear,
                    this._random, constants, objects, hmtypes, hm_variables);
        }

        // CHECK THIS HARISH
        Pair<String,String> key = new Pair<String, String>(stationary_clear.toString(),
                Collections.EMPTY_MAP.toString());
        if( substitute_expression_cache.containsKey( key ) ){
            stationary_clear = substitute_expression_cache.get(key);
        }else{
            stationary_clear = stationary_clear.substitute( Collections.EMPTY_MAP,
                    constants, objects, hmtypes, hm_variables );
            substitute_expression_cache.put(key, stationary_clear);
        }

        final EXPR non_stationary = stationary_clear.addTerm( TIME_PREDICATE ,
                constants, objects, hmtypes, hm_variables )
                .addTerm( future_PREDICATE, constants, objects, hmtypes, hm_variables );

        GRBLinExpr all_sum = new GRBLinExpr();
        //System.out.println(non_stationary);

        ArrayList<Integer> time_terms_indices = new ArrayList<Integer>( TIME_TERMS.size() );
        for( int i = 0 ; i < TIME_TERMS.size(); ++i ){
            time_terms_indices.add( i );
        }

        future_TERMS.stream().forEach( new Consumer<LCONST>() {
            @Override
            public void accept(final LCONST future_term) {

                time_terms_indices.stream().forEach( new Consumer<Integer>() {
                    @Override
                    public void accept(final Integer index){
                        try {
                            final LCONST time_term = TIME_TERMS.get(index);
                            //add discounting
                            final double cur_disc = Math.pow(rddl_instance._dDiscount, index);

                            final EXPR subs_tf = non_stationary.substitute(Collections.singletonMap(TIME_PREDICATE, time_term),
                                    constants, objects, hmtypes, hm_variables )
                                    .substitute(Collections.singletonMap(future_PREDICATE, future_term), constants, objects, hmtypes, hm_variables );
                            //System.out.println( subs_tf );//"Reward_" + time_term + "_" + future_term );

                            synchronized (grb_model) {

                                GRBVar this_future_var = subs_tf.getGRBConstr(
                                        GRB.EQUAL, grb_model, constants, objects,
                                        type_map, hmtypes, hm_variables);

                                all_sum.addTerm(cur_disc / num_futures, this_future_var);
                                //System.out.println(all_sum);
                            }
                        }
                        catch (Exception e){
                            e.printStackTrace();
                        }
                    }
                });
            }
        });
        grb_model.setObjective(all_sum);
        grb_model.update();

        if ( replace_reward_pwl != null ){
            replace_reward_pwl = null;
        }
    }

    protected void translateCPTs(
            final HashMap<PVAR_NAME,HashMap<ArrayList<LCONST>,Object>> subs,
            final GRBModel grb_model,
            final boolean first_time) throws GRBException {
        System.out.println("----translateCPTs----");
        //THIS FUNCTION IS CALLED FOR EVERY NEW STATE.
        //Timer timer1 = new Timer();
        final GRBExpr old_obj = grb_model.getObjective();

        final ArrayList<HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>>> src
                = new ArrayList< HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>> >();
        src.add( rddl_state_vars ); src.add( rddl_interm_vars ); src.add( rddl_observ_vars );

        final ArrayList<Integer> time_terms_indices = new ArrayList<Integer>( TIME_TERMS.size() );
        for( int i = 0 ; i < TIME_TERMS.size(); ++i ){
            time_terms_indices.add( i );
        }

        final ArrayList<Integer> future_terms_indices = new ArrayList<Integer>( future_TERMS.size() );
        for( int i = 0 ; i < future_TERMS.size(); ++i ){
            future_terms_indices.add( i );
        }
        //System.out.println("-------This is were we are sampling the future!!!!--");
        src.stream().forEach( new Consumer< HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST> > > >() {

            @Override
            public void accept(
                    HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>> t) {
                t.entrySet().stream().forEach( new Consumer< Entry<PVAR_NAME, ArrayList< ArrayList<LCONST>> > >() {

                    @Override
                    public void accept(Entry<PVAR_NAME, ArrayList<ArrayList<LCONST>>> entry ) {

                        final PVAR_NAME p = entry.getKey();
                        final ArrayList<ArrayList<LCONST>> all_terms = entry.getValue();
                        final ArrayList<Integer> term_indices = new ArrayList<>();
                        for( int idx = 0; idx < all_terms.size(); ++idx ){
                            term_indices.add(idx);
                        }

                        term_indices.stream().forEach(new Consumer<Integer>() {
                            @Override
                            public void accept(final Integer idx) {
                                try {
                                    ArrayList<LCONST> terms = all_terms.get(idx);
                                    CPF_DEF cpf = null;
                                    if (rddl_state_vars.containsKey(p)) {
                                        cpf = rddl_state._hmCPFs.get(new PVAR_NAME(p._sPVarName + "'"));
                                    } else {
                                        cpf = rddl_state._hmCPFs.get(new PVAR_NAME(p._sPVarName));
                                    }

                                    if ( !first_time && cpf._exprEquals._bDet &&
                                            !replace_cpf_pwl.containsKey(p)) {
                                        System.out.println("---Skipping" + p );
                                        return;
                                    }

                                    Map<LVAR, LCONST> subs = getSubs(cpf._exprVarName._alTerms, terms);

                                    EXPR new_lhs_stationary = cpf._exprVarName.substitute(subs, constants, objects, hmtypes, hm_variables);
                                    EXPR new_rhs_stationary = null;

                                    if (replace_cpf_pwl.containsKey(p)) {
                                        new_rhs_stationary = replace_cpf_pwl.get(p).get(idx);
                                    } else {
                                        final Pair<String,String> key = new Pair<>(
                                                cpf._exprEquals.toString(), subs.toString());
                                        if ( substitute_expression_cache .containsKey(key) ){
                                            new_rhs_stationary = substitute_expression_cache.get(key);
                                        }else{
                                            new_rhs_stationary = cpf._exprEquals.substitute(subs,
                                                    constants, objects, hmtypes, hm_variables);
                                            substitute_expression_cache.put(key, new_rhs_stationary);
                                        }
                                    }
                                    final EXPR new_rhs_stationary1 = new_rhs_stationary;

                                    try {
                                        EXPR lhs_with_tf = new_lhs_stationary.addTerm(TIME_PREDICATE, constants, objects, hmtypes, hm_variables)
                                                .addTerm(future_PREDICATE, constants, objects, hmtypes, hm_variables);
                                        EXPR rhs_with_tf = new_rhs_stationary.addTerm(TIME_PREDICATE, constants, objects, hmtypes, hm_variables)
                                                .addTerm(future_PREDICATE, constants, objects, hmtypes, hm_variables);
                                        System.out.println(lhs_with_tf + " " + rhs_with_tf);

                                        time_terms_indices.stream().forEach(new Consumer<Integer>() {
                                            @Override
                                            public void accept(final Integer time_term_index) {
                                                try {
                                                    EXPR lhs_with_f = null;
                                                    if (rddl_state_vars.containsKey(p)) {
                                                        if (time_term_index == lookahead - 1) {
                                                            return;
                                                        }

                                                        lhs_with_f = lhs_with_tf.substitute(
                                                                Collections.singletonMap(TIME_PREDICATE,
                                                                        TIME_TERMS.get(time_term_index + 1)),
                                                                constants, objects, hmtypes, hm_variables);
                                                    } else {
                                                        lhs_with_f = lhs_with_tf.substitute(
                                                                Collections.singletonMap(TIME_PREDICATE,
                                                                        TIME_TERMS.get(time_term_index)),
                                                                constants, objects, hmtypes, hm_variables);
                                                    }
                                                    final EXPR rhs_with_f = rhs_with_tf.substitute(
                                                            Collections.singletonMap(TIME_PREDICATE,
                                                                    TIME_TERMS.get(time_term_index)), constants,
                                                            objects, hmtypes, hm_variables);
                                                    final EXPR lhs_with_f1 =  lhs_with_f;

                                                    future_terms_indices.stream().forEach(
                                                            new Consumer<Integer>() {
                                                                public void accept(Integer future_term_index) {
                                                                    try {
                                                                        EXPR lhs = lhs_with_f1.substitute(
                                                                                Collections.singletonMap(future_PREDICATE, future_TERMS.get(future_term_index)), constants, objects, hmtypes, hm_variables);
                                                                        EXPR rhs = rhs_with_f.substitute(
                                                                                Collections.singletonMap(future_PREDICATE, future_TERMS.get(future_term_index)), constants, objects, hmtypes, hm_variables);
                                                                        //System.out.println("Something related to future is happening");
                                                                        EXPR lhs_future = future_gen.getFuture(
                                                                                lhs, _random, constants,
                                                                                objects, hmtypes, hm_variables);
                                                                        EXPR rhs_future = future_gen.getFuture(
                                                                                rhs,_random, constants,
                                                                                objects, hmtypes, hm_variables);

                                                                        synchronized (grb_model) {

                                                                            try {
                                                                                GRBVar lhs_var = lhs_future.getGRBConstr(
                                                                                        GRB.EQUAL, grb_model, constants, objects, type_map, hmtypes, hm_variables);

                                                                                GRBVar rhs_var = rhs_future.getGRBConstr(
                                                                                        GRB.EQUAL, grb_model, constants, objects, type_map, hmtypes, hm_variables);

                                                                                //System.out.println( lhs_future.toString()+"="+rhs_future.toString() );
                                                                                final String nam = RDDL.EXPR.getGRBName(lhs_future) + "=" + RDDL.EXPR.getGRBName(rhs_future);
//																		System.out.println(nam);;

                                                                                GRBConstr this_constr
                                                                                        = grb_model.addConstr(lhs_var, GRB.EQUAL, rhs_var, nam);

                                                                                if( !new_rhs_stationary1._bDet || replace_cpf_pwl.containsKey(p) ){
                                                                                    to_remove_constr.add(this_constr);
                                                                                    to_remove_expr.add(lhs_future);
                                                                                    to_remove_expr.add(rhs_future);
                                                                                }

                                                                            } catch (GRBException e) {
                                                                                e.printStackTrace();
                                                                                //System.exit(1);
                                                                            } catch (Exception e) {
                                                                                e.printStackTrace();
                                                                            }

                                                                        }

                                                                    } catch (Exception e) {
                                                                        e.printStackTrace();
                                                                    }
                                                                }
                                                            });
                                                } catch (Exception e) {
                                                    e.printStackTrace();
                                                }
                                            }
                                        });
                                    } catch (Exception e) {
                                        e.printStackTrace();
                                    }
                                } catch (Exception e) {
                                    e.printStackTrace();
                                }
                            }
                        });
                    }});}});

        grb_model.setObjective(old_obj);
        grb_model.update();
    }

    protected void translateInitialState( final GRBModel grb_model,
                                          final HashMap<PVAR_NAME, HashMap<ArrayList<LCONST>, Object>> subs ) throws GRBException {
        System.out.println("----Translating Initial State----");

        final GRBExpr old_obj = grb_model.getObjective();

        for( final PVAR_NAME p : rddl_state_vars.keySet() ){
            for( final ArrayList<LCONST> terms : rddl_state_vars.get( p ) ){
                try{

                    Object rhs = null;
                    if( subs.containsKey( p ) && subs.get( p ).containsKey( terms ) ){
                        rhs = subs.get(p).get( terms );
                    }else{
                        rhs = rddl_state.getDefaultValue(p);
                    }

                    EXPR rhs_expr = null;
                    if( rhs instanceof Boolean ){
                        rhs_expr = new BOOL_CONST_EXPR( (boolean) rhs );
                    }else if( rhs instanceof Double ){
                        rhs_expr = new REAL_CONST_EXPR( (double)rhs );
                    }else if( rhs instanceof Integer ){
                        rhs_expr = new INT_CONST_EXPR( (int)rhs );
                    }else if(rhs instanceof ENUM_VAL){
                        rhs_expr = (ENUM_VAL)rhs;
                    }

                    GRBVar rhs_var = null;

                    rhs_var = rhs_expr.getGRBConstr( GRB.EQUAL, grb_model, constants, objects, type_map, hmtypes, hm_variables);

                    PVAR_EXPR stationary_pvar_expr = new PVAR_EXPR( p._sPVarName, terms );
                    EXPR non_stationary_pvar_expr = stationary_pvar_expr
                            .addTerm( TIME_PREDICATE, constants, objects, hmtypes, hm_variables )
                            .addTerm( future_PREDICATE, constants, objects, hmtypes, hm_variables );

                    for( int future_id = 0 ; future_id < num_futures; ++future_id ){
                        try {
                            EXPR this_future_init_state = non_stationary_pvar_expr
                                    .substitute(Collections.singletonMap(TIME_PREDICATE, TIME_TERMS.get(0)), constants, objects, hmtypes, hm_variables )
                                    .substitute(Collections.singletonMap(future_PREDICATE, future_TERMS.get(future_id)), constants, objects, hmtypes, hm_variables );

                            GRBVar lhs_var = null;
                            lhs_var = this_future_init_state.getGRBConstr(GRB.EQUAL, grb_model, constants, objects, type_map, hmtypes, hm_variables);
                            String nam = null;
                            if(rhs_expr instanceof  ENUM_VAL){
                                nam = RDDL.EXPR.getGRBName(this_future_init_state) +
                                        "=" + RDDL.EXPR.getGRBName( new INT_CONST_EXPR(((ENUM_VAL)rhs_expr).enum_to_int(hmtypes,hm_variables)));

                            }else{
                                nam = RDDL.EXPR.getGRBName(this_future_init_state) +
                                        "=" + RDDL.EXPR.getGRBName(rhs_expr);

                            }


                            GRBConstr this_constr = grb_model.addConstr(lhs_var, GRB.EQUAL, rhs_var, nam);

//					System.out.println( this_constr.get(StringAttr.ConstrName) );
//					saved_vars.add( lhs_var ); saved_expr.add( this_future_init_state );
//					saved_vars.add( rhs_var ); saved_expr.add( rhs_expr );
                            to_remove_expr.add(this_future_init_state);
                            to_remove_constr.add(this_constr);
                        }
                        catch (Exception e){
                            e.printStackTrace();
                        }


                    }

                } catch (Exception e) { e.printStackTrace(); }

            }
        }

        grb_model.setObjective(old_obj);
        grb_model.update();
    }

    protected ArrayList<PVAR_INST_DEF> getRootActions(
            final Map<EXPR, Double> ret_expr, final State s) {
        System.out.println("------Gettting RootActions (Overrided) -------");
        final ArrayList<PVAR_INST_DEF> ret = new ArrayList<PVAR_INST_DEF>();
        if( ret_expr == null ){
            System.out.println("The Solution is Unbounded / inFeasible, Returing nothing");
            return ret;

        }

        future_TERMS.stream().forEach( new Consumer<LCONST>() {
            @Override
            public void accept(LCONST future_term) {

                HashMap<EXPR, Object> this_future_actions = new HashMap<EXPR, Object>();

                rddl_action_vars.entrySet().stream().forEach( new Consumer< Map.Entry< PVAR_NAME, ArrayList<ArrayList<LCONST>> > >() {
                    @Override
                    public void accept( Map.Entry< PVAR_NAME , ArrayList<ArrayList<LCONST>> > entry ) {
                        final PVAR_NAME pvar = entry.getKey();
                        final Object def_val = rddl_state.getDefaultValue( pvar );

                        entry.getValue().stream().forEach( new Consumer< ArrayList<LCONST> >() {
                            @Override
                            public void accept(ArrayList<LCONST> terms ) {
                                try {
                                    final PVAR_EXPR action_var = new PVAR_EXPR(pvar._sPVarName, terms);
                                    EXPR this_action_var = action_var.addTerm(TIME_PREDICATE, constants, objects, hmtypes, hm_variables )
                                            .addTerm(future_PREDICATE, constants, objects, hmtypes, hm_variables )
                                            .substitute(Collections.singletonMap(TIME_PREDICATE, TIME_TERMS.get(0)), constants, objects, hmtypes, hm_variables )
                                            .substitute(Collections.singletonMap(future_PREDICATE, future_term), constants, objects, hmtypes, hm_variables );
                                    assert (ret_expr.containsKey(this_action_var));

                                    Object value = sanitize(action_var._pName, ret_expr.get(this_action_var));

                                    if (!value.equals(def_val)) {
                                        this_future_actions.put(action_var, value);
                                    }
                                }
                                catch (Exception e){
                                    e.printStackTrace();
                                }
                            }

                        });
                    }
                } );

                if( all_votes.containsKey( this_future_actions ) ){
                    all_votes.put( this_future_actions,  all_votes.get( this_future_actions ) + 1 );
                }else{
                    all_votes.put( this_future_actions,  1 );
                }

            }
        });

        System.out.println("Votes  " + all_votes );
        HashMap<EXPR, Object> chosen_vote = null;


        if( hindsight_method.equals( HINDSIGHT_STRATEGY.CONSENSUS ) ){
            final int max_votes = all_votes.values().stream().mapToInt(m->m).max().getAsInt();
            List<Entry<HashMap<EXPR, Object>, Integer>> ties  =
                    all_votes.entrySet().stream().filter( m -> (m.getValue()==max_votes) )
                            .collect( Collectors.toList() );
            if(ties.size()==1){
                chosen_vote=ties.get(0).getKey();
            }else{
                chosen_vote = ties.get( this._random.nextInt(0,ties.size()) ).getKey();
            }
        }

        final HashMap<EXPR, Object> winning_vote = chosen_vote;
        ArrayList<Double> violations = new ArrayList<>();
        rddl_action_vars.entrySet().stream().forEach( new Consumer< Map.Entry< PVAR_NAME, ArrayList<ArrayList<LCONST>> > >() {
            @Override
            public void accept( Map.Entry< PVAR_NAME , ArrayList<ArrayList<LCONST>> > entry ) {
                final PVAR_NAME pvar = entry.getKey();
                //assuming number here
                final Object def_val = rddl_state.getDefaultValue( pvar );
                entry.getValue().stream().forEach( new Consumer< ArrayList<LCONST> >() {
                    @Override
                    public void accept(ArrayList<LCONST> terms ) {

                        final PVAR_EXPR action_var = new PVAR_EXPR( pvar._sPVarName, terms );
                        EXPR lookup = null;
                        Object ret_value = Double.NaN;

                        switch( hindsight_method ){
                            case ALL_ACTIONS :
                            case ROOT :
                                try {
                                    lookup = action_var
                                            .addTerm(TIME_PREDICATE, constants, objects, hmtypes, hm_variables )
                                            .addTerm(future_PREDICATE, constants, objects, hmtypes, hm_variables )
                                            .substitute(Collections.singletonMap(TIME_PREDICATE, TIME_TERMS.get(0)), constants, objects, hmtypes, hm_variables )
                                            .substitute(Collections.singletonMap(future_PREDICATE, future_TERMS.get(0)), constants, objects, hmtypes, hm_variables );
                                    assert (ret_expr.containsKey(lookup));
                                    ret_value = sanitize(action_var._pName, ret_expr.get(lookup));
                                }catch (Exception e){
                                    e.printStackTrace();
                                }
                                break;
                            case CONSENSUS :
                                ret_value = winning_vote.containsKey( action_var ) ?
                                        winning_vote.get( action_var ) : def_val;
                                break;

                            default : try{
                                throw new Exception("unknown hindisght strategy");
                            }catch( Exception exc ){
                                exc.printStackTrace();

                            }
                        }

                        if( ! ret_value.equals( def_val ) ){
                            synchronized( ret ){
                                ret.add( new PVAR_INST_DEF( pvar._sPVarName, ret_value, terms ) );
                            }
                        }

                        final double ref_value = (ret_value instanceof Boolean) ?
                                ((boolean) ret_value ? 1 : 0) : ((Number)ret_value).doubleValue();

                        future_TERMS.stream().forEach( new Consumer<LCONST>() {
                            @Override
                            public void accept(LCONST future_term) {
                                try {
                                    EXPR this_action_var = action_var.addTerm(TIME_PREDICATE, constants, objects, hmtypes, hm_variables )
                                            .addTerm(future_PREDICATE, constants, objects, hmtypes, hm_variables )
                                            .substitute(Collections.singletonMap(TIME_PREDICATE, TIME_TERMS.get(0)), constants, objects, hmtypes, hm_variables )
                                            .substitute(Collections.singletonMap(future_PREDICATE, future_term), constants, objects, hmtypes, hm_variables );

                                    assert (ret_expr.containsKey(this_action_var));

                                    double value = ret_expr.get(this_action_var);
                                    final double this_vio = Math.abs(ref_value - value);

                                    final Double added = new Double(this_vio);
                                    assert (added != null);

                                    synchronized (violations) {
                                        violations.add(added);
                                    }
                                }catch (Exception e){
                                    e.printStackTrace();
                                }
                            }
                        });
                    }
                });
            }
        });
        System.out.println("Total violation of root action " + violations.stream().mapToDouble(m->m).sum() );
        System.out.println("Average absolute violation of root action " + violations.stream().mapToDouble(m->m).average().getAsDouble() );
        violations.clear();
        all_votes.clear();
        return ret;

    }




    protected int goOptimize(final GRBModel grb_model) throws GRBException {

        grb_model.update();
        System.out.println("Optimizing.............");
        grb_model.optimize();
        return grb_model.get( GRB.IntAttr.Status );
    }

    protected Map< EXPR, Double > outputResults( final GRBModel grb_model ) throws GRBException{

        System.out.println("------This is output results for GRB MODEL -------");
//		DecimalFormat df = new DecimalFormat("#.##########");
//		df.setRoundingMode( RoundingMode.DOWN );
        if( grb_model.get( GRB.IntAttr.SolCount ) == 0 ){
            return null;
        }

        Map< EXPR, Double > ret = new HashMap< EXPR, Double >();

        HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>> src = new HashMap<>();
        src.putAll( rddl_action_vars );
        src.putAll( rddl_interm_vars );
        src.putAll( rddl_state_vars );

        src.forEach( new BiConsumer<PVAR_NAME, ArrayList<ArrayList<LCONST> > >( ) {

            @Override
            public void accept(PVAR_NAME pvar,
                               ArrayList<ArrayList<LCONST>> u) {
                u.forEach( new Consumer< ArrayList<LCONST> >( ) {
                    @Override
                    public void accept(ArrayList<LCONST> terms ) {
                        future_TERMS.forEach( new Consumer<LCONST>() {
                            @Override
                            public void accept(LCONST future_term) {
                                try {
                                    EXPR action_var = new PVAR_EXPR( pvar._sPVarName, terms )
                                            .addTerm(TIME_PREDICATE, constants, objects, hmtypes, hm_variables )
                                            .addTerm(future_PREDICATE, constants, objects, hmtypes, hm_variables )
                                            .substitute( Collections.singletonMap( TIME_PREDICATE, TIME_TERMS.get(0) ), constants, objects, hmtypes, hm_variables )
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
                                    GRBVar grb_var = EXPR.grb_cache.get( action_var );
                                    assert( grb_var != null );
                                    double actual = grb_var.get( GRB.DoubleAttr.X );

                                    //NOTE : uncomment this part if having issues with constrained actions
                                    // such as if you get -1E-11 instead of 0,
                                    //and you are expecting a positive action >= 0
                                    String interm_val = State._df.format( actual );
                                    //System.out.println( actual + " rounded to " + interm_val );

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

        System.out.println( "Maximum (unscaled) bound violation : " +  + grb_model.get( GRB.DoubleAttr.BoundVio	) );
        System.out.println("Sum of (unscaled) constraint violations : " + grb_model.get( GRB.DoubleAttr.ConstrVioSum ) );
        System.out.println("Maximum integrality violation : "+ grb_model.get( GRB.DoubleAttr.IntVio ) );
        System.out.println("Sum of integrality violations : " + grb_model.get( GRB.DoubleAttr.IntVioSum ) );
        System.out.println("Objective value : " + grb_model.get( GRB.DoubleAttr.ObjVal ) );

        return ret;
    }



    protected static void outputNAMEMAPFile(final GRBModel grb_model) throws GRBException, IOException {
        grb_model.write( OUTPUT_FILE );
        ArrayList<String> data = new ArrayList<>();

        for( String key : EXPR.reverse_name_map.keySet()){
            data.add(key +"     ::   " + EXPR.reverse_name_map.get(key));
        }
        FileWriter writer = new FileWriter(OUTPUT_NAMEMAP_FILE);
        for(String str: data) {
            writer.write(str);
            writer.write("\n");
        }
        writer.close();
    }



    protected void outputLPFile(final GRBModel grb_model) throws GRBException, IOException {
        grb_model.write( OUTPUT_FILE );

        List<String> src = new ArrayList<>( EXPR.reverse_name_map.keySet() );
        Collections.sort( src, new Comparator<String>() {

            @Override
            public int compare(  String o1, String o2) {
                return (new Integer(o1.length()).compareTo(
                        new Integer( o2.length()) ) );
            }
        });
        Collections.reverse( src );

        Files.write( Paths.get( OUTPUT_FILE + ".post" ),
                Files.readAllLines( Paths.get( OUTPUT_FILE ) ).stream().map( new Function<String, String>() {

                    @Override
                    public String apply(String t) {
                        String ret = t;
                        for( String entry :  src ){
                            ret = ret.replace( entry, EXPR.reverse_name_map.get( entry ) );
                        }
                        return ret;
                    }

                }).collect( Collectors.toList() ) );
    }

    public void setActionVariables(final ArrayList<PVAR_INST_DEF> action_zero,
                                   final GRBModel static_grb_model){

        if(action_zero.size()==0)
            return;

        Object val = null;
        HashMap<EXPR,Object> action_value = new HashMap<>();
        for(int i = 0; i < action_zero.size(); i++){
            PVAR_INST_DEF act = action_zero.get(i);
            if( act._oValue instanceof Double){
                val = (Double) act._oValue; }
            else if(act._oValue instanceof Boolean){
                val = (Boolean) act._oValue;
                val = val.equals(true) ? 1.0 : 0.0; }
            else{
                val = (Integer) act._oValue;
            }

            for(int j = 0; j < future_TERMS.size(); j++){
                try {
                    EXPR action_var = new PVAR_EXPR(act._sPredName._sPVarName, act._alTerms)
                            .addTerm(TIME_PREDICATE, constants, objects, hmtypes, hm_variables )
                            .addTerm(future_PREDICATE, constants, objects, hmtypes, hm_variables )
                            .substitute(Collections.singletonMap(TIME_PREDICATE, TIME_TERMS.get(0)), constants, objects, hmtypes, hm_variables )
                            .substitute(Collections.singletonMap(future_PREDICATE, future_TERMS.get(j)), constants, objects, hmtypes, hm_variables );
                    action_value.put(action_var, val);
                }
                catch (Exception e){
                    e.printStackTrace();
                }
            }
        }

        if(action_value.size() == 0){
            return;
        }

        HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>> src = new HashMap<>();
        src.putAll(rddl_action_vars);
        src.forEach( new BiConsumer<PVAR_NAME, ArrayList<ArrayList<LCONST>> >() {
            @Override
            public void accept(PVAR_NAME pvar, ArrayList<ArrayList<LCONST>> u) {
                u.stream().forEach( new Consumer<ArrayList<LCONST>>() {
                    @Override
                    public void accept(ArrayList<LCONST> terms) {
                        //System.out.print(pvar.toString());
                        try{
                            future_TERMS.forEach(new Consumer<LCONST>() {
                                @Override
                                public void accept(LCONST future_term) {
                                    try {
                                        //System.out.println("These are the terms" + terms.toString());
                                        EXPR action_var = new PVAR_EXPR(pvar._sPVarName, terms)
                                                .addTerm(TIME_PREDICATE, constants, objects, hmtypes, hm_variables )
                                                .addTerm(future_PREDICATE, constants, objects, hmtypes, hm_variables )
                                                .substitute(Collections.singletonMap(TIME_PREDICATE, TIME_TERMS.get(0)), constants, objects, hmtypes, hm_variables )
                                                .substitute(Collections.singletonMap(future_PREDICATE, future_term), constants, objects, hmtypes, hm_variables );
                                        GRBVar this_var = EXPR.grb_cache.get(action_var);

                                        if(action_value.containsKey(action_var)){
                                            if(SHOW_TIME_ZERO_GUROBI_ACTION){
                                                System.out.println("Setting value Action : " + action_var.toString() +"  value :" +action_value.get(action_var).toString() );
                                            }
                                            this_var.set(GRB.DoubleAttr.Start, (Double) action_value.get(action_var));
                                        }
                                    } catch (GRBException e) {
                                        e.printStackTrace();
                                    } catch (Exception e) {
                                        e.printStackTrace();
                                    }

                                }
                            });
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                    }
                });
            }
        });
    }


    protected void modelSummary(final GRBModel grb_model) throws GRBException {
        System.out.println( "Status : "+ grb_model.get( GRB.IntAttr.Status ) + "(Optimal/Inf/Unb: " + GRB.OPTIMAL + ", " + GRB.INFEASIBLE +", " + GRB.UNBOUNDED + ")" );
        System.out.println( "Number of solutions found : " + grb_model.get( GRB.IntAttr.SolCount ) );
        System.out.println( "Number of simplex iterations performed in most recent optimization : " + grb_model.get( GRB.DoubleAttr.IterCount ) );
        System.out.println( "Number of branch-and-cut nodes explored in most recent optimization : " + grb_model.get( GRB.DoubleAttr.NodeCount ) );

        //System.out.println("Maximum (unscaled) primal constraint error : " + grb_model.get( DoubleAttr.ConstrResidual ) );

//		System.out.println("Sum of (unscaled) primal constraint errors : " + grb_model.get( DoubleAttr.ConstrResidualSum ) );
//		System.out.println("Maximum (unscaled) dual constraint error : " + grb_model.get( DoubleAttr.DualResidual ) ) ;
//		System.out.println("Sum of (unscaled) dual constraint errors : " + grb_model.get( DoubleAttr.DualResidualSum ) );

        System.out.println( "#Variables : "+ grb_model.get( GRB.IntAttr.NumVars ) );
        System.out.println( "#Integer variables : "+ grb_model.get( GRB.IntAttr.NumIntVars ) );
        System.out.println( "#Binary variables : "+ grb_model.get( GRB.IntAttr.NumBinVars ) );
        System.out.println( "#Constraints : "+ grb_model.get( GRB.IntAttr.NumConstrs ) );
        System.out.println( "#NumPWLObjVars : "+ grb_model.get( GRB.IntAttr.NumPWLObjVars ) );

        System.out.println("#State Vars : " + string_state_vars.size() );
        System.out.println("#Action Vars : " + string_action_vars.size() );
        System.out.println("Optimization Runtime(mins) : " + grb_model.get( GRB.DoubleAttr.Runtime ) );
    }



    protected void handleOOM(GRBModel grb_model) throws GRBException {
        System.out.println("JVM free memory : " + Runtime.getRuntime().freeMemory() + " / " +
                Runtime.getRuntime().maxMemory() + " = " + ( ((double)Runtime.getRuntime().freeMemory()) / Runtime.getRuntime().maxMemory()) );
        System.out.println("round end / out of memory detected; trying cleanup");

        cleanUp(grb_model);
        grb_model.getEnv().dispose();
        grb_model.dispose();

        RDDL.EXPR.cleanUpGRB();
        System.gc();
        firstTimeModel( );
    }




    protected Map<LVAR,LCONST> getSubs(ArrayList<RDDL.LTERM> terms, ArrayList<LCONST> consts) {
        Map<LVAR, LCONST> ret = new HashMap<RDDL.LVAR, RDDL.LCONST>();
        for( int i = 0 ; i < terms.size(); ++i ){
            assert( terms.get(i) instanceof LVAR );
            ret.put( (LVAR)terms.get(i), consts.get(i) );
        }
        return ret;
    }



    protected void cleanUp(final GRBModel grb_model) throws GRBException {
//		saved_expr.clear(); saved_vars.clear();

//		RDDL.EXPR.cleanUpGRB();
        for( final GRBConstr constr : to_remove_constr ){
            if( !saved_constr.contains(constr) ){
//				System.out.println(constr.toString());
                try{
//					System.out.println("Removing constraint " + );
                    constr.get(StringAttr.ConstrName);
                    //get can throw an exception

                }catch(GRBException exc){
                    System.out.println(exc.getErrorCode());
                    exc.printStackTrace();
                    //////System.exit(1);
                }
                grb_model.remove( constr );
                grb_model.update();
                //System.out.println(grb_model.get( IntAttr.NumConstrs ));
                //System.out.println(grb_model.toString());

            }
        }

        grb_model.get( GRB.IntAttr.NumConstrs );
        to_remove_constr.clear();

        to_remove_expr.clear();
    }


    protected void addRootPolicyConstraints(final GRBModel grb_model) throws Exception{
        GRBExpr old_obj = grb_model.getObjective();

        getHindSightConstraintExpr(hindsight_method).stream().forEach( new Consumer<RDDL.BOOL_EXPR>() {

            @Override
            public void accept( RDDL.BOOL_EXPR t) {
                synchronized( grb_model ){
                    try {
                        GRBVar gvar = t.getGRBConstr( GRB.EQUAL, grb_model, constants, objects, type_map, hmtypes, hm_variables);

                        GRBConstr this_constr = grb_model.addConstr( gvar, GRB.EQUAL, 1, RDDL.EXPR.getGRBName(t) );
                        saved_expr.add( t ); // saved_vars.add( gvar );
                        saved_constr.add(this_constr);
                    } catch (GRBException e) {
                        e.printStackTrace();
                        //System.exit(1);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        });

        grb_model.setObjective(old_obj);
        grb_model.update();
    }


    public double runNooopPolicyForState(State s)throws Exception{
        //Getting Random Trajectories and adding to buffer.
        int num_trajectories = 30;
        int length_trajectories = 10;

        ArrayList[] stats = runNoopPolicy(s, length_trajectories,
                num_trajectories);
        ArrayList<ArrayList<HashMap<PVAR_NAME,HashMap<ArrayList<LCONST>,Object>>>> buffer_state = stats[0];
        ArrayList<ArrayList<ArrayList<PVAR_INST_DEF>>> buffer_action = stats[1];
        ArrayList<ArrayList<Double>> buffer_reward = stats[2];

        return buffer_reward.stream().mapToDouble(
                m -> m.stream().mapToDouble(n -> n).sum()).sum() / ((double)buffer_reward.size());

    }

    protected ArrayList[] runNoopPolicy(final State rddl_state,
                                        int trajectory_length, int number_trajectories) throws EvalException {

        ArrayList<PVAR_INST_DEF> noop_action = new ArrayList<>();

        HashMap<PVAR_NAME, HashMap<ArrayList<LCONST>, Object>>
                traj_inital_state = State.deepCopyState(rddl_state);

        final ArrayList<ArrayList<HashMap<PVAR_NAME,HashMap<ArrayList<LCONST>,Object>>>> buffer_state = new ArrayList<>();
        final ArrayList<ArrayList<ArrayList<PVAR_INST_DEF>>> buffer_action = new ArrayList<>();
        final ArrayList<ArrayList<Double>> buffer_reward = new ArrayList<>();

        for (int j = 0; j < number_trajectories; j++) {
            rddl_state.copyStateRDDLState(traj_inital_state, true);

            ArrayList<HashMap<PVAR_NAME, HashMap<ArrayList<LCONST>, Object>>>
                    store_traj_states = new ArrayList<>();
            ArrayList<Double> store_traj_rewards = new ArrayList<>();
            ArrayList<ArrayList<PVAR_INST_DEF>> store_traj_actions = new ArrayList<>();

            for (int i = 0; i < trajectory_length; i++) {
                ArrayList<PVAR_INST_DEF> traj_action = noop_action;
                HashMap<PVAR_NAME, HashMap<ArrayList<LCONST>, Object>>
                        store_state = State.deepCopyState(rddl_state);
                store_traj_states.add(store_state);
                //Advance to Next State
                rddl_state.computeNextState(traj_action, this._random);
                //Calculate Immediate Reward
                final double immediate_reward =
                        Math.pow(rddl_instance._dDiscount, i) *
                                ((Number) rddl_domain._exprReward.sample(
                                        new HashMap<LVAR, LCONST>(), rddl_state, this._random)).doubleValue();
                store_traj_rewards.add(immediate_reward);
                store_traj_actions.add(traj_action);
                //System.out.println("Immediate Reward :"+ immediate_reward);
                rddl_state.advanceNextState();
            }

            buffer_state.add(store_traj_states);
            buffer_action.add(store_traj_actions);
            buffer_reward.add(store_traj_rewards);
        }
        rddl_state.copyStateRDDLState(traj_inital_state, true);
        return new ArrayList[]{buffer_state, buffer_action, buffer_reward};
    }


    public double runRandomPolicyForState(State s)throws Exception{

        //Getting Random Trajectories and adding to buffer.
        int num_trajectories = 30;
        int length_trajectories = 10;

        ArrayList[] stats = runRandomPolicy(s, length_trajectories,
                num_trajectories);
        ArrayList<ArrayList<HashMap<PVAR_NAME,HashMap<ArrayList<LCONST>,Object>>>> buffer_state = stats[0];
        ArrayList<ArrayList<ArrayList<PVAR_INST_DEF>>> buffer_action = stats[1];
        ArrayList<ArrayList<Double>> buffer_reward = stats[2];

        return buffer_reward.stream().mapToDouble(
                m -> m.stream().mapToDouble(n -> n).sum()).sum() / ((double)buffer_reward.size());

        //We are updating current as previous.
//        pre_buffer_action = buffer_action;
//        pre_buffer_state  = buffer_state;
//        pre_buffer_reward = buffer_reward;
        //This is for random policy immediate average reward.
    }


    public void convertNPWLtoPWL(State s) throws Exception {
        long startTime1 = System.currentTimeMillis();
        checkNonLinearExpressions(s);
        long endTime2 = System.currentTimeMillis();
        double pwl_timer_value =(double) (endTime2-startTime1)/1000;
        double r_api_timer = running_R_api/1000;
    }


    public Pair<Boolean,Pair<Integer,Integer>> CompetitionExplorationPhase(
            String rddl_filepath, String instanceName, String gurobi_timeout,
            String future_gen_type,String hindsight_strat, String rand_seed,
            RDDL rddl_object, State s, String total_explo_time ) throws Exception{


        ////////////////////////////////////////////////////////////////////////////
        final double start_time = Double.valueOf(System.currentTimeMillis());
        RDDL rddl = null;
        RDDL.DOMAIN domain = null;
        RDDL.INSTANCE instance = null;
        State state = null;
        RDDL.NONFLUENTS nonFluents = null;
        ////////////////////////////////////////////////////////////////////////////

        int exp_steps = 8;
        int exp_rounds = 10;
        HashMap<Pair<Integer,Integer>,Double> exploration_rewards = new HashMap<>();
        int START_LOOKAHEAD_VALUE = 4;
        int START_FUTURE_VALUE = 4;
        int current_lookAhead = START_LOOKAHEAD_VALUE;
        ////////////////////////////////////////////////////////////////////////////

        boolean EXPLORATION = true;
        int best_lookahead = Integer.MAX_VALUE;
        int best_future    = Integer.MAX_VALUE;
        boolean NPWL_OR_NOT = true;
        //This one is for lookahead value.
        while(current_lookAhead < 40){
            //This is the starting value of future.
            int current_future = START_FUTURE_VALUE;
            while(current_future < 100) {
                ////////////////////////////////////////////////////////////////////////////
                //This is place for setting up RDDL Object.
                rddl = new RDDL(rddl_filepath);
                state = new State();
                instance = rddl._tmInstanceNodes.get(instanceName);
                if (instance._sNonFluents != null) {
                    nonFluents = rddl._tmNonFluentNodes.get(instance._sNonFluents);
                }
                domain = rddl._tmDomainNodes.get(instance._sDomain);

                state.init(domain._hmObjects, nonFluents != null ? nonFluents._hmObjects : null, instance._hmObjects,
                        domain._hmTypes, domain._hmPVariables, domain._hmCPF,
                        instance._alInitState, nonFluents == null ? new ArrayList<PVAR_INST_DEF>() : nonFluents._alNonFluents, instance._alNonFluents,
                        domain._alStateConstraints, domain._alActionPreconditions, domain._alStateInvariants,
                        domain._exprReward, instance._nNonDefActions);

                // If necessary, correct the partially observed flag since this flag determines what content will be seen by the Client
                if ((domain._bPartiallyObserved && state._alObservNames.size() == 0)
                        || (!domain._bPartiallyObserved && state._alObservNames.size() > 0)) {
                    boolean observations_present = (state._alObservNames.size() > 0);
                    System.err.println("WARNING: Domain '" + domain._sDomainName
                            + "' partially observed (PO) flag and presence of observations mismatched.\nSetting PO flag = " + observations_present + ".");
                    domain._bPartiallyObserved = observations_present;
                }

                ///################################################################################################################


                HOPPlanner explo_planner  = new HOPPlanner( String.valueOf(current_future),
                        String.valueOf(current_lookAhead),  instanceName,  gurobi_timeout,
                        future_gen_type, hindsight_strat, rand_seed , rddl, state);

                explo_planner.setRDDL(rddl);
                Double average_round_reward = 0.0;
                ////////////////////////////////////////////////////////////////////////////
                //Number of rounds start here.
                boolean found_round_best_action = false;
                for (int j = 0; j < exp_rounds; j++) {
                    //Each round initialize the state
                    state.init(domain._hmObjects, nonFluents != null ? nonFluents._hmObjects : null, instance._hmObjects,
                            domain._hmTypes, domain._hmPVariables, domain._hmCPF,
                            instance._alInitState, nonFluents == null ? new ArrayList<PVAR_INST_DEF>() : nonFluents._alNonFluents, instance._alNonFluents,
                            domain._alStateConstraints, domain._alActionPreconditions, domain._alStateInvariants,
                            domain._exprReward, instance._nNonDefActions);

                    double round_reward = 0.0;
                    ArrayList<PVAR_INST_DEF> round_best_action = new ArrayList<>();
                    double max_step_reward = -Double.MAX_VALUE;
                    //This is for sequential Steps..
                    ArrayList<ArrayList<Object>> checking = new ArrayList<>();
                    for (int n = 0; n < exp_steps; n++) {
                        try {

                            //This is to check if there are any NPWL expression.  If there are no,then DO_NWPL_PWL will be false.
                            //explo_planner.DO_NPWL_PWL = true;
                            //This value changes when we don't find any expression which is NPWL.
                            NPWL_OR_NOT =  explo_planner.DO_NPWL_PWL;
                            if (explo_planner.DO_NPWL_PWL) {
                                explo_planner.checkNonLinearExpressions(state);
                            }
                            ArrayList<Object> temp_s_a_r = new ArrayList<>();
                            explo_planner.TIME_LIMIT_MINS = Double.valueOf(gurobi_timeout);
                            explo_planner.DO_GUROBI_INITIALIZATION = true;
                            //System.out.println(">>>>>>>>>>This is State ::: ");
                            //System.out.println(state._state.toString());
                            ArrayList<PVAR_INST_DEF> actions = new ArrayList<>();

                            try {
                                if(found_round_best_action)
                                    explo_planner.setActionVariables(round_best_action,
                                            explo_planner.static_grb_model);
                                actions = explo_planner.getActions(state);
                                System.out.println("The Action Taken is >>" + actions.toString());
                                state.checkStateActionConstraints(actions);
                            }catch(Exception e){
                                // This is the case when actions are infeasible or gurobi has infeasiblity.
                                e.printStackTrace();
                                actions = baseLineAction(rddl_state);
                            }

                            state.computeNextState(actions, this._random);

                            final double immediate_reward = ((Number) domain._exprReward.sample(
                                    new HashMap<RDDL.LVAR, LCONST>(), state, this._random)).doubleValue();
                            temp_s_a_r.add(state._state); temp_s_a_r.add(actions) ; temp_s_a_r.add(immediate_reward);
                            state.advanceNextState();
                            round_reward += immediate_reward;
                            if (immediate_reward > max_step_reward) {
                                round_best_action = actions;
                                max_step_reward = immediate_reward;
                            }
                            checking.add(temp_s_a_r);
                        }catch (Exception e){
                            e.printStackTrace();
                        }
                    }
                    if(round_best_action!=null){
                        found_round_best_action = true;
                    }
                    average_round_reward += round_reward;
                }

                Double cur_end_time = Double.valueOf(System.currentTimeMillis());

                exploration_rewards.put(new Pair<>(current_lookAhead, current_future), average_round_reward / exp_rounds);
                explo_planner.dispose_Gurobi();
                current_future = getFutureValue(current_future, true); //Double.valueOf(Math.pow(FUTURE_VALUE,2)).intValue();
                if(cur_end_time-start_time>Double.valueOf(total_explo_time)){
                    break;
                }
            }
            current_lookAhead = getLookAheadValue(current_lookAhead, true);

            Double cur_end_time = Double.valueOf(System.currentTimeMillis());
            if(cur_end_time-start_time>Double.valueOf(total_explo_time)){
                break;
            }
        }
        Double max_reward = -Double.MAX_VALUE;

        //This piece of code is to get the best lookahead and future pair.
        HashMap<Pair<Integer,Integer>,Double> equal_rewards = new HashMap<>();
        for(Pair<Integer,Integer> key : exploration_rewards.keySet()){
            if(exploration_rewards.get(key)>=max_reward){
                if(exploration_rewards.get(key).equals(max_reward)){ equal_rewards.put(key,max_reward); }
                else{ max_reward = exploration_rewards.get(key);if(equal_rewards!=null){equal_rewards.clear();}
                    equal_rewards.put(key,max_reward); }
            }
        }
        for(Pair<Integer,Integer> key : equal_rewards.keySet()){
            if(key._o1 < best_lookahead){
                best_lookahead = key._o1;
                best_future    = key._o2;
            }
        }

        //these are not returned?
        Pair<Integer,Integer> best_parameters =new Pair<Integer, Integer>(best_lookahead,best_future);
        Pair<Boolean,Pair<Integer,Integer>> return_value = new Pair(NPWL_OR_NOT,best_parameters);

        return return_value;
    }


    public ArrayList<PVAR_INST_DEF> baseLineAction(State s) {

        ArrayList<PVAR_INST_DEF> noop_action      = new ArrayList<>();
        ArrayList<PVAR_INST_DEF> returning_action = new ArrayList<>();
        try {
            final double avg_reward_random = runRandomPolicyForState(s);
            final double avg_reward_noop   = runNooopPolicyForState(s);
            if(avg_reward_random > avg_reward_noop){
                returning_action = this.random_policy.getActions(s);
                if(SHOW_LEVEL_1)
                    System.out.println("Choosing Random Action");
            }
            else {
                returning_action = noop_action;
            }
        } catch(Exception e){
            returning_action = noop_action;
        }

        return returning_action;
    }


    static Integer getLookAheadValue(Integer current_lookahead, Boolean change){
        Integer next_look_ahead = current_lookahead;
        if(change){
            if(current_lookahead==1){
                next_look_ahead = 2;
            }
            else {
                //next_look_ahead = Double.valueOf(Math.pow(current_lookahead,2)).intValue() ;
                next_look_ahead = current_lookahead * 2 ;}
        }
        else{
            next_look_ahead = current_lookahead;
        }

        return next_look_ahead;
    }



    static Integer getFutureValue(Integer current_future, Boolean change){
        Integer next_future_value = current_future;
        if(change){
            if(current_future==1){
                next_future_value = 4;
            }
            else {
                //next_look_ahead = Double.valueOf(Math.pow(current_lookahead,2)).intValue() ;
                next_future_value = current_future * 2 ;}
        }
        else{
            next_future_value = current_future;
        }
        return next_future_value;
    }




    protected ArrayList[] runRandomPolicy(final State rddl_state,
                                          int trajectory_length, int number_trajectories) throws EvalException {


        HashMap<PVAR_NAME, HashMap<ArrayList<LCONST>, Object>>
                traj_inital_state = State.deepCopyState(rddl_state);

        final ArrayList<ArrayList<HashMap<PVAR_NAME,HashMap<ArrayList<LCONST>,Object>>>> buffer_state = new ArrayList<>();
        final ArrayList<ArrayList<ArrayList<PVAR_INST_DEF>>> buffer_action = new ArrayList<>();
        final ArrayList<ArrayList<Double>> buffer_reward = new ArrayList<>();

        for (int j = 0; j < number_trajectories; j++) {
            //Deep copies traj_inital_state to rddl_state._state.
            rddl_state.copyStateRDDLState(traj_inital_state, true);

            ArrayList<HashMap<PVAR_NAME, HashMap<ArrayList<LCONST>, Object>>>
                    store_traj_states = new ArrayList<>();
            ArrayList<Double> store_traj_rewards = new ArrayList<>();
            ArrayList<ArrayList<PVAR_INST_DEF>> store_traj_actions = new ArrayList<>();

            for (int i = 0; i < trajectory_length; i++) {
                ArrayList<PVAR_INST_DEF> traj_action
                        = this.random_policy.getActions(rddl_state);
                HashMap<PVAR_NAME, HashMap<ArrayList<LCONST>, Object>>
                        store_state = State.deepCopyState(rddl_state);
                store_traj_states.add(store_state);
                //Advance to Next State
                rddl_state.computeNextState(traj_action, this._random);
                //Calculate Immediate Reward
                final double immediate_reward =
                        Math.pow(rddl_instance._dDiscount, i) *
                                ((Number) rddl_domain._exprReward.sample(
                                        new HashMap<LVAR, LCONST>(), rddl_state, this._random)).doubleValue();
                store_traj_rewards.add(immediate_reward);
                store_traj_actions.add(traj_action);
                //System.out.println("Immediate Reward :"+ immediate_reward);
                rddl_state.advanceNextState();
            }

            buffer_state.add(store_traj_states);
            buffer_action.add(store_traj_actions);
            buffer_reward.add(store_traj_rewards);
        }
        rddl_state.copyStateRDDLState(traj_inital_state, true);
        return new ArrayList[]{buffer_state, buffer_action, buffer_reward};
    }

    public void checkNonLinearExpressions(final State rddl_state) throws Exception {

        ArrayList[] buffers = null;
        //PASS BUFFERS TO FITTING

        //Reward
        final Map<LVAR,LCONST> subs2 = new HashMap<>();

        EXPR sub_expr = null;
        Pair<String,String> key_check = new Pair<>(rddl_state._reward.toString(),subs2.toString());
        if( substitute_expression_cache.containsKey(key_check) ){
            sub_expr=substitute_expression_cache.get(key_check);
        }else{
            sub_expr = rddl_state._reward.substitute(subs2, constants, objects,
                    hmtypes, hm_variables );
        }

        final boolean check_pwl = sub_expr.isPiecewiseLinear(constants, objects,
                hmtypes, hm_variables);
        ArrayList<RDDL.LTERM> raw_terms1 = new ArrayList<>();
        if(!check_pwl){

            if(buffers==null)
                buffers = runRandomPolicy(rddl_state, 2, 10000);
            //EXPR e, ArrayList<RDDL.LTERM> raw_terms, State s, ArrayList[] buffers, RandomDataGenerator random
            EXPR final_expr   = earth_obj.fitPWL(null,null, true, sub_expr, rddl_state, buffers,type_map, hm_variables,hmtypes,this._random);
            //STORE THIS IS replace_reward_pwl
            replace_reward_pwl = final_expr;
        }

        ArrayList<HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>>> pvar_variables = new ArrayList<>();
        pvar_variables.add(rddl_state_vars); pvar_variables.add(rddl_interm_vars); pvar_variables.add(rddl_observ_vars);
        for( final HashMap<PVAR_NAME, ArrayList<ArrayList<LCONST>>> map : pvar_variables ) {
            for (final PVAR_NAME p : map.keySet()) {

                CPF_DEF cpf = null;
                if (rddl_state_vars.containsKey(p)) {
                    cpf = rddl_state._hmCPFs.get(new PVAR_NAME(p._sPVarName + "'"));

                } else {
                    cpf = rddl_state._hmCPFs.get(new PVAR_NAME(p._sPVarName));
                }

                ArrayList<RDDL.LTERM> raw_terms = cpf._exprVarName._alTerms;
                ArrayList<EXPR> final_pwl_cond = new ArrayList<>();
                boolean pvar_npwl = false;

                //This loop is for $t1, $t2, $t3..........
                for (final ArrayList<LCONST> terms : map.get(p)) {
                    Map<LVAR, LCONST> subs1 = getSubs(cpf._exprVarName._alTerms, terms);
                    try {
                        if (SHOW_PWL_NON_PWL)
                            System.out.println(cpf._exprEquals.toString());
                        EXPR subs_expr = null;
                        Pair<String,String> key_check_expr = new Pair<>(cpf._exprEquals.toString(),subs1.toString());
                        if( substitute_expression_cache.containsKey(key_check_expr)){
                            subs_expr = substitute_expression_cache.get(key_check_expr);
                        }else{
                            subs_expr = cpf._exprEquals.substitute(subs1, constants, objects, hmtypes, hm_variables);
                        }

                        EXPR det_expr = subs_expr.sampleDeterminization(_random, constants, objects, hmtypes, hm_variables);

                        final boolean check_PWL = det_expr.isPiecewiseLinear(constants, objects, hmtypes, hm_variables);

                        if (SHOW_PWL_NON_PWL)
                            System.out.println("Substituted PWL: " + check_PWL);
                        if (!check_PWL) {

                            if(buffers==null)
                                buffers = runRandomPolicy(rddl_state, 2, 10000);
                            pvar_npwl = true;
                            EXPR final_expr = earth_obj.fitPWL(cpf._exprVarName._pName,terms,false,det_expr,rddl_state, buffers,type_map, hm_variables,hmtypes,this._random);
                            final_pwl_cond.add(final_expr);
                        } else {
                            final_pwl_cond.add( det_expr );
                        }
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                }

                if (pvar_npwl) {
                    replace_cpf_pwl.put(p, final_pwl_cond);
                }

            }
        }

    }




    protected Object sanitize(PVAR_NAME pName, double value) {
        if( value == -1*value ){
            value = Math.abs( value );
        }

        Object ret = null;
        if( type_map.get( pName ).equals( GRB.BINARY ) ){
            if( value > 1.0 ){
                value = 1;
            }else if( value < 0.0 ){
                value = 0;
            }else{
                value = Math.rint( value );
            }
            assert( value == 0d || value == 1d );
            ret = new Boolean( value == 0d ? false : true );
        }
        else if( type_map.get( pName ).equals( GRB.INTEGER ) ){
            value = Math.rint( value );
            ret = new Integer( (int)value );
        } else{
            ret = new Double( value );
        }
        return ret;
    }


    public void dispose_Gurobi() throws GRBException {
        System.out.println("Cleaning Gurobi");
        cleanUp(static_grb_model);
        static_grb_model.getEnv().dispose();
        static_grb_model.dispose();
        RDDL.EXPR.cleanUpGRB();
        System.gc();
    }


}