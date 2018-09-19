package rddl.det.comMipNew;
import gurobi.GRB;
import gurobi.GRBEnv;
import gurobi.GRBModel;
import rddl.State;
import rddl.det.comMipNew.EarthForPlanner;
import rddl.RDDL;
import util.Pair;

import java.util.*;
import gurobi.*;


import static rddl.RDDL.EXPR.M;
import static rddl.RDDL.EXPR.grb_cache;
import static rddl.RDDL.EXPR.name_map;

public class TestCaseEarth {
    protected static Map<RDDL.PVAR_NAME, Map<ArrayList<RDDL.LCONST>, Object>> constants = new HashMap<>();
    protected static HashMap<RDDL.TYPE_NAME, RDDL.OBJECTS_DEF> objects;

    protected static GRBEnv grb_env;
    protected static GRBModel static_grb_model = null;



    protected static void initalizeGurobi() throws Exception{

        grb_env = new GRBEnv();


        grb_env.set( GRB.DoubleParam.TimeLimit, 0.3*60 );
        grb_env.set( GRB.DoubleParam.MIPGap, 0.01 );
        grb_env.set( GRB.DoubleParam.Heuristics, 0.2 );
        grb_env.set( GRB.IntParam.InfUnbdInfo , 1 );
        grb_env.set( GRB.IntParam.DualReductions, 0 );
        grb_env.set( GRB.IntParam.IISMethod, 1 );
        grb_env.set( GRB.IntParam.NumericFocus, 3);
        grb_env.set( GRB.IntParam.MIPFocus, 1);
        grb_env.set( GRB.DoubleParam.FeasibilityTol, 1e-3 );// Math.pow(10,  -(State._df.getMaximumFractionDigits() ) ) ); 1e-6
        grb_env.set( GRB.DoubleParam.IntFeasTol,  1e-9);//Math.pow(10,  -(State._df.getMaximumFractionDigits() ) ) ); //Math.pow( 10 , -(1+State._df.getMaximumFractionDigits() ) ) );
        grb_env.set( GRB.DoubleParam.FeasRelaxBigM, M);
        grb_env.set( GRB.IntParam.Threads, 1 );
        grb_env.set( GRB.IntParam.Quad, 1 );
        grb_env.set( GRB.IntParam.Method, 1 );
        grb_env.set( GRB.DoubleParam.NodefileStart, 0.5 );
        //grb_env.set(GRB.IntParam.Presolve,0);
        //grb_env.set(GRB.IntParam.Presolve,0);
        //grb_env.set(DoubleParam.OptimalityTol, 1e-2);
        grb_env.set(GRB.IntParam.NumericFocus, 3);
//		grb_env.set( IntParam.SolutionLimit, 5);
        static_grb_model = new GRBModel( grb_env );


        static_grb_model.set( GRB.IntAttr.ModelSense, -1);
        static_grb_model.update();
        System.out.println("current nodefile directly " + grb_env.get( GRB.StringParam.NodefileDir ) );
    }





    //This is the test case for titanic Example.  Check R Program for more Details.
    public static void testCaseTitanicExample()throws Exception{

        initalizeGurobi();
        String target      = "pclass";
        String target_type = "enum";

        String feat1 = "survived";
        String feat2 = "sex";
        String feat3 = "age";
        String feat4 = "sibp";
        String feat5 = "parch";
        String feat1_type = "bool";
        String feat2_type = "enum";
        String feat3_type = "real";
        String feat4_type = "int";
        String feat5_type = "int";
        String [] all_features  = new String[]{feat1,feat2,feat3,feat4,feat5};
        String [] all_feat_type = new String[]{feat1_type,feat2_type,feat3_type,feat4_type,feat5_type};
        HashMap<String,Character> mapping_type = new HashMap<>();
        mapping_type.put("bool",'B');  mapping_type.put("int",'I');  mapping_type.put("real",'C');  mapping_type.put("enum",'I');



        ArrayList<RDDL.PVAR_NAME> array_pvars = new ArrayList<>();
        HashMap<RDDL.PVAR_NAME,Character> type_map = new HashMap<>();

        int count=0;
        for(String feat_name : all_features){
            RDDL.PVAR_NAME pName =new RDDL.PVAR_NAME(feat_name);
            array_pvars.add(pName);
            type_map.put(pName,mapping_type.get(all_feat_type[count]));
        }
        //This is for target.
        RDDL.PVAR_NAME targetPvar = new RDDL.PVAR_NAME(target);
        type_map.put(targetPvar,'I');


        HashMap<RDDL.PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes = new HashMap<>();


        //This is for sex.
        RDDL.TYPE_NAME sex = new RDDL.TYPE_NAME("sex");
        ArrayList<RDDL.LCONST> sex_values = new ArrayList<>();
        sex_values.add(new RDDL.ENUM_VAL("@Male"));
        sex_values.add(new RDDL.ENUM_VAL("@Female"));
        RDDL.ENUM_TYPE_DEF sex_tdef = new RDDL.ENUM_TYPE_DEF("sex",sex_values);
        hmtypes.put(sex,sex_tdef);

        //
        RDDL.TYPE_NAME pclass = new RDDL.TYPE_NAME("class");
        ArrayList<RDDL.LCONST> pclass_values = new ArrayList<>();
        pclass_values.add(new RDDL.ENUM_VAL("@First"));
        pclass_values.add(new RDDL.ENUM_VAL("@Second"));
        pclass_values.add(new RDDL.ENUM_VAL("@Third"));

        RDDL.ENUM_TYPE_DEF pclass_tdef = new RDDL.ENUM_TYPE_DEF("class",pclass_values);
        hmtypes.put(pclass,pclass_tdef);



        //This is for hm_variables.
        RDDL.PVARIABLE_STATE_DEF survived_psd = new RDDL.PVARIABLE_STATE_DEF("survived",false,"bool",new ArrayList(),false);
        RDDL.PVARIABLE_STATE_DEF sex_psd = new RDDL.PVARIABLE_STATE_DEF("sex",false,"sex",new ArrayList(),new RDDL.ENUM_VAL("@Male"));
        RDDL.PVARIABLE_STATE_DEF age_psd = new RDDL.PVARIABLE_STATE_DEF("age",false,"real",new ArrayList(),0.0);
        RDDL.PVARIABLE_STATE_DEF sibp_psd = new RDDL.PVARIABLE_STATE_DEF("sibp",false,"int",new ArrayList(),0);
        RDDL.PVARIABLE_STATE_DEF parch_psd = new RDDL.PVARIABLE_STATE_DEF("parch",false,"int",new ArrayList(),0);
        RDDL.PVARIABLE_STATE_DEF pclass_psd = new RDDL.PVARIABLE_STATE_DEF("pclass",false,"class",new ArrayList(),new RDDL.ENUM_VAL("@Third"));


        hm_variables.put(array_pvars.get(0),survived_psd);
        hm_variables.put(array_pvars.get(1),sex_psd);
        hm_variables.put(array_pvars.get(2),age_psd);
        hm_variables.put(array_pvars.get(3),sibp_psd);
        hm_variables.put(array_pvars.get(4),parch_psd);
        hm_variables.put(targetPvar,pclass_psd);


        //Variables

        String target_string="c(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2)";
        String surv_string = "c(1,1,0,0,0,1,1,0,1,0,0,1,1,1,1,0,1,1,0,1,1,1,1,1,0,1,1,1,1,0,1,1,1,0,1,1,0,0,1,1,1,1,0,1,1,1,1,0,0,0,1,1,1,1,0,0,1,0,1,1,1,1,1,1,0,1,1,0,1,0,1,1,0,1,1,0,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,0,1,1,1,0,1,0,1,1,1,0,0,1,1,1,1,1,1,1,0,1,0,1,1,1,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,1,0,1,1,0,1,0,0,1,1,1,0,1,1,1,1,1,0,1,0,0,0,0,0,1,1,1,1,1,1,0,1,1,1,0,1,0,1,1,0,1,0,1,1,0,0,1,0,0,0,1,1,1,0,0,0,1,1,0,1,0,1,1,0,0,0,0,0,1,0,1,1,1,0,1,0,0,1,0,1,1,0,0,1,0,1,0,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,0,0,0,1,1,1,1,1,1,0,1,0,1,1,1,1,0,0,0,1,1,0,1,1,0,1,1,1,0,0,0,1,0,1,0,0,0,1,1,0,1,0,0,1,1,0,1,1,0,1,0,1,0,0,0,0,1,0,0,0,1,0,0,1,1,0,1,1,1,1,1,1,0,0,0,0,1,1,0,1,1,0,1,0,0,1,1,1,1,1,0,0,0,0,0,0,1,1,0,1,0,0,1,1,0,1,1,0,0,1,0,1,1,0,0,0,1,0,0,1,1,0,1,0,1,1,1,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,1,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,0,0,1,0,0,1,0,1,0,0,0,1,0,1,0,0,0,0,1,0,1,0,0,1,0,0,0,0,1,1,0,1,1,1,0,0,0,0,1,0,1,0,1,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,1,1,0,1,0,1,0,1,0,0,1,1,0,1,0,1,0,1,1,1,0,0,1,1,0,1,1,1,1,0,1,0,0,0,1,1,1,1,0,1,0,1,0,0,0,0,0,1,0,1,1,0,0,0,1,0,0,1,1,1,1,0,1,1,1,1,1,1,0,1,0,1,1,0,0,0,0,1,1,1,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1,1,1,0,0,0,0,0,1,0,0,1,1,1,1,1,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1,1,0,0,1,0,0,0,1,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,0,1,1,0,1,0,0,1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,1,0,1,0,1,0,0,1,1,0,0,1,0,0,0,0,1,1,0,0,0,0,1,0,0,0,0,1,1,0,0,0,1,0,1,0,0,0,1,0,0,0,1,0,0,1,1,0,0,0,0,0,1,1,1,0,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,1,1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,1,0,1,0,0,1,1,1,1,0,1,1,0,0,1,1,0,0,1,0,0,1,0,0,1,1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,1,1,1,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,1,1,0,0,1,0,1,0,1,1,0,0,0,1,1,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0)";
        String sex_string  = "c(1,0,1,0,1,0,1,0,1,0,0,1,1,1,0,0,1,1,0,0,1,0,1,1,0,0,1,1,0,0,0,1,1,0,1,1,0,0,1,1,1,1,0,0,1,0,1,0,0,0,0,1,0,1,0,0,1,0,1,0,1,1,1,0,0,1,1,0,1,0,1,1,0,1,1,0,1,0,0,1,0,1,0,1,0,0,1,0,1,1,1,0,0,1,1,1,1,0,0,1,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,1,1,0,0,0,0,1,0,1,0,1,0,0,1,0,1,0,1,0,0,1,0,1,0,0,1,1,1,0,1,0,0,1,1,1,0,0,0,0,0,0,0,1,1,1,1,0,0,1,1,1,0,1,0,1,1,0,1,0,1,1,0,0,0,0,0,0,1,1,0,0,0,0,1,1,0,1,0,1,1,0,0,0,0,0,1,0,1,1,0,0,1,0,0,1,0,0,1,0,0,1,0,1,0,0,1,1,0,1,1,1,1,0,1,0,0,1,0,0,0,0,0,1,0,1,0,0,1,0,0,0,0,1,1,1,0,1,0,1,1,0,1,0,0,0,1,0,0,0,0,1,0,1,0,0,0,1,1,0,1,0,0,1,1,0,0,1,0,1,0,1,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,1,1,1,0,1,0,0,0,0,1,1,0,1,1,0,1,0,0,1,0,0,1,1,0,1,0,0,0,1,1,1,0,1,0,0,0,1,0,1,1,1,0,0,0,1,1,0,0,0,1,0,0,1,1,0,0,0,1,1,1,0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,1,1,1,0,1,1,0,0,0,1,1,0,0,1,0,0,1,0,0,0,1,0,1,0,1,0,0,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,1,1,0,0,0,0,1,1,0,0,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,1,1,0,1,0,0,0,1,1,1,0,0,1,1,0,0,0,1,1,0,1,0,0,0,1,1,1,1,0,1,0,1,0,0,0,0,0,1,0,1,1,0,1,0,1,0,0,1,1,1,1,0,1,0,1,1,1,1,0,1,0,0,1,1,0,0,0,1,1,0,0,1,0,0,1,0,1,0,0,0,0,0,0,0,0,1,0,1,1,1,1,1,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,1,0,0,1,1,0,0,1,0,0,1,0,0,1,1,1,1,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1,1,0,0,0,0,1,0,0,1,1,1,0,0,0,0,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,1,1,1,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,0,0,1,0,0,1,0,0,0,1,0,0,0,0,1,1,0,0,1,0,0,0,0,0,0,1,1,0,1,0,0,0,0,0,1,0,0,0,1,1,1,1,0,1,1,1,0,0,1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,1,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,1,1,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,1,0,0,0,1,0,0,0,1,0,0,0,0,1,0,0,1,1,0,1,0,0,0,0,0,0,0,1,0,1,0,0,1,1,0,1,0,0,1,1,0,1,1,0,0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,1,1,0,0,1,0,0,1,1,1,0,0,0,0,0,1,0,0,0,0,1,1,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,1,0,1,0,0,0,0,0,0,1,0,1,1,1,0,0,0,0,0,0,0,1,0,0,1,1,0,1,0,0,0,0,1,0,0,0,1,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,1,0,1,1,0,0,1,0,1,0,0,0,0,1,0,0,1,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0)";
        String age_string = "c(29,0.916700006,2,30,25,48,63,39,53,71,47,18,24,26,80,24,50,32,36,37,47,26,42,29,25,25,19,35,28,45,40,30,58,42,45,22,41,48,44,59,60,41,45,42,53,36,58,33,28,17,11,14,36,36,49,36,76,46,47,27,33,36,30,45,27,26,22,47,39,37,64,55,70,36,64,39,38,51,27,33,31,27,31,17,53,4,54,50,27,48,48,49,39,23,38,54,36,36,30,24,28,23,19,64,60,30,50,43,22,60,48,37,35,47,35,22,45,24,49,71,53,19,38,58,23,45,46,25,25,48,49,45,35,40,27,24,55,52,42,55,16,44,51,42,35,35,38,35,38,50,49,46,50,32.5,58,41,42,45,39,49,30,35,42,55,16,51,29,21,30,58,15,30,16,19,18,24,46,54,36,28,65,44,33,37,30,55,47,37,31,23,58,19,64,39,22,65,28.5,45.5,23,29,22,18,17,30,52,47,56,38,22,43,31,45,33,46,36,33,55,54,33,13,18,21,61,48,24,35,30,34,40,35,50,39,56,28,56,56,24,18,24,23,6,45,40,57,32,62,54,43,52,62,67,63,61,48,18,52,39,48,49,17,39,31,40,61,47,35,64,60,60,54,21,55,31,57,45,50,27,50,21,51,21,31,62,36,30,28,30,18,25,34,36,57,18,23,36,28,51,32,19,28,1,4,12,36,34,19,23,26,42,27,24,15,60,40,20,25,36,25,42,42,0.833299994,26,22,35,19,44,54,52,37,29,25,45,29,28,29,28,24,8,31,31,22,30,21,8,18,48,28,32,17,29,24,25,18,18,34,54,8,42,34,27,30,23,21,18,40,29,18,36,38,35,38,34,34,16,26,47,21,21,24,24,34,30,52,30,0.666700006,24,44,6,28,62,30,7,43,45,24,24,49,48,55,24,32,21,18,20,23,36,54,50,44,29,21,42,63,60,33,17,42,24,47,24,22,32,23,34,24,22,35,45,57,31,26,30,1,3,25,22,17,34,36,24,61,50,42,57,1,31,24,30,40,32,30,46,13,41,19,39,48,70,27,54,39,16,62,32.5,14,2,3,36.5,26,19,28,20,29,39,22,23,29,28,50,19,41,21,19,43,32,34,30,27,2,8,33,36,34,30,28,23,0.833299994,3,24,50,19,21,26,25,27,25,18,20,30,59,30,35,40,25,41,25,18.5,14,50,23,28,27,29,27,40,31,30,23,31,12,40,32.5,27,29,2,4,29,0.916700006,5,36,33,66,31,26,24,42,13,16,35,16,25,20,18,30,26,40,0.833299994,18,26,26,20,24,25,35,18,32,19,4,6,2,17,38,9,11,39,27,26,39,20,26,25,18,24,35,5,9,3,13,5,40,23,38,45,21,23,17,30,23,13,20,32,33,0.75,0.75,5,24,18,40,26,20,18,45,27,22,19,26,22,20,32,21,18,26,6,9,40,32,21,22,20,29,22,22,35,18.5,21,19,18,21,30,18,38,17,17,21,21,21,28,24,16,37,28,24,21,32,29,26,18,20,18,24,36,24,31,31,22,30,70.5,43,35,27,19,30,9,3,36,59,19,17,44,17,22.5,45,22,19,30,29,0.333299994,34,28,27,25,24,22,21,17,36.5,36,30,16,1,0.166700006,26,33,25,22,36,19,17,42,43,32,19,30,24,23,33,65,24,23,22,18,16,45,39,17,15,47,5,40.5,40.5,18,26,21,9,18,16,48,25,22,16,9,33,41,31,38,9,1,11,10,16,14,40,43,51,32,20,37,28,19,24,17,28,24,20,23.5,41,26,21,45,25,11,27,18,26,23,22,28,28,2,22,43,28,27,42,30,27,25,29,21,20,48,17,34,26,22,33,31,29,4,1,49,33,19,27,23,32,27,20,21,32,17,21,30,21,33,22,4,39,18.5,34.5,44,22,26,4,29,26,1,18,36,25,37,22,26,29,29,22,22,32,34.5,36,39,24,25,45,36,30,20,28,30,26,20.5,27,51,23,32,24,22,29,30.5,35,33,15,35,24,19,55.5,21,24,21,28,25,6,27,34,24,18,22,15,1,20,19,33,12,14,29,28,18,26,21,41,39,21,28.5,22,61,23,22,9,28,42,31,28,32,20,23,20,20,16,31,2,6,3,8,29,1,7,2,16,14,41,21,19,32,0.75,3,26,21,25,22,25,24,28,19,25,18,32,17,24,38,21,10,4,7,2,8,39,22,35,50,47,2,18,41,50,16,25,38.5,14.5,24,21,39,1,24,4,25,20,24.5,29,22,40,21,18,4,10,9,2,40,45,19,30,32,33,23,21,60.5,19,22,31,27,2,29,16,44,25,74,14,24,25,34,0.416700006,16,32,30.5,44,25,7,9,29,36,18,63,11.5,40.5,10,36,30,33,28,28,47,18,31,16,31,22,20,14,22,22,32.5,38,51,18,21,47,28.5,21,27,36,27,15,45.5,14.5,26.5,27,29)";
        String sib_string = "c(0,1,1,1,1,0,1,0,2,0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,1,1,1,1,0,0,0,1,1,0,0,1,1,0,2,1,0,1,1,1,0,0,0,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,1,0,0,0,3,3,3,3,1,1,0,2,1,0,1,1,1,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,1,0,1,1,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,0,2,1,1,1,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,2,2,2,1,1,0,1,0,0,0,0,1,1,0,0,0,0,1,1,1,1,0,1,1,1,0,0,1,1,1,0,1,1,0,0,0,1,1,1,1,0,1,0,0,0,0,0,1,1,0,0,0,0,0,1,1,1,0,1,0,0,0,0,0,0,1,1,0,0,0,1,1,0,0,0,0,0,0,1,1,0,2,2,2,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,1,1,0,0,1,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,0,0,1,0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,1,1,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1,1,1,1,1,0,2,2,2,1,2,2,0,1,0,1,1,0,0,1,1,0,0,1,2,0,2,2,0,0,1,1,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,3,0,0,1,1,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,1,0,0,0,0,1,1,1,1,0,1,1,1,1,0,0,0,0,0,0,1,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,4,4,4,4,4,4,4,1,0,0,1,0,0,1,1,0,0,4,4,4,4,4,1,0,1,0,0,0,0,0,0,0,0,1,3,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,2,0,2,2,1,1,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,0,0,2,2,2,1,1,0,0,0,0,1,0,1,0,5,5,5,5,5,5,1,1,0,0,0,2,2,0,0,0,1,1,0,0,2,1,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,1,0,0,0,0,1,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,2,2,0,3,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,3,3,3,0,4,4,4,4,4,0,0,0,0,1,1,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,4,4,4,4,4,0,0,0,1,1,1,1,0,0,0,0,0,8,0,0,0,1,0,1,0,0,0,0,0,0,0,0,3,3,3,3,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,1,0,0,0,0,1,0,0,1,1,0,0,0,0,2,3,2,1,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,1,1,0,1,0,0,0)";
        String par_string = "c(0,2,2,2,2,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,0,0,0,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,2,0,1,2,1,0,0,0,0,0,0,2,0,0,1,2,1,0,1,0,0,0,0,1,0,0,0,0,0,2,2,2,2,4,4,0,0,0,2,1,1,0,0,0,0,1,1,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,2,2,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,2,2,2,3,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,1,1,0,0,0,0,0,0,0,0,0,0,0,2,1,1,0,1,2,1,0,0,0,0,0,0,0,0,1,1,0,2,1,1,1,2,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,3,0,0,0,0,0,0,0,2,1,1,0,0,0,0,0,0,2,1,1,0,0,0,0,0,0,0,1,2,0,0,0,0,0,2,1,1,0,0,0,1,0,2,0,0,0,0,0,0,0,1,1,0,2,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,2,0,1,1,0,0,2,1,1,2,2,2,2,0,0,0,0,1,1,1,0,3,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,2,2,2,2,0,0,0,0,0,0,0,0,2,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,2,0,1,0,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,0,0,1,1,2,0,0,0,0,0,1,1,3,0,0,0,0,0,0,1,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,2,2,2,2,2,0,0,0,0,0,2,1,1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,5,0,0,5,0,0,0,0,0,0,2,2,2,2,2,5,0,5,0,0,0,0,0,0,0,0,0,0,1,1,1,3,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,2,0,0,1,1,0,0,0,0,0,0,0,2,1,1,0,0,0,0,0,0,0,0,0,0,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,1,1,0,0,0,0,0,0,2,2,2,3,3,0,0,0,2,1,0,1,0,2,2,2,2,2,2,6,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,2,1,1,1,1,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,2,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,1,1,1,1,4,1,1,1,1,1,5,0,0,0,1,1,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,5,0,0,0,0,1,1,2,0,0,0,0,2,0,0,0,1,2,1,0,0,0,0,0,0,0,0,2,2,2,2,4,4,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,1,2,0,0,0,1,2,2,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)";

        ArrayList<String> featues_strings = new ArrayList<>();
        featues_strings.add(surv_string); featues_strings.add(sex_string); featues_strings.add(age_string); featues_strings.add(sib_string); featues_strings.add(par_string);
        HashMap<String,String> input_variables = new HashMap<>();
        HashMap<String,String> input_type_map = new HashMap<>();



        TreeMap<String, Pair<RDDL.PVAR_NAME,ArrayList<RDDL.LCONST>>> variables = new TreeMap<>();
        HashMap<String, TreeSet<String>> input_factors = new HashMap<>();
        for(int i =0 ; i<array_pvars.size();i++){
            Pair<RDDL.PVAR_NAME,ArrayList<RDDL.LCONST>> temp_pair = new Pair(array_pvars.get(i),new ArrayList<RDDL.LCONST>());
            String feat_name = EarthForPlanner._getRFeat(temp_pair);
            variables.put(feat_name,temp_pair);
            input_variables.put(feat_name,featues_strings.get(i));
            input_type_map.put(feat_name,all_feat_type[i]);
            input_factors.put(feat_name,new TreeSet<String>());

        }

        EarthForPlanner earth_obj = new EarthForPlanner();

        TreeSet<String>output_factors = new TreeSet<String>();
        output_factors.add("0");
        output_factors.add("1");
        output_factors.add("2");
        String [] earthPWLS =earth_obj.runEarth(variables,input_variables,target_string,input_type_map,target_type,input_factors,output_factors,hm_variables,hmtypes);
        earth_obj.output_writer.write(">>>>>>>Earth Output Beginning :"+"\n");
        for(String str_temp : earthPWLS){
            earth_obj.output_writer.write(str_temp + "\n");

        }
        earth_obj.output_writer.flush();

        earth_obj.output_writer.write(">>>>>>>>>>Earth Output End"+"\n");



        ArrayList<RDDL.EXPR> input_exprs = new ArrayList<>();
        ArrayList<RDDL.EXPR> max_exprs   = new ArrayList<>();
        for(String earth_str : earthPWLS){

            RDDL.EXPR temp_expr_mapp = earth_obj.reconstruct_expr_new(earth_str, variables, input_type_map, target_type, hm_variables, hmtypes);
            input_exprs.add(temp_expr_mapp);

        }
        earth_obj.output_writer.write(">>>>>>Start of Independent EXPR's for each pClass"+"\n");

        for(int j=0 ; j<input_exprs.size();j++){

            earth_obj.output_writer.write("EXPR "+String.valueOf(j+1) + "::: \n" +input_exprs.get(j).toString()+"\n");

        }
        for(int j=0;j<input_exprs.size();j++){
            RDDL.FUN_EXPR max_exprs_fun = new RDDL.FUN_EXPR("max",input_exprs);
            RDDL.COMP_EXPR tempcomp=new RDDL.COMP_EXPR(max_exprs_fun,input_exprs.get(j),"==");
            max_exprs.add(tempcomp);
        }

        earth_obj.output_writer.write(">>>>>>End of Independent EXPR's for each pClass" +"\n\n");
        earth_obj.output_writer.flush();
        RDDL.EXPR final_expr = earth_obj.get_final_target(input_exprs,output_factors,false,targetPvar,type_map,hm_variables,hmtypes);
        earth_obj.output_writer.write("final EXPR  ::::\n" + final_expr.toString());


        //This is for sample Checking.
        State s = null;
        ///////////////////////////////This is for Gurobi testing
        earth_obj.output_writer.flush();
        RDDL.PVAR_EXPR sex_pexpr=(new RDDL.PVAR_EXPR("sex",new ArrayList()));
        RDDL.BOOL_CONST_EXPR sex_const = new RDDL.BOOL_CONST_EXPR(true);

        RDDL.PVAR_EXPR age_pexpr=(new RDDL.PVAR_EXPR("age",new ArrayList()));
        RDDL.REAL_CONST_EXPR age_const = new RDDL.REAL_CONST_EXPR(28.0);

        GRBVar x1_var_lhs = sex_pexpr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = sex_const.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(sex_pexpr._pName.toString()) +"="+ name_map.get( sex_const.toString() ));

        GRBVar x2_var_lhs = age_pexpr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = age_const.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(age_pexpr._pName.toString()) +"="+ name_map.get( age_const.toString() ));
        try{
            GRBVar var1  = final_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
            //static_grb_model.setObjective( new GRBLinExpr() );

//            GRBLinExpr all_sum = new GRBLinExpr();
//            all_sum.addTerm(1.0d, RDDL.EXPR.grb_cache.get(final_expr));
//            static_grb_model.setObjective(all_sum);
            static_grb_model.optimize();
            static_grb_model.write("etitanic.lp");
            //These are values set to.
            System.out.println(">>>>>>Value of Sex : "+ sex_pexpr.toString());
            System.out.println(">>>>>>Value of Age : "+ age_pexpr.toString());
            for(int i=0 ; i<input_exprs.size();i++){
                System.out.println(">>>>>>Value of EXPR ::: "+input_exprs.get(i).toString());
                System.out.println(String.valueOf(grb_cache.get(input_exprs.get(i)).get(GRB.DoubleAttr.X)));

            }
            for(int i=0 ; i<max_exprs.size();i++){
                System.out.println(">>>>>>Value of EXPR ::: "+max_exprs.get(i).toString());
                System.out.println(String.valueOf(grb_cache.get(max_exprs.get(i)).get(GRB.DoubleAttr.X)));
            }

            System.out.println("dlfdjfkd");
        }catch (Exception e){
            System.out.println("dfjdkfkdjfkd");
        }


        earth_obj.output_writer.close();

    }


    public static void main(String[] args)throws Exception{


        //Checking Titanic Dataset.

        testCaseTitanicExample();





    }





}
