package rddl.det.comMipNew;

import gurobi.*;
import org.apache.commons.math3.random.RandomDataGenerator;
import rddl.RDDL;
import rddl.RDDL.OPER_EXPR;
import rddl.RDDL.QUANT_EXPR;
import rddl.RDDL.BOOL_EXPR;
import rddl.RDDL.COMP_EXPR;
import rddl.RDDL.EXPR;
import rddl.RDDL.LCONST;
import rddl.RDDL.LVAR;
import rddl.RDDL.OBJECTS_DEF;
import rddl.RDDL.PVAR_EXPR;
import rddl.RDDL.PVAR_NAME;
import rddl.RDDL.REAL_CONST_EXPR;
import rddl.RDDL.TYPE_NAME;
import rddl.RDDL.ENUM_VAL;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import rddl.RDDL.LTERM;
import rddl.RDDL.AGG_EXPR;
import rddl.RDDL.LTYPED_VAR;
import rddl.RDDL.OBJECT_VAL;
import rddl.RDDL.CONN_EXPR;
import rddl.RDDL.Discrete;
import rddl.RDDL.TYPE_DEF;

import static rddl.RDDL.EXPR.*;

public class TestCasesGurobi {

    //This part is for GURBOI OBJECT CREATIOIN>
    protected static GRBEnv  grb_env;
    protected static  GRBModel static_grb_model = null;



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




    public static void testCaseGurobiOPER() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());
        PVAR_EXPR p_x3 = new PVAR_EXPR("X3",new ArrayList());
        PVAR_EXPR p_x4 = new PVAR_EXPR("X4",new ArrayList<>());





        //This is checking the case when X1,X2,X3 are integers.

        TYPE_NAME int_type = new TYPE_NAME("bool");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "bool", new ArrayList(), true);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "bool", new ArrayList(), true);
        hm_variables.put(p_x2._pName,x2_s_def);

        RDDL.PVARIABLE_STATE_DEF x3_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x3._pName,x3_s_def);

        RDDL.PVARIABLE_STATE_DEF x4_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x4._pName,x4_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.BINARY);
        type_map.put(p_x2._pName,GRB.BINARY);
        type_map.put(p_x3._pName,GRB.INTEGER);
        type_map.put(p_x4._pName,GRB.INTEGER);

        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();





        //checking equality "==", X1==X2
        OPER_EXPR x1_eq_x2 = new OPER_EXPR(p_x1,p_x2,"*");

        OPER_EXPR x1_p_x2 = new OPER_EXPR(p_x3,p_x4,"+");

        OPER_EXPR x1_or = new OPER_EXPR(p_x1,x1_p_x2,"*");
        OPER_EXPR x1_or_2 = new OPER_EXPR(x1_p_x2,p_x1,"*");


        //x1_eq_x2.getDoubleValue(constants,objects,hmtypes,hm_variables,null);

        //Adding Default values.
        EXPR def_val1=new RDDL.BOOL_CONST_EXPR(false);
        EXPR def_val2 = new RDDL.BOOL_CONST_EXPR(false);

        EXPR def_val3=new RDDL.INT_CONST_EXPR(1);
        EXPR def_val4 = new RDDL.INT_CONST_EXPR(2);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));

        GRBVar x3_var_lhs = p_x3.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x3_var_rhs = def_val3.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x3_var_lhs,GRB.EQUAL,x3_var_rhs, name_map.get(p_x3.toString()) +"="+ name_map.get(def_val3.toString()));


        GRBVar x4_var_lhs = p_x4.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x4_var_rhs = def_val4.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x4_var_lhs,GRB.EQUAL,x4_var_rhs, name_map.get(p_x4.toString()) +"="+ name_map.get(def_val4.toString()));


        GRBVar eq_var =x1_or_2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.write("testing_Mult.lp");
        static_grb_model.optimize();
        double val =grb_cache.get(x1_or_2).get(GRB.DoubleAttr.X);
        System.out.println("KDjfkdjfkd");

        System.out.println("########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() + "   x3 : "+ def_val3.toString() + "   x4 : "+ def_val4.toString() +"    "+x1_or_2.toString()+ "   " +String.valueOf(val) );



//
//        /////////////////////////////////////////////////////////
//        //die-value(?d) == @1
//
//        //LVAR Arraylist ;
//        LVAR l1 = new LVAR("?d");
//        ArrayList<LVAR> lvars_1 = new ArrayList<>();  lvars_1.add(l1);
//
//        //pvar_expr...
//        PVAR_EXPR die_value_p  = new PVAR_EXPR("die-value",lvars_1);
//        PVAR_EXPR nf_pname     = new PVAR_EXPR("NF",lvars_1);
//        PVAR_EXPR roll_p       = new PVAR_EXPR("roll",lvars_1);
//
//        ENUM_VAL e1 = new ENUM_VAL("@1");
//        ENUM_VAL e2 = new ENUM_VAL("@2");
//        ENUM_VAL e3 = new ENUM_VAL("@3");
//        ENUM_VAL e4 = new ENUM_VAL("@4");
//        ENUM_VAL e5 = new ENUM_VAL("@5");
//        ENUM_VAL e6 = new ENUM_VAL("@6");
//
//
//        //Checking ENUM_VAL isConstant()
//        //hmtypes are defined!!!!.
//        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
//        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
//        array_enum.add(e6);
//        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
//
//        //First Hmtype
//        TYPE_NAME s_int = new TYPE_NAME("small-int");
//        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
//        hmtypes.put(s_int,enum_t_def);
//
//        //second hmtype
//        TYPE_NAME die_t = new TYPE_NAME("die");
//        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
//        hmtypes.put(die_t,die_o_def);
//
//
//        //hmvariables are defined!!!!.
//        //First fluent variable
//        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
//        PVAR_NAME die_value = new PVAR_NAME("die-value");
//        ArrayList<String> ar_string = new ArrayList<>();
//        ar_string.add("die");
//        RDDL.PVARIABLE_STATE_DEF p_s_def = new RDDL.PVARIABLE_STATE_DEF("die-value",false, "small-int", ar_string, e1);
//        hm_variables.put(die_value_p._pName,p_s_def);
//        //Second Non-fluent Variable.
//
//        ArrayList<String> ar_string_1 = new ArrayList<>();
//        ar_string_1.add("die");
//        RDDL.PVARIABLE_STATE_DEF nf_s_def = new RDDL.PVARIABLE_STATE_DEF("NF",true, "small-int", ar_string_1, e1);
//        hm_variables.put(nf_pname._pName,nf_s_def);
//
//        //Third Action Fluent.
//        ArrayList<String> ar_string_2 = new ArrayList<>();
//        ar_string_2.add("roll");
//        RDDL.PVARIABLE_ACTION_DEF ac_def = new RDDL.PVARIABLE_ACTION_DEF("roll","bool",ar_string_2,false);
//        hm_variables.put(roll_p._pName,ac_def);
//
//
//
//        //defining objects////////////////////////////////////////
//        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
//        TYPE_NAME die_type = new TYPE_NAME("die");
//        LCONST lconst_d1   = new OBJECT_VAL("$d1");
//        LCONST lconst_d2   = new OBJECT_VAL("$d2");
//        LCONST lconst_d3   = new OBJECT_VAL("$d3");
//        ArrayList<Object> temp_objects = new ArrayList<>();
//        temp_objects.add(lconst_d1); temp_objects.add(lconst_d2); temp_objects.add(lconst_d3);
//        OBJECTS_DEF ob = new OBJECTS_DEF("die",temp_objects);
//        objects.put(die_type,ob);
//        /////////////////////////////////////////////////////////////////
//        //Defining Constants
//        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
//        Map<ArrayList<LCONST>,Object> cons1 = new HashMap<>();
//        for(int i = 1; i<4 ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(new OBJECT_VAL("$d"+Integer.valueOf(i).toString())); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons1.put(lconst_array,new ENUM_VAL("@"+Integer.valueOf(i).toString()));
//        }
//        constants.put(nf_pname._pName,cons1);
//        //////////////////////////////////////////////////////////////////
//
//
//
//        LTYPED_VAR qd1 = new LTYPED_VAR("?d","die");
//        ArrayList<LTYPED_VAR> array_ltyped = new ArrayList<>();     array_ltyped.add(qd1);
//
//
//        COMP_EXPR e_1  = new COMP_EXPR(die_value_p,e1,"==");
//        COMP_EXPR e_2  = new COMP_EXPR(nf_pname,e1,"==");
//        CONN_EXPR e_3  = new CONN_EXPR(e_1,e_2,"|");
//        QUANT_EXPR e_4 = new QUANT_EXPR("forall",array_ltyped,e_3);
//
//        //e_2.substitute(null,null,objects,hmtypes, hm_variables);
//
//        Map<LVAR, LCONST> subs = new HashMap<>();
//        LVAR a_lvar = new LVAR("?d");
//        LCONST a_cont = new OBJECT_VAL("$d1");
//        subs.put(a_lvar,a_cont);
//        EXPR sub_e_2 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_e_2.getDoubleValue(constants,objects,hmtypes,hm_variables,  null);
//        EXPR sub_expr = nf_pname.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_expr.equals(new ENUM_VAL("@1"));
////        EXPR sub_e1 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//
//        //Adding type_map :
//        Map<PVAR_NAME,Character> type_map = new HashMap<>();
//        type_map.put(die_value_p._pName,GRB.INTEGER);
//
//
//        //This Piece of code for testing Discerte Expression.
//
//
//        //GRBVar gvar = sub_e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//
//        //new RDDL.Discrete("small-int")
//
//        //Defining 1/6
//
//        OPER_EXPR div_6 = new OPER_EXPR(new RDDL.INT_CONST_EXPR(1),new RDDL.INT_CONST_EXPR(6),"/");
//        ArrayList<EXPR> discrete_array = new ArrayList<>();
//        for(int i=1 ; i< 7 ; i++){
//            discrete_array.add(new ENUM_VAL("@"+String.valueOf(i)));
//            discrete_array.add(div_6);
//        }
//
//        Discrete dis = new Discrete("small-int",discrete_array);
//
//        RDDL.IF_EXPR if_else_expr = new RDDL.IF_EXPR(roll_p,dis,die_value_p);
//        RandomDataGenerator rand = new RandomDataGenerator();
//        EXPR test_dis_sample = dis.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //EXPR test_1 =( ((OPER_EXPR) ((OPER_EXPR) ((OPER_EXPR)((OPER_EXPR) ((OPER_EXPR) test_dis_sample)._e1)._e1)._e1)._e1)._e1);
//        //test_1.getDoubleValue(constants,objects,hmtypes,hm_variables);
//        EXPR test_2 = new OPER_EXPR(e1,new REAL_CONST_EXPR(2.0),"*");
//        //test_2.isPiecewiseLinear(constants,objects,hmtypes,hm_variables);
//        //test_2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        System.out.println("dkfjkdjfkdjfkjd");
//        EXPR die_sub_expr = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        //die_sub_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.write("testing.lp");
//
//        ///////////////////////////////////////////////////////////        //FOrall Expression
//        //NF2 adding king
//        ENUM_VAL k_1 = new ENUM_VAL("@Blue");
//        ENUM_VAL k_2 = new ENUM_VAL("@Green");
//        ENUM_VAL k_3 = new ENUM_VAL("@Red");
//        ENUM_VAL k_4 = new ENUM_VAL("@Yellow");
//        ENUM_VAL k_5 = new ENUM_VAL("@Purple");
//        ENUM_VAL k_6 = new ENUM_VAL("@Pink");
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        ArrayList<ENUM_VAL> array_enum1 = new ArrayList<>(); array_enum.add(e1);
//        array_enum1.add(k_1); array_enum1.add(k_2); array_enum1.add(k_3); array_enum1.add(k_4);
//
//
//        //First Hmtype
//        TYPE_NAME color_type = new TYPE_NAME("color");
//        TYPE_DEF enum_color_def = new RDDL.ENUM_TYPE_DEF("color", array_enum1);
//        hmtypes.put(color_type,enum_color_def);
//
//
//        LVAR l_col = new LVAR("?c");
//        ArrayList<LVAR> lvars_col_1 = new ArrayList<>();  lvars_col_1.add(l_col);
//        PVAR_EXPR a_to =new PVAR_EXPR("a-to",lvars_col_1);
//        PVAR_EXPR b_to = new PVAR_EXPR("B-TO",lvars_col_1);
//
//        ArrayList<String> co_string = new ArrayList<>();
//        co_string.add("color");
//
//        RDDL.PVARIABLE_STATE_DEF a_to_def = new RDDL.PVARIABLE_STATE_DEF("a-to",false, "bool", co_string, false);
//        hm_variables.put(a_to._pName,a_to_def);
//
//        RDDL.PVARIABLE_STATE_DEF b_to_def = new RDDL.PVARIABLE_STATE_DEF("B-TO",true, "bool", co_string, false);
//        hm_variables.put(b_to._pName,b_to_def);
//
//        Map<ArrayList<LCONST>,Object> cons2 = new HashMap<>();
//        ArrayList<ENUM_VAL> enum_array = new ArrayList<>();
//        enum_array.add(k_1); enum_array.add(k_2);enum_array.add(k_3);enum_array.add(k_4);
//        for(int i = 0; i<enum_array.size() ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(enum_array.get(i)); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons2.put(lconst_array,true);
//        }
//        constants.put(b_to._pName,cons2);
//
//        LTYPED_VAR colr_type = new LTYPED_VAR("?c","color");
//        ArrayList<LTYPED_VAR> array_ltyped1 = new ArrayList<>();     array_ltyped1.add(colr_type);
//
//
//        EXPR f2 = new CONN_EXPR(a_to,b_to,"=>");
//        EXPR f1 = new QUANT_EXPR("forall",array_ltyped1,f2);
//        EXPR p2 = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        ArrayList<BOOL_EXPR> n_list = new ArrayList<>();
//        n_list.add((BOOL_EXPR) p2); n_list.add(new RDDL.BOOL_CONST_EXPR(true));
//        EXPR f4 = new CONN_EXPR(n_list,"=>");
//        //EXPR f3 = new CONN_EXPR(new RDDL.BOOL_CONST_EXPR(true),new RDDL.BOOL_CONST_EXPR(true),"^");
//
//        EXPR s1 =f1.substitute(Collections.EMPTY_MAP,constants,objects,hmtypes,hm_variables);
//
//        ArrayList<EXPR> discrete_array1 = new ArrayList<>();
//        for(int i=0 ; i< array_enum1.size() ; i++){
//            discrete_array1.add(array_enum1.get(i));
//            discrete_array1.add(div_6);
//        }
//
//        Discrete dis1 = new Discrete("color",discrete_array1);
//        EXPR sam1 =dis1.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //sam1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        EXPR c_m_va = new OPER_EXPR(k_1,new RDDL.BOOL_CONST_EXPR(true),"*");
//        //c_m_va.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //Current Phase Code
//        PVAR_EXPR curr_phase =new PVAR_EXPR("Current-Phase",new ArrayList());
////        ArrayList<String> cur_list = new ArrayList<>();
////        cur_list.add("die");
//        RDDL.PVARIABLE_STATE_DEF cur_phase_psd = new RDDL.PVARIABLE_STATE_DEF("Current-Phase",true, "small-int", new ArrayList(), e1);
//        hm_variables.put(curr_phase._pName,cur_phase_psd);
//
//        COMP_EXPR curr_expr = new COMP_EXPR(curr_phase,e1,"==");
//
//        System.out.println("dkjfkdjfkd");
//
//        PVAR_EXPR temp1_pvar = new PVAR_EXPR("temp1",new ArrayList());
//        PVAR_EXPR temp2_pvar = new PVAR_EXPR("temp2",new ArrayList());
//
//        type_map.put(temp1_pvar._pName,GRB.BINARY);
//        type_map.put(temp2_pvar._pName,GRB.BINARY);
//        type_map.put(curr_phase._pName,GRB.INTEGER);
//        RDDL.PVARIABLE_STATE_DEF temp1_psd = new RDDL.PVARIABLE_STATE_DEF("temp1",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp1_pvar._pName,temp1_psd);
//        RDDL.PVARIABLE_STATE_DEF temp2_psd = new RDDL.PVARIABLE_STATE_DEF("temp2",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp2_pvar._pName,temp2_psd);
//
//
//
//
//        CONN_EXPR or_expr =new CONN_EXPR(temp1_pvar, temp2_pvar,"|");
//
//        //z = v1 or v2
//        GRBVar z = curr_expr.getGRBVar(curr_expr, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar or_grbvar =or_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(z,GRB.EQUAL,or_grbvar,EXPR.name_map.get(curr_expr.toString()) +"==" +EXPR.name_map.get(or_expr.toString()) );
//
//
//        GRBVar t1_grb = temp1_pvar.getGRBVar(temp1_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar t2_grb = temp2_pvar.getGRBVar(temp2_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//
//
//
//        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
//        //v1=1 : -ep <= x-y <= 0
//        //v1=0 : -M-ep <= x-y <= 0,
//
//
//        final GRBLinExpr minus_M_one_minus_z = new GRBLinExpr();//-M(1-v1)-ep = -M+Mv1 -ep
//        minus_M_one_minus_z.addConstant(-1d*M );
//        minus_M_one_minus_z.addTerm(1d*M, t1_grb);
//        minus_M_one_minus_z.addConstant(-1d*ep);
//
//        final GRBLinExpr zero_grb = new GRBLinExpr();
//        zero_grb.addConstant(0);
//
//        static_grb_model.addConstr( minus_M_one_minus_z, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, zero_grb, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_2" );
//
//
//        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
//        //v2 =1 : 0 <= x-y <= ep
//        //v2=0 : 0 <= x-y <= M+ep
//
//
//        final GRBLinExpr ex_val  = new GRBLinExpr();//M(1-v2)+ep = M-Mv1+ep
//        ex_val.addConstant(M);
//        ex_val.addTerm(-1d*M, t2_grb);
//        ex_val.addConstant(ep);
//
//
//        static_grb_model.addConstr( zero_grb, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, ex_val, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_2" );
//
//
//
//        //Working on Current-phase = @1, and @1.
//
//        GRBVar cur_expr_grbvar =curr_phase.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        GRBVar enum_val_grbvar = e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(cur_expr_grbvar,GRB.EQUAL,enum_val_grbvar,EXPR.name_map.get(curr_phase.toString())+"=="+EXPR.name_map.get(new RDDL.INT_CONST_EXPR(e1.enum_to_int(hmtypes,hm_variables)).toString()));
//
//
//        System.out.println("dkfkdfjdk");
//
//
//
//
//
//
//
//
//
//
//








        //z = [ x == y ]
        // z = v1 OR v2
        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
        //v1=1 : -ep <= x-y <= 0
        //v1=0 : -M-ep <= x-y <= 0,

        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
        //v2 =1 : 0 <= x-y <= ep
        //v2=0 : 0 <= x-y <= M+ep


    }




    public static void testCaseGurobiIntEqual() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());



        //This is checking the case when X1,X2,X3 are integers.

        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.INTEGER);
        type_map.put(p_x2._pName,GRB.INTEGER);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();





        //checking equality "==", X1==X2
        COMP_EXPR x1_eq_x2 = new COMP_EXPR(p_x1,p_x2,"==");

        //Adding Default values.
        EXPR def_val1=new RDDL.INT_CONST_EXPR(1);
        EXPR def_val2=new RDDL.INT_CONST_EXPR(2);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));




        GRBVar eq_var =x1_eq_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.write("testing_eq_int.lp");
        static_grb_model.optimize();
        grb_cache.get(x1_eq_x2).get(GRB.DoubleAttr.X);
        System.out.println("KDjfkdjfkd");


//
//        /////////////////////////////////////////////////////////
//        //die-value(?d) == @1
//
//        //LVAR Arraylist ;
//        LVAR l1 = new LVAR("?d");
//        ArrayList<LVAR> lvars_1 = new ArrayList<>();  lvars_1.add(l1);
//
//        //pvar_expr...
//        PVAR_EXPR die_value_p  = new PVAR_EXPR("die-value",lvars_1);
//        PVAR_EXPR nf_pname     = new PVAR_EXPR("NF",lvars_1);
//        PVAR_EXPR roll_p       = new PVAR_EXPR("roll",lvars_1);
//
//        ENUM_VAL e1 = new ENUM_VAL("@1");
//        ENUM_VAL e2 = new ENUM_VAL("@2");
//        ENUM_VAL e3 = new ENUM_VAL("@3");
//        ENUM_VAL e4 = new ENUM_VAL("@4");
//        ENUM_VAL e5 = new ENUM_VAL("@5");
//        ENUM_VAL e6 = new ENUM_VAL("@6");
//
//
//        //Checking ENUM_VAL isConstant()
//        //hmtypes are defined!!!!.
//        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
//        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
//        array_enum.add(e6);
//        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
//
//        //First Hmtype
//        TYPE_NAME s_int = new TYPE_NAME("small-int");
//        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
//        hmtypes.put(s_int,enum_t_def);
//
//        //second hmtype
//        TYPE_NAME die_t = new TYPE_NAME("die");
//        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
//        hmtypes.put(die_t,die_o_def);
//
//
//        //hmvariables are defined!!!!.
//        //First fluent variable
//        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
//        PVAR_NAME die_value = new PVAR_NAME("die-value");
//        ArrayList<String> ar_string = new ArrayList<>();
//        ar_string.add("die");
//        RDDL.PVARIABLE_STATE_DEF p_s_def = new RDDL.PVARIABLE_STATE_DEF("die-value",false, "small-int", ar_string, e1);
//        hm_variables.put(die_value_p._pName,p_s_def);
//        //Second Non-fluent Variable.
//
//        ArrayList<String> ar_string_1 = new ArrayList<>();
//        ar_string_1.add("die");
//        RDDL.PVARIABLE_STATE_DEF nf_s_def = new RDDL.PVARIABLE_STATE_DEF("NF",true, "small-int", ar_string_1, e1);
//        hm_variables.put(nf_pname._pName,nf_s_def);
//
//        //Third Action Fluent.
//        ArrayList<String> ar_string_2 = new ArrayList<>();
//        ar_string_2.add("roll");
//        RDDL.PVARIABLE_ACTION_DEF ac_def = new RDDL.PVARIABLE_ACTION_DEF("roll","bool",ar_string_2,false);
//        hm_variables.put(roll_p._pName,ac_def);
//
//
//
//        //defining objects////////////////////////////////////////
//        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
//        TYPE_NAME die_type = new TYPE_NAME("die");
//        LCONST lconst_d1   = new OBJECT_VAL("$d1");
//        LCONST lconst_d2   = new OBJECT_VAL("$d2");
//        LCONST lconst_d3   = new OBJECT_VAL("$d3");
//        ArrayList<Object> temp_objects = new ArrayList<>();
//        temp_objects.add(lconst_d1); temp_objects.add(lconst_d2); temp_objects.add(lconst_d3);
//        OBJECTS_DEF ob = new OBJECTS_DEF("die",temp_objects);
//        objects.put(die_type,ob);
//        /////////////////////////////////////////////////////////////////
//        //Defining Constants
//        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
//        Map<ArrayList<LCONST>,Object> cons1 = new HashMap<>();
//        for(int i = 1; i<4 ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(new OBJECT_VAL("$d"+Integer.valueOf(i).toString())); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons1.put(lconst_array,new ENUM_VAL("@"+Integer.valueOf(i).toString()));
//        }
//        constants.put(nf_pname._pName,cons1);
//        //////////////////////////////////////////////////////////////////
//
//
//
//        LTYPED_VAR qd1 = new LTYPED_VAR("?d","die");
//        ArrayList<LTYPED_VAR> array_ltyped = new ArrayList<>();     array_ltyped.add(qd1);
//
//
//        COMP_EXPR e_1  = new COMP_EXPR(die_value_p,e1,"==");
//        COMP_EXPR e_2  = new COMP_EXPR(nf_pname,e1,"==");
//        CONN_EXPR e_3  = new CONN_EXPR(e_1,e_2,"|");
//        QUANT_EXPR e_4 = new QUANT_EXPR("forall",array_ltyped,e_3);
//
//        //e_2.substitute(null,null,objects,hmtypes, hm_variables);
//
//        Map<LVAR, LCONST> subs = new HashMap<>();
//        LVAR a_lvar = new LVAR("?d");
//        LCONST a_cont = new OBJECT_VAL("$d1");
//        subs.put(a_lvar,a_cont);
//        EXPR sub_e_2 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_e_2.getDoubleValue(constants,objects,hmtypes,hm_variables,  null);
//        EXPR sub_expr = nf_pname.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_expr.equals(new ENUM_VAL("@1"));
////        EXPR sub_e1 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//
//        //Adding type_map :
//        Map<PVAR_NAME,Character> type_map = new HashMap<>();
//        type_map.put(die_value_p._pName,GRB.INTEGER);
//
//
//        //This Piece of code for testing Discerte Expression.
//
//
//        //GRBVar gvar = sub_e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//
//        //new RDDL.Discrete("small-int")
//
//        //Defining 1/6
//
//        OPER_EXPR div_6 = new OPER_EXPR(new RDDL.INT_CONST_EXPR(1),new RDDL.INT_CONST_EXPR(6),"/");
//        ArrayList<EXPR> discrete_array = new ArrayList<>();
//        for(int i=1 ; i< 7 ; i++){
//            discrete_array.add(new ENUM_VAL("@"+String.valueOf(i)));
//            discrete_array.add(div_6);
//        }
//
//        Discrete dis = new Discrete("small-int",discrete_array);
//
//        RDDL.IF_EXPR if_else_expr = new RDDL.IF_EXPR(roll_p,dis,die_value_p);
//        RandomDataGenerator rand = new RandomDataGenerator();
//        EXPR test_dis_sample = dis.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //EXPR test_1 =( ((OPER_EXPR) ((OPER_EXPR) ((OPER_EXPR)((OPER_EXPR) ((OPER_EXPR) test_dis_sample)._e1)._e1)._e1)._e1)._e1);
//        //test_1.getDoubleValue(constants,objects,hmtypes,hm_variables);
//        EXPR test_2 = new OPER_EXPR(e1,new REAL_CONST_EXPR(2.0),"*");
//        //test_2.isPiecewiseLinear(constants,objects,hmtypes,hm_variables);
//        //test_2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        System.out.println("dkfjkdjfkdjfkjd");
//        EXPR die_sub_expr = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        //die_sub_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.write("testing.lp");
//
//        ///////////////////////////////////////////////////////////        //FOrall Expression
//        //NF2 adding king
//        ENUM_VAL k_1 = new ENUM_VAL("@Blue");
//        ENUM_VAL k_2 = new ENUM_VAL("@Green");
//        ENUM_VAL k_3 = new ENUM_VAL("@Red");
//        ENUM_VAL k_4 = new ENUM_VAL("@Yellow");
//        ENUM_VAL k_5 = new ENUM_VAL("@Purple");
//        ENUM_VAL k_6 = new ENUM_VAL("@Pink");
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        ArrayList<ENUM_VAL> array_enum1 = new ArrayList<>(); array_enum.add(e1);
//        array_enum1.add(k_1); array_enum1.add(k_2); array_enum1.add(k_3); array_enum1.add(k_4);
//
//
//        //First Hmtype
//        TYPE_NAME color_type = new TYPE_NAME("color");
//        TYPE_DEF enum_color_def = new RDDL.ENUM_TYPE_DEF("color", array_enum1);
//        hmtypes.put(color_type,enum_color_def);
//
//
//        LVAR l_col = new LVAR("?c");
//        ArrayList<LVAR> lvars_col_1 = new ArrayList<>();  lvars_col_1.add(l_col);
//        PVAR_EXPR a_to =new PVAR_EXPR("a-to",lvars_col_1);
//        PVAR_EXPR b_to = new PVAR_EXPR("B-TO",lvars_col_1);
//
//        ArrayList<String> co_string = new ArrayList<>();
//        co_string.add("color");
//
//        RDDL.PVARIABLE_STATE_DEF a_to_def = new RDDL.PVARIABLE_STATE_DEF("a-to",false, "bool", co_string, false);
//        hm_variables.put(a_to._pName,a_to_def);
//
//        RDDL.PVARIABLE_STATE_DEF b_to_def = new RDDL.PVARIABLE_STATE_DEF("B-TO",true, "bool", co_string, false);
//        hm_variables.put(b_to._pName,b_to_def);
//
//        Map<ArrayList<LCONST>,Object> cons2 = new HashMap<>();
//        ArrayList<ENUM_VAL> enum_array = new ArrayList<>();
//        enum_array.add(k_1); enum_array.add(k_2);enum_array.add(k_3);enum_array.add(k_4);
//        for(int i = 0; i<enum_array.size() ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(enum_array.get(i)); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons2.put(lconst_array,true);
//        }
//        constants.put(b_to._pName,cons2);
//
//        LTYPED_VAR colr_type = new LTYPED_VAR("?c","color");
//        ArrayList<LTYPED_VAR> array_ltyped1 = new ArrayList<>();     array_ltyped1.add(colr_type);
//
//
//        EXPR f2 = new CONN_EXPR(a_to,b_to,"=>");
//        EXPR f1 = new QUANT_EXPR("forall",array_ltyped1,f2);
//        EXPR p2 = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        ArrayList<BOOL_EXPR> n_list = new ArrayList<>();
//        n_list.add((BOOL_EXPR) p2); n_list.add(new RDDL.BOOL_CONST_EXPR(true));
//        EXPR f4 = new CONN_EXPR(n_list,"=>");
//        //EXPR f3 = new CONN_EXPR(new RDDL.BOOL_CONST_EXPR(true),new RDDL.BOOL_CONST_EXPR(true),"^");
//
//        EXPR s1 =f1.substitute(Collections.EMPTY_MAP,constants,objects,hmtypes,hm_variables);
//
//        ArrayList<EXPR> discrete_array1 = new ArrayList<>();
//        for(int i=0 ; i< array_enum1.size() ; i++){
//            discrete_array1.add(array_enum1.get(i));
//            discrete_array1.add(div_6);
//        }
//
//        Discrete dis1 = new Discrete("color",discrete_array1);
//        EXPR sam1 =dis1.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //sam1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        EXPR c_m_va = new OPER_EXPR(k_1,new RDDL.BOOL_CONST_EXPR(true),"*");
//        //c_m_va.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //Current Phase Code
//        PVAR_EXPR curr_phase =new PVAR_EXPR("Current-Phase",new ArrayList());
////        ArrayList<String> cur_list = new ArrayList<>();
////        cur_list.add("die");
//        RDDL.PVARIABLE_STATE_DEF cur_phase_psd = new RDDL.PVARIABLE_STATE_DEF("Current-Phase",true, "small-int", new ArrayList(), e1);
//        hm_variables.put(curr_phase._pName,cur_phase_psd);
//
//        COMP_EXPR curr_expr = new COMP_EXPR(curr_phase,e1,"==");
//
//        System.out.println("dkjfkdjfkd");
//
//        PVAR_EXPR temp1_pvar = new PVAR_EXPR("temp1",new ArrayList());
//        PVAR_EXPR temp2_pvar = new PVAR_EXPR("temp2",new ArrayList());
//
//        type_map.put(temp1_pvar._pName,GRB.BINARY);
//        type_map.put(temp2_pvar._pName,GRB.BINARY);
//        type_map.put(curr_phase._pName,GRB.INTEGER);
//        RDDL.PVARIABLE_STATE_DEF temp1_psd = new RDDL.PVARIABLE_STATE_DEF("temp1",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp1_pvar._pName,temp1_psd);
//        RDDL.PVARIABLE_STATE_DEF temp2_psd = new RDDL.PVARIABLE_STATE_DEF("temp2",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp2_pvar._pName,temp2_psd);
//
//
//
//
//        CONN_EXPR or_expr =new CONN_EXPR(temp1_pvar, temp2_pvar,"|");
//
//        //z = v1 or v2
//        GRBVar z = curr_expr.getGRBVar(curr_expr, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar or_grbvar =or_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(z,GRB.EQUAL,or_grbvar,EXPR.name_map.get(curr_expr.toString()) +"==" +EXPR.name_map.get(or_expr.toString()) );
//
//
//        GRBVar t1_grb = temp1_pvar.getGRBVar(temp1_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar t2_grb = temp2_pvar.getGRBVar(temp2_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//
//
//
//        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
//        //v1=1 : -ep <= x-y <= 0
//        //v1=0 : -M-ep <= x-y <= 0,
//
//
//        final GRBLinExpr minus_M_one_minus_z = new GRBLinExpr();//-M(1-v1)-ep = -M+Mv1 -ep
//        minus_M_one_minus_z.addConstant(-1d*M );
//        minus_M_one_minus_z.addTerm(1d*M, t1_grb);
//        minus_M_one_minus_z.addConstant(-1d*ep);
//
//        final GRBLinExpr zero_grb = new GRBLinExpr();
//        zero_grb.addConstant(0);
//
//        static_grb_model.addConstr( minus_M_one_minus_z, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, zero_grb, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_2" );
//
//
//        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
//        //v2 =1 : 0 <= x-y <= ep
//        //v2=0 : 0 <= x-y <= M+ep
//
//
//        final GRBLinExpr ex_val  = new GRBLinExpr();//M(1-v2)+ep = M-Mv1+ep
//        ex_val.addConstant(M);
//        ex_val.addTerm(-1d*M, t2_grb);
//        ex_val.addConstant(ep);
//
//
//        static_grb_model.addConstr( zero_grb, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, ex_val, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_2" );
//
//
//
//        //Working on Current-phase = @1, and @1.
//
//        GRBVar cur_expr_grbvar =curr_phase.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        GRBVar enum_val_grbvar = e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(cur_expr_grbvar,GRB.EQUAL,enum_val_grbvar,EXPR.name_map.get(curr_phase.toString())+"=="+EXPR.name_map.get(new RDDL.INT_CONST_EXPR(e1.enum_to_int(hmtypes,hm_variables)).toString()));
//
//
//        System.out.println("dkfkdfjdk");
//
//
//
//
//
//
//
//
//
//
//








        //z = [ x == y ]
        // z = v1 OR v2
        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
        //v1=1 : -ep <= x-y <= 0
        //v1=0 : -M-ep <= x-y <= 0,

        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
        //v2 =1 : 0 <= x-y <= ep
        //v2=0 : 0 <= x-y <= M+ep


    }

    public static void testCaseGurobiRealEqual() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());



        //This is checking the case when X1,X2,X3 are integers.

        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "real", new ArrayList(), 1.0);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "real", new ArrayList(), 1.0);
        hm_variables.put(p_x2._pName,x2_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.CONTINUOUS);
        type_map.put(p_x2._pName,GRB.CONTINUOUS);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();





        //checking equality "==", X1==X2
        COMP_EXPR x1_eq_x2 = new COMP_EXPR(p_x1,p_x2,"==");

        //Adding Default values.
        EXPR def_val1=new RDDL.REAL_CONST_EXPR(1.0);
        EXPR def_val2=new RDDL.REAL_CONST_EXPR(1.0);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));




        GRBVar eq_var =x1_eq_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.write("testing_eq_real.lp");
        static_grb_model.optimize();
        grb_cache.get(x1_eq_x2).get(GRB.DoubleAttr.X);
        System.out.println("KDjfkdjfkd");


//
//        /////////////////////////////////////////////////////////
//        //die-value(?d) == @1
//
//        //LVAR Arraylist ;
//        LVAR l1 = new LVAR("?d");
//        ArrayList<LVAR> lvars_1 = new ArrayList<>();  lvars_1.add(l1);
//
//        //pvar_expr...
//        PVAR_EXPR die_value_p  = new PVAR_EXPR("die-value",lvars_1);
//        PVAR_EXPR nf_pname     = new PVAR_EXPR("NF",lvars_1);
//        PVAR_EXPR roll_p       = new PVAR_EXPR("roll",lvars_1);
//
//        ENUM_VAL e1 = new ENUM_VAL("@1");
//        ENUM_VAL e2 = new ENUM_VAL("@2");
//        ENUM_VAL e3 = new ENUM_VAL("@3");
//        ENUM_VAL e4 = new ENUM_VAL("@4");
//        ENUM_VAL e5 = new ENUM_VAL("@5");
//        ENUM_VAL e6 = new ENUM_VAL("@6");
//
//
//        //Checking ENUM_VAL isConstant()
//        //hmtypes are defined!!!!.
//        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
//        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
//        array_enum.add(e6);
//        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
//
//        //First Hmtype
//        TYPE_NAME s_int = new TYPE_NAME("small-int");
//        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
//        hmtypes.put(s_int,enum_t_def);
//
//        //second hmtype
//        TYPE_NAME die_t = new TYPE_NAME("die");
//        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
//        hmtypes.put(die_t,die_o_def);
//
//
//        //hmvariables are defined!!!!.
//        //First fluent variable
//        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
//        PVAR_NAME die_value = new PVAR_NAME("die-value");
//        ArrayList<String> ar_string = new ArrayList<>();
//        ar_string.add("die");
//        RDDL.PVARIABLE_STATE_DEF p_s_def = new RDDL.PVARIABLE_STATE_DEF("die-value",false, "small-int", ar_string, e1);
//        hm_variables.put(die_value_p._pName,p_s_def);
//        //Second Non-fluent Variable.
//
//        ArrayList<String> ar_string_1 = new ArrayList<>();
//        ar_string_1.add("die");
//        RDDL.PVARIABLE_STATE_DEF nf_s_def = new RDDL.PVARIABLE_STATE_DEF("NF",true, "small-int", ar_string_1, e1);
//        hm_variables.put(nf_pname._pName,nf_s_def);
//
//        //Third Action Fluent.
//        ArrayList<String> ar_string_2 = new ArrayList<>();
//        ar_string_2.add("roll");
//        RDDL.PVARIABLE_ACTION_DEF ac_def = new RDDL.PVARIABLE_ACTION_DEF("roll","bool",ar_string_2,false);
//        hm_variables.put(roll_p._pName,ac_def);
//
//
//
//        //defining objects////////////////////////////////////////
//        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
//        TYPE_NAME die_type = new TYPE_NAME("die");
//        LCONST lconst_d1   = new OBJECT_VAL("$d1");
//        LCONST lconst_d2   = new OBJECT_VAL("$d2");
//        LCONST lconst_d3   = new OBJECT_VAL("$d3");
//        ArrayList<Object> temp_objects = new ArrayList<>();
//        temp_objects.add(lconst_d1); temp_objects.add(lconst_d2); temp_objects.add(lconst_d3);
//        OBJECTS_DEF ob = new OBJECTS_DEF("die",temp_objects);
//        objects.put(die_type,ob);
//        /////////////////////////////////////////////////////////////////
//        //Defining Constants
//        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
//        Map<ArrayList<LCONST>,Object> cons1 = new HashMap<>();
//        for(int i = 1; i<4 ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(new OBJECT_VAL("$d"+Integer.valueOf(i).toString())); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons1.put(lconst_array,new ENUM_VAL("@"+Integer.valueOf(i).toString()));
//        }
//        constants.put(nf_pname._pName,cons1);
//        //////////////////////////////////////////////////////////////////
//
//
//
//        LTYPED_VAR qd1 = new LTYPED_VAR("?d","die");
//        ArrayList<LTYPED_VAR> array_ltyped = new ArrayList<>();     array_ltyped.add(qd1);
//
//
//        COMP_EXPR e_1  = new COMP_EXPR(die_value_p,e1,"==");
//        COMP_EXPR e_2  = new COMP_EXPR(nf_pname,e1,"==");
//        CONN_EXPR e_3  = new CONN_EXPR(e_1,e_2,"|");
//        QUANT_EXPR e_4 = new QUANT_EXPR("forall",array_ltyped,e_3);
//
//        //e_2.substitute(null,null,objects,hmtypes, hm_variables);
//
//        Map<LVAR, LCONST> subs = new HashMap<>();
//        LVAR a_lvar = new LVAR("?d");
//        LCONST a_cont = new OBJECT_VAL("$d1");
//        subs.put(a_lvar,a_cont);
//        EXPR sub_e_2 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_e_2.getDoubleValue(constants,objects,hmtypes,hm_variables,  null);
//        EXPR sub_expr = nf_pname.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_expr.equals(new ENUM_VAL("@1"));
////        EXPR sub_e1 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//
//        //Adding type_map :
//        Map<PVAR_NAME,Character> type_map = new HashMap<>();
//        type_map.put(die_value_p._pName,GRB.INTEGER);
//
//
//        //This Piece of code for testing Discerte Expression.
//
//
//        //GRBVar gvar = sub_e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//
//        //new RDDL.Discrete("small-int")
//
//        //Defining 1/6
//
//        OPER_EXPR div_6 = new OPER_EXPR(new RDDL.INT_CONST_EXPR(1),new RDDL.INT_CONST_EXPR(6),"/");
//        ArrayList<EXPR> discrete_array = new ArrayList<>();
//        for(int i=1 ; i< 7 ; i++){
//            discrete_array.add(new ENUM_VAL("@"+String.valueOf(i)));
//            discrete_array.add(div_6);
//        }
//
//        Discrete dis = new Discrete("small-int",discrete_array);
//
//        RDDL.IF_EXPR if_else_expr = new RDDL.IF_EXPR(roll_p,dis,die_value_p);
//        RandomDataGenerator rand = new RandomDataGenerator();
//        EXPR test_dis_sample = dis.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //EXPR test_1 =( ((OPER_EXPR) ((OPER_EXPR) ((OPER_EXPR)((OPER_EXPR) ((OPER_EXPR) test_dis_sample)._e1)._e1)._e1)._e1)._e1);
//        //test_1.getDoubleValue(constants,objects,hmtypes,hm_variables);
//        EXPR test_2 = new OPER_EXPR(e1,new REAL_CONST_EXPR(2.0),"*");
//        //test_2.isPiecewiseLinear(constants,objects,hmtypes,hm_variables);
//        //test_2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        System.out.println("dkfjkdjfkdjfkjd");
//        EXPR die_sub_expr = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        //die_sub_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.write("testing.lp");
//
//        ///////////////////////////////////////////////////////////        //FOrall Expression
//        //NF2 adding king
//        ENUM_VAL k_1 = new ENUM_VAL("@Blue");
//        ENUM_VAL k_2 = new ENUM_VAL("@Green");
//        ENUM_VAL k_3 = new ENUM_VAL("@Red");
//        ENUM_VAL k_4 = new ENUM_VAL("@Yellow");
//        ENUM_VAL k_5 = new ENUM_VAL("@Purple");
//        ENUM_VAL k_6 = new ENUM_VAL("@Pink");
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        ArrayList<ENUM_VAL> array_enum1 = new ArrayList<>(); array_enum.add(e1);
//        array_enum1.add(k_1); array_enum1.add(k_2); array_enum1.add(k_3); array_enum1.add(k_4);
//
//
//        //First Hmtype
//        TYPE_NAME color_type = new TYPE_NAME("color");
//        TYPE_DEF enum_color_def = new RDDL.ENUM_TYPE_DEF("color", array_enum1);
//        hmtypes.put(color_type,enum_color_def);
//
//
//        LVAR l_col = new LVAR("?c");
//        ArrayList<LVAR> lvars_col_1 = new ArrayList<>();  lvars_col_1.add(l_col);
//        PVAR_EXPR a_to =new PVAR_EXPR("a-to",lvars_col_1);
//        PVAR_EXPR b_to = new PVAR_EXPR("B-TO",lvars_col_1);
//
//        ArrayList<String> co_string = new ArrayList<>();
//        co_string.add("color");
//
//        RDDL.PVARIABLE_STATE_DEF a_to_def = new RDDL.PVARIABLE_STATE_DEF("a-to",false, "bool", co_string, false);
//        hm_variables.put(a_to._pName,a_to_def);
//
//        RDDL.PVARIABLE_STATE_DEF b_to_def = new RDDL.PVARIABLE_STATE_DEF("B-TO",true, "bool", co_string, false);
//        hm_variables.put(b_to._pName,b_to_def);
//
//        Map<ArrayList<LCONST>,Object> cons2 = new HashMap<>();
//        ArrayList<ENUM_VAL> enum_array = new ArrayList<>();
//        enum_array.add(k_1); enum_array.add(k_2);enum_array.add(k_3);enum_array.add(k_4);
//        for(int i = 0; i<enum_array.size() ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(enum_array.get(i)); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons2.put(lconst_array,true);
//        }
//        constants.put(b_to._pName,cons2);
//
//        LTYPED_VAR colr_type = new LTYPED_VAR("?c","color");
//        ArrayList<LTYPED_VAR> array_ltyped1 = new ArrayList<>();     array_ltyped1.add(colr_type);
//
//
//        EXPR f2 = new CONN_EXPR(a_to,b_to,"=>");
//        EXPR f1 = new QUANT_EXPR("forall",array_ltyped1,f2);
//        EXPR p2 = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        ArrayList<BOOL_EXPR> n_list = new ArrayList<>();
//        n_list.add((BOOL_EXPR) p2); n_list.add(new RDDL.BOOL_CONST_EXPR(true));
//        EXPR f4 = new CONN_EXPR(n_list,"=>");
//        //EXPR f3 = new CONN_EXPR(new RDDL.BOOL_CONST_EXPR(true),new RDDL.BOOL_CONST_EXPR(true),"^");
//
//        EXPR s1 =f1.substitute(Collections.EMPTY_MAP,constants,objects,hmtypes,hm_variables);
//
//        ArrayList<EXPR> discrete_array1 = new ArrayList<>();
//        for(int i=0 ; i< array_enum1.size() ; i++){
//            discrete_array1.add(array_enum1.get(i));
//            discrete_array1.add(div_6);
//        }
//
//        Discrete dis1 = new Discrete("color",discrete_array1);
//        EXPR sam1 =dis1.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //sam1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        EXPR c_m_va = new OPER_EXPR(k_1,new RDDL.BOOL_CONST_EXPR(true),"*");
//        //c_m_va.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //Current Phase Code
//        PVAR_EXPR curr_phase =new PVAR_EXPR("Current-Phase",new ArrayList());
////        ArrayList<String> cur_list = new ArrayList<>();
////        cur_list.add("die");
//        RDDL.PVARIABLE_STATE_DEF cur_phase_psd = new RDDL.PVARIABLE_STATE_DEF("Current-Phase",true, "small-int", new ArrayList(), e1);
//        hm_variables.put(curr_phase._pName,cur_phase_psd);
//
//        COMP_EXPR curr_expr = new COMP_EXPR(curr_phase,e1,"==");
//
//        System.out.println("dkjfkdjfkd");
//
//        PVAR_EXPR temp1_pvar = new PVAR_EXPR("temp1",new ArrayList());
//        PVAR_EXPR temp2_pvar = new PVAR_EXPR("temp2",new ArrayList());
//
//        type_map.put(temp1_pvar._pName,GRB.BINARY);
//        type_map.put(temp2_pvar._pName,GRB.BINARY);
//        type_map.put(curr_phase._pName,GRB.INTEGER);
//        RDDL.PVARIABLE_STATE_DEF temp1_psd = new RDDL.PVARIABLE_STATE_DEF("temp1",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp1_pvar._pName,temp1_psd);
//        RDDL.PVARIABLE_STATE_DEF temp2_psd = new RDDL.PVARIABLE_STATE_DEF("temp2",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp2_pvar._pName,temp2_psd);
//
//
//
//
//        CONN_EXPR or_expr =new CONN_EXPR(temp1_pvar, temp2_pvar,"|");
//
//        //z = v1 or v2
//        GRBVar z = curr_expr.getGRBVar(curr_expr, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar or_grbvar =or_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(z,GRB.EQUAL,or_grbvar,EXPR.name_map.get(curr_expr.toString()) +"==" +EXPR.name_map.get(or_expr.toString()) );
//
//
//        GRBVar t1_grb = temp1_pvar.getGRBVar(temp1_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar t2_grb = temp2_pvar.getGRBVar(temp2_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//
//
//
//        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
//        //v1=1 : -ep <= x-y <= 0
//        //v1=0 : -M-ep <= x-y <= 0,
//
//
//        final GRBLinExpr minus_M_one_minus_z = new GRBLinExpr();//-M(1-v1)-ep = -M+Mv1 -ep
//        minus_M_one_minus_z.addConstant(-1d*M );
//        minus_M_one_minus_z.addTerm(1d*M, t1_grb);
//        minus_M_one_minus_z.addConstant(-1d*ep);
//
//        final GRBLinExpr zero_grb = new GRBLinExpr();
//        zero_grb.addConstant(0);
//
//        static_grb_model.addConstr( minus_M_one_minus_z, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, zero_grb, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_2" );
//
//
//        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
//        //v2 =1 : 0 <= x-y <= ep
//        //v2=0 : 0 <= x-y <= M+ep
//
//
//        final GRBLinExpr ex_val  = new GRBLinExpr();//M(1-v2)+ep = M-Mv1+ep
//        ex_val.addConstant(M);
//        ex_val.addTerm(-1d*M, t2_grb);
//        ex_val.addConstant(ep);
//
//
//        static_grb_model.addConstr( zero_grb, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, ex_val, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_2" );
//
//
//
//        //Working on Current-phase = @1, and @1.
//
//        GRBVar cur_expr_grbvar =curr_phase.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        GRBVar enum_val_grbvar = e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(cur_expr_grbvar,GRB.EQUAL,enum_val_grbvar,EXPR.name_map.get(curr_phase.toString())+"=="+EXPR.name_map.get(new RDDL.INT_CONST_EXPR(e1.enum_to_int(hmtypes,hm_variables)).toString()));
//
//
//        System.out.println("dkfkdfjdk");
//
//
//
//
//
//
//
//
//
//
//








        //z = [ x == y ]
        // z = v1 OR v2
        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
        //v1=1 : -ep <= x-y <= 0
        //v1=0 : -M-ep <= x-y <= 0,

        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
        //v2 =1 : 0 <= x-y <= ep
        //v2=0 : 0 <= x-y <= M+ep


    }


    public static void testCaseGurobiIntNotEqual() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());



        //This is checking the case when X1,X2,X3 are integers.

        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.INTEGER);
        type_map.put(p_x2._pName,GRB.INTEGER);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();





        //checking equality "==", X1==X2
        COMP_EXPR x1_eq_x2 = new COMP_EXPR(p_x1,p_x2,"~=");
        COMP_EXPR x1_x2    = new COMP_EXPR(p_x1,p_x2,"==");

        //Adding Default values.
        EXPR def_val1=new RDDL.INT_CONST_EXPR(1);
        EXPR def_val2=new RDDL.INT_CONST_EXPR(1);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));




        GRBVar eq_var =x1_eq_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        GRBLinExpr all_sum = new GRBLinExpr();
        all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        grb_cache.get(x1_eq_x2).get(GRB.DoubleAttr.X);

        System.out.println("KDjfkdjfkd");


//
//        /////////////////////////////////////////////////////////
//        //die-value(?d) == @1
//
//        //LVAR Arraylist ;
//        LVAR l1 = new LVAR("?d");
//        ArrayList<LVAR> lvars_1 = new ArrayList<>();  lvars_1.add(l1);
//
//        //pvar_expr...
//        PVAR_EXPR die_value_p  = new PVAR_EXPR("die-value",lvars_1);
//        PVAR_EXPR nf_pname     = new PVAR_EXPR("NF",lvars_1);
//        PVAR_EXPR roll_p       = new PVAR_EXPR("roll",lvars_1);
//
//        ENUM_VAL e1 = new ENUM_VAL("@1");
//        ENUM_VAL e2 = new ENUM_VAL("@2");
//        ENUM_VAL e3 = new ENUM_VAL("@3");
//        ENUM_VAL e4 = new ENUM_VAL("@4");
//        ENUM_VAL e5 = new ENUM_VAL("@5");
//        ENUM_VAL e6 = new ENUM_VAL("@6");
//
//
//        //Checking ENUM_VAL isConstant()
//        //hmtypes are defined!!!!.
//        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
//        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
//        array_enum.add(e6);
//        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
//
//        //First Hmtype
//        TYPE_NAME s_int = new TYPE_NAME("small-int");
//        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
//        hmtypes.put(s_int,enum_t_def);
//
//        //second hmtype
//        TYPE_NAME die_t = new TYPE_NAME("die");
//        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
//        hmtypes.put(die_t,die_o_def);
//
//
//        //hmvariables are defined!!!!.
//        //First fluent variable
//        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
//        PVAR_NAME die_value = new PVAR_NAME("die-value");
//        ArrayList<String> ar_string = new ArrayList<>();
//        ar_string.add("die");
//        RDDL.PVARIABLE_STATE_DEF p_s_def = new RDDL.PVARIABLE_STATE_DEF("die-value",false, "small-int", ar_string, e1);
//        hm_variables.put(die_value_p._pName,p_s_def);
//        //Second Non-fluent Variable.
//
//        ArrayList<String> ar_string_1 = new ArrayList<>();
//        ar_string_1.add("die");
//        RDDL.PVARIABLE_STATE_DEF nf_s_def = new RDDL.PVARIABLE_STATE_DEF("NF",true, "small-int", ar_string_1, e1);
//        hm_variables.put(nf_pname._pName,nf_s_def);
//
//        //Third Action Fluent.
//        ArrayList<String> ar_string_2 = new ArrayList<>();
//        ar_string_2.add("roll");
//        RDDL.PVARIABLE_ACTION_DEF ac_def = new RDDL.PVARIABLE_ACTION_DEF("roll","bool",ar_string_2,false);
//        hm_variables.put(roll_p._pName,ac_def);
//
//
//
//        //defining objects////////////////////////////////////////
//        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
//        TYPE_NAME die_type = new TYPE_NAME("die");
//        LCONST lconst_d1   = new OBJECT_VAL("$d1");
//        LCONST lconst_d2   = new OBJECT_VAL("$d2");
//        LCONST lconst_d3   = new OBJECT_VAL("$d3");
//        ArrayList<Object> temp_objects = new ArrayList<>();
//        temp_objects.add(lconst_d1); temp_objects.add(lconst_d2); temp_objects.add(lconst_d3);
//        OBJECTS_DEF ob = new OBJECTS_DEF("die",temp_objects);
//        objects.put(die_type,ob);
//        /////////////////////////////////////////////////////////////////
//        //Defining Constants
//        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
//        Map<ArrayList<LCONST>,Object> cons1 = new HashMap<>();
//        for(int i = 1; i<4 ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(new OBJECT_VAL("$d"+Integer.valueOf(i).toString())); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons1.put(lconst_array,new ENUM_VAL("@"+Integer.valueOf(i).toString()));
//        }
//        constants.put(nf_pname._pName,cons1);
//        //////////////////////////////////////////////////////////////////
//
//
//
//        LTYPED_VAR qd1 = new LTYPED_VAR("?d","die");
//        ArrayList<LTYPED_VAR> array_ltyped = new ArrayList<>();     array_ltyped.add(qd1);
//
//
//        COMP_EXPR e_1  = new COMP_EXPR(die_value_p,e1,"==");
//        COMP_EXPR e_2  = new COMP_EXPR(nf_pname,e1,"==");
//        CONN_EXPR e_3  = new CONN_EXPR(e_1,e_2,"|");
//        QUANT_EXPR e_4 = new QUANT_EXPR("forall",array_ltyped,e_3);
//
//        //e_2.substitute(null,null,objects,hmtypes, hm_variables);
//
//        Map<LVAR, LCONST> subs = new HashMap<>();
//        LVAR a_lvar = new LVAR("?d");
//        LCONST a_cont = new OBJECT_VAL("$d1");
//        subs.put(a_lvar,a_cont);
//        EXPR sub_e_2 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_e_2.getDoubleValue(constants,objects,hmtypes,hm_variables,  null);
//        EXPR sub_expr = nf_pname.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_expr.equals(new ENUM_VAL("@1"));
////        EXPR sub_e1 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//
//        //Adding type_map :
//        Map<PVAR_NAME,Character> type_map = new HashMap<>();
//        type_map.put(die_value_p._pName,GRB.INTEGER);
//
//
//        //This Piece of code for testing Discerte Expression.
//
//
//        //GRBVar gvar = sub_e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//
//        //new RDDL.Discrete("small-int")
//
//        //Defining 1/6
//
//        OPER_EXPR div_6 = new OPER_EXPR(new RDDL.INT_CONST_EXPR(1),new RDDL.INT_CONST_EXPR(6),"/");
//        ArrayList<EXPR> discrete_array = new ArrayList<>();
//        for(int i=1 ; i< 7 ; i++){
//            discrete_array.add(new ENUM_VAL("@"+String.valueOf(i)));
//            discrete_array.add(div_6);
//        }
//
//        Discrete dis = new Discrete("small-int",discrete_array);
//
//        RDDL.IF_EXPR if_else_expr = new RDDL.IF_EXPR(roll_p,dis,die_value_p);
//        RandomDataGenerator rand = new RandomDataGenerator();
//        EXPR test_dis_sample = dis.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //EXPR test_1 =( ((OPER_EXPR) ((OPER_EXPR) ((OPER_EXPR)((OPER_EXPR) ((OPER_EXPR) test_dis_sample)._e1)._e1)._e1)._e1)._e1);
//        //test_1.getDoubleValue(constants,objects,hmtypes,hm_variables);
//        EXPR test_2 = new OPER_EXPR(e1,new REAL_CONST_EXPR(2.0),"*");
//        //test_2.isPiecewiseLinear(constants,objects,hmtypes,hm_variables);
//        //test_2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        System.out.println("dkfjkdjfkdjfkjd");
//        EXPR die_sub_expr = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        //die_sub_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.write("testing.lp");
//
//        ///////////////////////////////////////////////////////////        //FOrall Expression
//        //NF2 adding king
//        ENUM_VAL k_1 = new ENUM_VAL("@Blue");
//        ENUM_VAL k_2 = new ENUM_VAL("@Green");
//        ENUM_VAL k_3 = new ENUM_VAL("@Red");
//        ENUM_VAL k_4 = new ENUM_VAL("@Yellow");
//        ENUM_VAL k_5 = new ENUM_VAL("@Purple");
//        ENUM_VAL k_6 = new ENUM_VAL("@Pink");
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        ArrayList<ENUM_VAL> array_enum1 = new ArrayList<>(); array_enum.add(e1);
//        array_enum1.add(k_1); array_enum1.add(k_2); array_enum1.add(k_3); array_enum1.add(k_4);
//
//
//        //First Hmtype
//        TYPE_NAME color_type = new TYPE_NAME("color");
//        TYPE_DEF enum_color_def = new RDDL.ENUM_TYPE_DEF("color", array_enum1);
//        hmtypes.put(color_type,enum_color_def);
//
//
//        LVAR l_col = new LVAR("?c");
//        ArrayList<LVAR> lvars_col_1 = new ArrayList<>();  lvars_col_1.add(l_col);
//        PVAR_EXPR a_to =new PVAR_EXPR("a-to",lvars_col_1);
//        PVAR_EXPR b_to = new PVAR_EXPR("B-TO",lvars_col_1);
//
//        ArrayList<String> co_string = new ArrayList<>();
//        co_string.add("color");
//
//        RDDL.PVARIABLE_STATE_DEF a_to_def = new RDDL.PVARIABLE_STATE_DEF("a-to",false, "bool", co_string, false);
//        hm_variables.put(a_to._pName,a_to_def);
//
//        RDDL.PVARIABLE_STATE_DEF b_to_def = new RDDL.PVARIABLE_STATE_DEF("B-TO",true, "bool", co_string, false);
//        hm_variables.put(b_to._pName,b_to_def);
//
//        Map<ArrayList<LCONST>,Object> cons2 = new HashMap<>();
//        ArrayList<ENUM_VAL> enum_array = new ArrayList<>();
//        enum_array.add(k_1); enum_array.add(k_2);enum_array.add(k_3);enum_array.add(k_4);
//        for(int i = 0; i<enum_array.size() ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(enum_array.get(i)); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons2.put(lconst_array,true);
//        }
//        constants.put(b_to._pName,cons2);
//
//        LTYPED_VAR colr_type = new LTYPED_VAR("?c","color");
//        ArrayList<LTYPED_VAR> array_ltyped1 = new ArrayList<>();     array_ltyped1.add(colr_type);
//
//
//        EXPR f2 = new CONN_EXPR(a_to,b_to,"=>");
//        EXPR f1 = new QUANT_EXPR("forall",array_ltyped1,f2);
//        EXPR p2 = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        ArrayList<BOOL_EXPR> n_list = new ArrayList<>();
//        n_list.add((BOOL_EXPR) p2); n_list.add(new RDDL.BOOL_CONST_EXPR(true));
//        EXPR f4 = new CONN_EXPR(n_list,"=>");
//        //EXPR f3 = new CONN_EXPR(new RDDL.BOOL_CONST_EXPR(true),new RDDL.BOOL_CONST_EXPR(true),"^");
//
//        EXPR s1 =f1.substitute(Collections.EMPTY_MAP,constants,objects,hmtypes,hm_variables);
//
//        ArrayList<EXPR> discrete_array1 = new ArrayList<>();
//        for(int i=0 ; i< array_enum1.size() ; i++){
//            discrete_array1.add(array_enum1.get(i));
//            discrete_array1.add(div_6);
//        }
//
//        Discrete dis1 = new Discrete("color",discrete_array1);
//        EXPR sam1 =dis1.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //sam1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        EXPR c_m_va = new OPER_EXPR(k_1,new RDDL.BOOL_CONST_EXPR(true),"*");
//        //c_m_va.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //Current Phase Code
//        PVAR_EXPR curr_phase =new PVAR_EXPR("Current-Phase",new ArrayList());
////        ArrayList<String> cur_list = new ArrayList<>();
////        cur_list.add("die");
//        RDDL.PVARIABLE_STATE_DEF cur_phase_psd = new RDDL.PVARIABLE_STATE_DEF("Current-Phase",true, "small-int", new ArrayList(), e1);
//        hm_variables.put(curr_phase._pName,cur_phase_psd);
//
//        COMP_EXPR curr_expr = new COMP_EXPR(curr_phase,e1,"==");
//
//        System.out.println("dkjfkdjfkd");
//
//        PVAR_EXPR temp1_pvar = new PVAR_EXPR("temp1",new ArrayList());
//        PVAR_EXPR temp2_pvar = new PVAR_EXPR("temp2",new ArrayList());
//
//        type_map.put(temp1_pvar._pName,GRB.BINARY);
//        type_map.put(temp2_pvar._pName,GRB.BINARY);
//        type_map.put(curr_phase._pName,GRB.INTEGER);
//        RDDL.PVARIABLE_STATE_DEF temp1_psd = new RDDL.PVARIABLE_STATE_DEF("temp1",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp1_pvar._pName,temp1_psd);
//        RDDL.PVARIABLE_STATE_DEF temp2_psd = new RDDL.PVARIABLE_STATE_DEF("temp2",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp2_pvar._pName,temp2_psd);
//
//
//
//
//        CONN_EXPR or_expr =new CONN_EXPR(temp1_pvar, temp2_pvar,"|");
//
//        //z = v1 or v2
//        GRBVar z = curr_expr.getGRBVar(curr_expr, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar or_grbvar =or_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(z,GRB.EQUAL,or_grbvar,EXPR.name_map.get(curr_expr.toString()) +"==" +EXPR.name_map.get(or_expr.toString()) );
//
//
//        GRBVar t1_grb = temp1_pvar.getGRBVar(temp1_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar t2_grb = temp2_pvar.getGRBVar(temp2_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//
//
//
//        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
//        //v1=1 : -ep <= x-y <= 0
//        //v1=0 : -M-ep <= x-y <= 0,
//
//
//        final GRBLinExpr minus_M_one_minus_z = new GRBLinExpr();//-M(1-v1)-ep = -M+Mv1 -ep
//        minus_M_one_minus_z.addConstant(-1d*M );
//        minus_M_one_minus_z.addTerm(1d*M, t1_grb);
//        minus_M_one_minus_z.addConstant(-1d*ep);
//
//        final GRBLinExpr zero_grb = new GRBLinExpr();
//        zero_grb.addConstant(0);
//
//        static_grb_model.addConstr( minus_M_one_minus_z, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, zero_grb, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_2" );
//
//
//        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
//        //v2 =1 : 0 <= x-y <= ep
//        //v2=0 : 0 <= x-y <= M+ep
//
//
//        final GRBLinExpr ex_val  = new GRBLinExpr();//M(1-v2)+ep = M-Mv1+ep
//        ex_val.addConstant(M);
//        ex_val.addTerm(-1d*M, t2_grb);
//        ex_val.addConstant(ep);
//
//
//        static_grb_model.addConstr( zero_grb, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, ex_val, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_2" );
//
//
//
//        //Working on Current-phase = @1, and @1.
//
//        GRBVar cur_expr_grbvar =curr_phase.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        GRBVar enum_val_grbvar = e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(cur_expr_grbvar,GRB.EQUAL,enum_val_grbvar,EXPR.name_map.get(curr_phase.toString())+"=="+EXPR.name_map.get(new RDDL.INT_CONST_EXPR(e1.enum_to_int(hmtypes,hm_variables)).toString()));
//
//
//        System.out.println("dkfkdfjdk");
//
//
//
//
//
//
//
//
//
//
//








        //z = [ x == y ]
        // z = v1 OR v2
        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
        //v1=1 : -ep <= x-y <= 0
        //v1=0 : -M-ep <= x-y <= 0,

        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
        //v2 =1 : 0 <= x-y <= ep
        //v2=0 : 0 <= x-y <= M+ep


    }




    public static void testCaseGurobiRealNotEqual() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());



        //This is checking the case when X1,X2,X3 are integers.

        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.CONTINUOUS);
        type_map.put(p_x2._pName,GRB.CONTINUOUS);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();





        //checking equality "==", X1==X2
        COMP_EXPR x1_eq_x2 = new COMP_EXPR(p_x1,p_x2,"~=");
        COMP_EXPR x1_x2    = new COMP_EXPR(p_x1,p_x2,"==");

        //Adding Default values.
        EXPR def_val1=new RDDL.REAL_CONST_EXPR(1.0);
        EXPR def_val2=new RDDL.REAL_CONST_EXPR(22.0);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));




        GRBVar eq_var =x1_eq_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        GRBLinExpr all_sum = new GRBLinExpr();
        all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        grb_cache.get(x1_eq_x2).get(GRB.DoubleAttr.X);

        System.out.println("KDjfkdjfkd");


//
//        /////////////////////////////////////////////////////////
//        //die-value(?d) == @1
//
//        //LVAR Arraylist ;
//        LVAR l1 = new LVAR("?d");
//        ArrayList<LVAR> lvars_1 = new ArrayList<>();  lvars_1.add(l1);
//
//        //pvar_expr...
//        PVAR_EXPR die_value_p  = new PVAR_EXPR("die-value",lvars_1);
//        PVAR_EXPR nf_pname     = new PVAR_EXPR("NF",lvars_1);
//        PVAR_EXPR roll_p       = new PVAR_EXPR("roll",lvars_1);
//
//        ENUM_VAL e1 = new ENUM_VAL("@1");
//        ENUM_VAL e2 = new ENUM_VAL("@2");
//        ENUM_VAL e3 = new ENUM_VAL("@3");
//        ENUM_VAL e4 = new ENUM_VAL("@4");
//        ENUM_VAL e5 = new ENUM_VAL("@5");
//        ENUM_VAL e6 = new ENUM_VAL("@6");
//
//
//        //Checking ENUM_VAL isConstant()
//        //hmtypes are defined!!!!.
//        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
//        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
//        array_enum.add(e6);
//        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
//
//        //First Hmtype
//        TYPE_NAME s_int = new TYPE_NAME("small-int");
//        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
//        hmtypes.put(s_int,enum_t_def);
//
//        //second hmtype
//        TYPE_NAME die_t = new TYPE_NAME("die");
//        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
//        hmtypes.put(die_t,die_o_def);
//
//
//        //hmvariables are defined!!!!.
//        //First fluent variable
//        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
//        PVAR_NAME die_value = new PVAR_NAME("die-value");
//        ArrayList<String> ar_string = new ArrayList<>();
//        ar_string.add("die");
//        RDDL.PVARIABLE_STATE_DEF p_s_def = new RDDL.PVARIABLE_STATE_DEF("die-value",false, "small-int", ar_string, e1);
//        hm_variables.put(die_value_p._pName,p_s_def);
//        //Second Non-fluent Variable.
//
//        ArrayList<String> ar_string_1 = new ArrayList<>();
//        ar_string_1.add("die");
//        RDDL.PVARIABLE_STATE_DEF nf_s_def = new RDDL.PVARIABLE_STATE_DEF("NF",true, "small-int", ar_string_1, e1);
//        hm_variables.put(nf_pname._pName,nf_s_def);
//
//        //Third Action Fluent.
//        ArrayList<String> ar_string_2 = new ArrayList<>();
//        ar_string_2.add("roll");
//        RDDL.PVARIABLE_ACTION_DEF ac_def = new RDDL.PVARIABLE_ACTION_DEF("roll","bool",ar_string_2,false);
//        hm_variables.put(roll_p._pName,ac_def);
//
//
//
//        //defining objects////////////////////////////////////////
//        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
//        TYPE_NAME die_type = new TYPE_NAME("die");
//        LCONST lconst_d1   = new OBJECT_VAL("$d1");
//        LCONST lconst_d2   = new OBJECT_VAL("$d2");
//        LCONST lconst_d3   = new OBJECT_VAL("$d3");
//        ArrayList<Object> temp_objects = new ArrayList<>();
//        temp_objects.add(lconst_d1); temp_objects.add(lconst_d2); temp_objects.add(lconst_d3);
//        OBJECTS_DEF ob = new OBJECTS_DEF("die",temp_objects);
//        objects.put(die_type,ob);
//        /////////////////////////////////////////////////////////////////
//        //Defining Constants
//        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
//        Map<ArrayList<LCONST>,Object> cons1 = new HashMap<>();
//        for(int i = 1; i<4 ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(new OBJECT_VAL("$d"+Integer.valueOf(i).toString())); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons1.put(lconst_array,new ENUM_VAL("@"+Integer.valueOf(i).toString()));
//        }
//        constants.put(nf_pname._pName,cons1);
//        //////////////////////////////////////////////////////////////////
//
//
//
//        LTYPED_VAR qd1 = new LTYPED_VAR("?d","die");
//        ArrayList<LTYPED_VAR> array_ltyped = new ArrayList<>();     array_ltyped.add(qd1);
//
//
//        COMP_EXPR e_1  = new COMP_EXPR(die_value_p,e1,"==");
//        COMP_EXPR e_2  = new COMP_EXPR(nf_pname,e1,"==");
//        CONN_EXPR e_3  = new CONN_EXPR(e_1,e_2,"|");
//        QUANT_EXPR e_4 = new QUANT_EXPR("forall",array_ltyped,e_3);
//
//        //e_2.substitute(null,null,objects,hmtypes, hm_variables);
//
//        Map<LVAR, LCONST> subs = new HashMap<>();
//        LVAR a_lvar = new LVAR("?d");
//        LCONST a_cont = new OBJECT_VAL("$d1");
//        subs.put(a_lvar,a_cont);
//        EXPR sub_e_2 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_e_2.getDoubleValue(constants,objects,hmtypes,hm_variables,  null);
//        EXPR sub_expr = nf_pname.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_expr.equals(new ENUM_VAL("@1"));
////        EXPR sub_e1 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//
//        //Adding type_map :
//        Map<PVAR_NAME,Character> type_map = new HashMap<>();
//        type_map.put(die_value_p._pName,GRB.INTEGER);
//
//
//        //This Piece of code for testing Discerte Expression.
//
//
//        //GRBVar gvar = sub_e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//
//        //new RDDL.Discrete("small-int")
//
//        //Defining 1/6
//
//        OPER_EXPR div_6 = new OPER_EXPR(new RDDL.INT_CONST_EXPR(1),new RDDL.INT_CONST_EXPR(6),"/");
//        ArrayList<EXPR> discrete_array = new ArrayList<>();
//        for(int i=1 ; i< 7 ; i++){
//            discrete_array.add(new ENUM_VAL("@"+String.valueOf(i)));
//            discrete_array.add(div_6);
//        }
//
//        Discrete dis = new Discrete("small-int",discrete_array);
//
//        RDDL.IF_EXPR if_else_expr = new RDDL.IF_EXPR(roll_p,dis,die_value_p);
//        RandomDataGenerator rand = new RandomDataGenerator();
//        EXPR test_dis_sample = dis.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //EXPR test_1 =( ((OPER_EXPR) ((OPER_EXPR) ((OPER_EXPR)((OPER_EXPR) ((OPER_EXPR) test_dis_sample)._e1)._e1)._e1)._e1)._e1);
//        //test_1.getDoubleValue(constants,objects,hmtypes,hm_variables);
//        EXPR test_2 = new OPER_EXPR(e1,new REAL_CONST_EXPR(2.0),"*");
//        //test_2.isPiecewiseLinear(constants,objects,hmtypes,hm_variables);
//        //test_2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        System.out.println("dkfjkdjfkdjfkjd");
//        EXPR die_sub_expr = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        //die_sub_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.write("testing.lp");
//
//        ///////////////////////////////////////////////////////////        //FOrall Expression
//        //NF2 adding king
//        ENUM_VAL k_1 = new ENUM_VAL("@Blue");
//        ENUM_VAL k_2 = new ENUM_VAL("@Green");
//        ENUM_VAL k_3 = new ENUM_VAL("@Red");
//        ENUM_VAL k_4 = new ENUM_VAL("@Yellow");
//        ENUM_VAL k_5 = new ENUM_VAL("@Purple");
//        ENUM_VAL k_6 = new ENUM_VAL("@Pink");
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        ArrayList<ENUM_VAL> array_enum1 = new ArrayList<>(); array_enum.add(e1);
//        array_enum1.add(k_1); array_enum1.add(k_2); array_enum1.add(k_3); array_enum1.add(k_4);
//
//
//        //First Hmtype
//        TYPE_NAME color_type = new TYPE_NAME("color");
//        TYPE_DEF enum_color_def = new RDDL.ENUM_TYPE_DEF("color", array_enum1);
//        hmtypes.put(color_type,enum_color_def);
//
//
//        LVAR l_col = new LVAR("?c");
//        ArrayList<LVAR> lvars_col_1 = new ArrayList<>();  lvars_col_1.add(l_col);
//        PVAR_EXPR a_to =new PVAR_EXPR("a-to",lvars_col_1);
//        PVAR_EXPR b_to = new PVAR_EXPR("B-TO",lvars_col_1);
//
//        ArrayList<String> co_string = new ArrayList<>();
//        co_string.add("color");
//
//        RDDL.PVARIABLE_STATE_DEF a_to_def = new RDDL.PVARIABLE_STATE_DEF("a-to",false, "bool", co_string, false);
//        hm_variables.put(a_to._pName,a_to_def);
//
//        RDDL.PVARIABLE_STATE_DEF b_to_def = new RDDL.PVARIABLE_STATE_DEF("B-TO",true, "bool", co_string, false);
//        hm_variables.put(b_to._pName,b_to_def);
//
//        Map<ArrayList<LCONST>,Object> cons2 = new HashMap<>();
//        ArrayList<ENUM_VAL> enum_array = new ArrayList<>();
//        enum_array.add(k_1); enum_array.add(k_2);enum_array.add(k_3);enum_array.add(k_4);
//        for(int i = 0; i<enum_array.size() ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(enum_array.get(i)); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons2.put(lconst_array,true);
//        }
//        constants.put(b_to._pName,cons2);
//
//        LTYPED_VAR colr_type = new LTYPED_VAR("?c","color");
//        ArrayList<LTYPED_VAR> array_ltyped1 = new ArrayList<>();     array_ltyped1.add(colr_type);
//
//
//        EXPR f2 = new CONN_EXPR(a_to,b_to,"=>");
//        EXPR f1 = new QUANT_EXPR("forall",array_ltyped1,f2);
//        EXPR p2 = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        ArrayList<BOOL_EXPR> n_list = new ArrayList<>();
//        n_list.add((BOOL_EXPR) p2); n_list.add(new RDDL.BOOL_CONST_EXPR(true));
//        EXPR f4 = new CONN_EXPR(n_list,"=>");
//        //EXPR f3 = new CONN_EXPR(new RDDL.BOOL_CONST_EXPR(true),new RDDL.BOOL_CONST_EXPR(true),"^");
//
//        EXPR s1 =f1.substitute(Collections.EMPTY_MAP,constants,objects,hmtypes,hm_variables);
//
//        ArrayList<EXPR> discrete_array1 = new ArrayList<>();
//        for(int i=0 ; i< array_enum1.size() ; i++){
//            discrete_array1.add(array_enum1.get(i));
//            discrete_array1.add(div_6);
//        }
//
//        Discrete dis1 = new Discrete("color",discrete_array1);
//        EXPR sam1 =dis1.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //sam1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        EXPR c_m_va = new OPER_EXPR(k_1,new RDDL.BOOL_CONST_EXPR(true),"*");
//        //c_m_va.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //Current Phase Code
//        PVAR_EXPR curr_phase =new PVAR_EXPR("Current-Phase",new ArrayList());
////        ArrayList<String> cur_list = new ArrayList<>();
////        cur_list.add("die");
//        RDDL.PVARIABLE_STATE_DEF cur_phase_psd = new RDDL.PVARIABLE_STATE_DEF("Current-Phase",true, "small-int", new ArrayList(), e1);
//        hm_variables.put(curr_phase._pName,cur_phase_psd);
//
//        COMP_EXPR curr_expr = new COMP_EXPR(curr_phase,e1,"==");
//
//        System.out.println("dkjfkdjfkd");
//
//        PVAR_EXPR temp1_pvar = new PVAR_EXPR("temp1",new ArrayList());
//        PVAR_EXPR temp2_pvar = new PVAR_EXPR("temp2",new ArrayList());
//
//        type_map.put(temp1_pvar._pName,GRB.BINARY);
//        type_map.put(temp2_pvar._pName,GRB.BINARY);
//        type_map.put(curr_phase._pName,GRB.INTEGER);
//        RDDL.PVARIABLE_STATE_DEF temp1_psd = new RDDL.PVARIABLE_STATE_DEF("temp1",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp1_pvar._pName,temp1_psd);
//        RDDL.PVARIABLE_STATE_DEF temp2_psd = new RDDL.PVARIABLE_STATE_DEF("temp2",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp2_pvar._pName,temp2_psd);
//
//
//
//
//        CONN_EXPR or_expr =new CONN_EXPR(temp1_pvar, temp2_pvar,"|");
//
//        //z = v1 or v2
//        GRBVar z = curr_expr.getGRBVar(curr_expr, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar or_grbvar =or_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(z,GRB.EQUAL,or_grbvar,EXPR.name_map.get(curr_expr.toString()) +"==" +EXPR.name_map.get(or_expr.toString()) );
//
//
//        GRBVar t1_grb = temp1_pvar.getGRBVar(temp1_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar t2_grb = temp2_pvar.getGRBVar(temp2_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//
//
//
//        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
//        //v1=1 : -ep <= x-y <= 0
//        //v1=0 : -M-ep <= x-y <= 0,
//
//
//        final GRBLinExpr minus_M_one_minus_z = new GRBLinExpr();//-M(1-v1)-ep = -M+Mv1 -ep
//        minus_M_one_minus_z.addConstant(-1d*M );
//        minus_M_one_minus_z.addTerm(1d*M, t1_grb);
//        minus_M_one_minus_z.addConstant(-1d*ep);
//
//        final GRBLinExpr zero_grb = new GRBLinExpr();
//        zero_grb.addConstant(0);
//
//        static_grb_model.addConstr( minus_M_one_minus_z, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, zero_grb, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_2" );
//
//
//        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
//        //v2 =1 : 0 <= x-y <= ep
//        //v2=0 : 0 <= x-y <= M+ep
//
//
//        final GRBLinExpr ex_val  = new GRBLinExpr();//M(1-v2)+ep = M-Mv1+ep
//        ex_val.addConstant(M);
//        ex_val.addTerm(-1d*M, t2_grb);
//        ex_val.addConstant(ep);
//
//
//        static_grb_model.addConstr( zero_grb, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, ex_val, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_2" );
//
//
//
//        //Working on Current-phase = @1, and @1.
//
//        GRBVar cur_expr_grbvar =curr_phase.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        GRBVar enum_val_grbvar = e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(cur_expr_grbvar,GRB.EQUAL,enum_val_grbvar,EXPR.name_map.get(curr_phase.toString())+"=="+EXPR.name_map.get(new RDDL.INT_CONST_EXPR(e1.enum_to_int(hmtypes,hm_variables)).toString()));
//
//
//        System.out.println("dkfkdfjdk");
//
//
//
//
//
//
//
//
//
//
//








        //z = [ x == y ]
        // z = v1 OR v2
        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
        //v1=1 : -ep <= x-y <= 0
        //v1=0 : -M-ep <= x-y <= 0,

        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
        //v2 =1 : 0 <= x-y <= ep
        //v2=0 : 0 <= x-y <= M+ep


    }


    public static void testCaseGurobiIntGreat() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());



        //This is checking the case when X1,X2,X3 are integers.

        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.INTEGER);
        type_map.put(p_x2._pName,GRB.INTEGER);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();





        //checking equality "==", X1==X2
        COMP_EXPR x1_comp_x2 = new COMP_EXPR(p_x1,p_x2,">");
        COMP_EXPR x1_x2    = new COMP_EXPR(p_x1,p_x2,"==");

        //Adding Default values.
        EXPR def_val1=new RDDL.INT_CONST_EXPR(1);
        EXPR def_val2=new RDDL.INT_CONST_EXPR(1);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));




        GRBVar eq_var =x1_comp_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        grb_cache.get(x1_comp_x2).get(GRB.DoubleAttr.X);

        System.out.println("KDjfkdjfkd");



//
//        /////////////////////////////////////////////////////////
//        //die-value(?d) == @1
//
//        //LVAR Arraylist ;
//        LVAR l1 = new LVAR("?d");
//        ArrayList<LVAR> lvars_1 = new ArrayList<>();  lvars_1.add(l1);
//
//        //pvar_expr...
//        PVAR_EXPR die_value_p  = new PVAR_EXPR("die-value",lvars_1);
//        PVAR_EXPR nf_pname     = new PVAR_EXPR("NF",lvars_1);
//        PVAR_EXPR roll_p       = new PVAR_EXPR("roll",lvars_1);
//
//        ENUM_VAL e1 = new ENUM_VAL("@1");
//        ENUM_VAL e2 = new ENUM_VAL("@2");
//        ENUM_VAL e3 = new ENUM_VAL("@3");
//        ENUM_VAL e4 = new ENUM_VAL("@4");
//        ENUM_VAL e5 = new ENUM_VAL("@5");
//        ENUM_VAL e6 = new ENUM_VAL("@6");
//
//
//        //Checking ENUM_VAL isConstant()
//        //hmtypes are defined!!!!.
//        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
//        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
//        array_enum.add(e6);
//        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
//
//        //First Hmtype
//        TYPE_NAME s_int = new TYPE_NAME("small-int");
//        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
//        hmtypes.put(s_int,enum_t_def);
//
//        //second hmtype
//        TYPE_NAME die_t = new TYPE_NAME("die");
//        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
//        hmtypes.put(die_t,die_o_def);
//
//
//        //hmvariables are defined!!!!.
//        //First fluent variable
//        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
//        PVAR_NAME die_value = new PVAR_NAME("die-value");
//        ArrayList<String> ar_string = new ArrayList<>();
//        ar_string.add("die");
//        RDDL.PVARIABLE_STATE_DEF p_s_def = new RDDL.PVARIABLE_STATE_DEF("die-value",false, "small-int", ar_string, e1);
//        hm_variables.put(die_value_p._pName,p_s_def);
//        //Second Non-fluent Variable.
//
//        ArrayList<String> ar_string_1 = new ArrayList<>();
//        ar_string_1.add("die");
//        RDDL.PVARIABLE_STATE_DEF nf_s_def = new RDDL.PVARIABLE_STATE_DEF("NF",true, "small-int", ar_string_1, e1);
//        hm_variables.put(nf_pname._pName,nf_s_def);
//
//        //Third Action Fluent.
//        ArrayList<String> ar_string_2 = new ArrayList<>();
//        ar_string_2.add("roll");
//        RDDL.PVARIABLE_ACTION_DEF ac_def = new RDDL.PVARIABLE_ACTION_DEF("roll","bool",ar_string_2,false);
//        hm_variables.put(roll_p._pName,ac_def);
//
//
//
//        //defining objects////////////////////////////////////////
//        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
//        TYPE_NAME die_type = new TYPE_NAME("die");
//        LCONST lconst_d1   = new OBJECT_VAL("$d1");
//        LCONST lconst_d2   = new OBJECT_VAL("$d2");
//        LCONST lconst_d3   = new OBJECT_VAL("$d3");
//        ArrayList<Object> temp_objects = new ArrayList<>();
//        temp_objects.add(lconst_d1); temp_objects.add(lconst_d2); temp_objects.add(lconst_d3);
//        OBJECTS_DEF ob = new OBJECTS_DEF("die",temp_objects);
//        objects.put(die_type,ob);
//        /////////////////////////////////////////////////////////////////
//        //Defining Constants
//        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
//        Map<ArrayList<LCONST>,Object> cons1 = new HashMap<>();
//        for(int i = 1; i<4 ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(new OBJECT_VAL("$d"+Integer.valueOf(i).toString())); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons1.put(lconst_array,new ENUM_VAL("@"+Integer.valueOf(i).toString()));
//        }
//        constants.put(nf_pname._pName,cons1);
//        //////////////////////////////////////////////////////////////////
//
//
//
//        LTYPED_VAR qd1 = new LTYPED_VAR("?d","die");
//        ArrayList<LTYPED_VAR> array_ltyped = new ArrayList<>();     array_ltyped.add(qd1);
//
//
//        COMP_EXPR e_1  = new COMP_EXPR(die_value_p,e1,"==");
//        COMP_EXPR e_2  = new COMP_EXPR(nf_pname,e1,"==");
//        CONN_EXPR e_3  = new CONN_EXPR(e_1,e_2,"|");
//        QUANT_EXPR e_4 = new QUANT_EXPR("forall",array_ltyped,e_3);
//
//        //e_2.substitute(null,null,objects,hmtypes, hm_variables);
//
//        Map<LVAR, LCONST> subs = new HashMap<>();
//        LVAR a_lvar = new LVAR("?d");
//        LCONST a_cont = new OBJECT_VAL("$d1");
//        subs.put(a_lvar,a_cont);
//        EXPR sub_e_2 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_e_2.getDoubleValue(constants,objects,hmtypes,hm_variables,  null);
//        EXPR sub_expr = nf_pname.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_expr.equals(new ENUM_VAL("@1"));
////        EXPR sub_e1 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//
//        //Adding type_map :
//        Map<PVAR_NAME,Character> type_map = new HashMap<>();
//        type_map.put(die_value_p._pName,GRB.INTEGER);
//
//
//        //This Piece of code for testing Discerte Expression.
//
//
//        //GRBVar gvar = sub_e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//
//        //new RDDL.Discrete("small-int")
//
//        //Defining 1/6
//
//        OPER_EXPR div_6 = new OPER_EXPR(new RDDL.INT_CONST_EXPR(1),new RDDL.INT_CONST_EXPR(6),"/");
//        ArrayList<EXPR> discrete_array = new ArrayList<>();
//        for(int i=1 ; i< 7 ; i++){
//            discrete_array.add(new ENUM_VAL("@"+String.valueOf(i)));
//            discrete_array.add(div_6);
//        }
//
//        Discrete dis = new Discrete("small-int",discrete_array);
//
//        RDDL.IF_EXPR if_else_expr = new RDDL.IF_EXPR(roll_p,dis,die_value_p);
//        RandomDataGenerator rand = new RandomDataGenerator();
//        EXPR test_dis_sample = dis.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //EXPR test_1 =( ((OPER_EXPR) ((OPER_EXPR) ((OPER_EXPR)((OPER_EXPR) ((OPER_EXPR) test_dis_sample)._e1)._e1)._e1)._e1)._e1);
//        //test_1.getDoubleValue(constants,objects,hmtypes,hm_variables);
//        EXPR test_2 = new OPER_EXPR(e1,new REAL_CONST_EXPR(2.0),"*");
//        //test_2.isPiecewiseLinear(constants,objects,hmtypes,hm_variables);
//        //test_2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        System.out.println("dkfjkdjfkdjfkjd");
//        EXPR die_sub_expr = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        //die_sub_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.write("testing.lp");
//
//        ///////////////////////////////////////////////////////////        //FOrall Expression
//        //NF2 adding king
//        ENUM_VAL k_1 = new ENUM_VAL("@Blue");
//        ENUM_VAL k_2 = new ENUM_VAL("@Green");
//        ENUM_VAL k_3 = new ENUM_VAL("@Red");
//        ENUM_VAL k_4 = new ENUM_VAL("@Yellow");
//        ENUM_VAL k_5 = new ENUM_VAL("@Purple");
//        ENUM_VAL k_6 = new ENUM_VAL("@Pink");
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        ArrayList<ENUM_VAL> array_enum1 = new ArrayList<>(); array_enum.add(e1);
//        array_enum1.add(k_1); array_enum1.add(k_2); array_enum1.add(k_3); array_enum1.add(k_4);
//
//
//        //First Hmtype
//        TYPE_NAME color_type = new TYPE_NAME("color");
//        TYPE_DEF enum_color_def = new RDDL.ENUM_TYPE_DEF("color", array_enum1);
//        hmtypes.put(color_type,enum_color_def);
//
//
//        LVAR l_col = new LVAR("?c");
//        ArrayList<LVAR> lvars_col_1 = new ArrayList<>();  lvars_col_1.add(l_col);
//        PVAR_EXPR a_to =new PVAR_EXPR("a-to",lvars_col_1);
//        PVAR_EXPR b_to = new PVAR_EXPR("B-TO",lvars_col_1);
//
//        ArrayList<String> co_string = new ArrayList<>();
//        co_string.add("color");
//
//        RDDL.PVARIABLE_STATE_DEF a_to_def = new RDDL.PVARIABLE_STATE_DEF("a-to",false, "bool", co_string, false);
//        hm_variables.put(a_to._pName,a_to_def);
//
//        RDDL.PVARIABLE_STATE_DEF b_to_def = new RDDL.PVARIABLE_STATE_DEF("B-TO",true, "bool", co_string, false);
//        hm_variables.put(b_to._pName,b_to_def);
//
//        Map<ArrayList<LCONST>,Object> cons2 = new HashMap<>();
//        ArrayList<ENUM_VAL> enum_array = new ArrayList<>();
//        enum_array.add(k_1); enum_array.add(k_2);enum_array.add(k_3);enum_array.add(k_4);
//        for(int i = 0; i<enum_array.size() ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(enum_array.get(i)); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons2.put(lconst_array,true);
//        }
//        constants.put(b_to._pName,cons2);
//
//        LTYPED_VAR colr_type = new LTYPED_VAR("?c","color");
//        ArrayList<LTYPED_VAR> array_ltyped1 = new ArrayList<>();     array_ltyped1.add(colr_type);
//
//
//        EXPR f2 = new CONN_EXPR(a_to,b_to,"=>");
//        EXPR f1 = new QUANT_EXPR("forall",array_ltyped1,f2);
//        EXPR p2 = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        ArrayList<BOOL_EXPR> n_list = new ArrayList<>();
//        n_list.add((BOOL_EXPR) p2); n_list.add(new RDDL.BOOL_CONST_EXPR(true));
//        EXPR f4 = new CONN_EXPR(n_list,"=>");
//        //EXPR f3 = new CONN_EXPR(new RDDL.BOOL_CONST_EXPR(true),new RDDL.BOOL_CONST_EXPR(true),"^");
//
//        EXPR s1 =f1.substitute(Collections.EMPTY_MAP,constants,objects,hmtypes,hm_variables);
//
//        ArrayList<EXPR> discrete_array1 = new ArrayList<>();
//        for(int i=0 ; i< array_enum1.size() ; i++){
//            discrete_array1.add(array_enum1.get(i));
//            discrete_array1.add(div_6);
//        }
//
//        Discrete dis1 = new Discrete("color",discrete_array1);
//        EXPR sam1 =dis1.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //sam1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        EXPR c_m_va = new OPER_EXPR(k_1,new RDDL.BOOL_CONST_EXPR(true),"*");
//        //c_m_va.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //Current Phase Code
//        PVAR_EXPR curr_phase =new PVAR_EXPR("Current-Phase",new ArrayList());
////        ArrayList<String> cur_list = new ArrayList<>();
////        cur_list.add("die");
//        RDDL.PVARIABLE_STATE_DEF cur_phase_psd = new RDDL.PVARIABLE_STATE_DEF("Current-Phase",true, "small-int", new ArrayList(), e1);
//        hm_variables.put(curr_phase._pName,cur_phase_psd);
//
//        COMP_EXPR curr_expr = new COMP_EXPR(curr_phase,e1,"==");
//
//        System.out.println("dkjfkdjfkd");
//
//        PVAR_EXPR temp1_pvar = new PVAR_EXPR("temp1",new ArrayList());
//        PVAR_EXPR temp2_pvar = new PVAR_EXPR("temp2",new ArrayList());
//
//        type_map.put(temp1_pvar._pName,GRB.BINARY);
//        type_map.put(temp2_pvar._pName,GRB.BINARY);
//        type_map.put(curr_phase._pName,GRB.INTEGER);
//        RDDL.PVARIABLE_STATE_DEF temp1_psd = new RDDL.PVARIABLE_STATE_DEF("temp1",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp1_pvar._pName,temp1_psd);
//        RDDL.PVARIABLE_STATE_DEF temp2_psd = new RDDL.PVARIABLE_STATE_DEF("temp2",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp2_pvar._pName,temp2_psd);
//
//
//
//
//        CONN_EXPR or_expr =new CONN_EXPR(temp1_pvar, temp2_pvar,"|");
//
//        //z = v1 or v2
//        GRBVar z = curr_expr.getGRBVar(curr_expr, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar or_grbvar =or_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(z,GRB.EQUAL,or_grbvar,EXPR.name_map.get(curr_expr.toString()) +"==" +EXPR.name_map.get(or_expr.toString()) );
//
//
//        GRBVar t1_grb = temp1_pvar.getGRBVar(temp1_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar t2_grb = temp2_pvar.getGRBVar(temp2_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//
//
//
//        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
//        //v1=1 : -ep <= x-y <= 0
//        //v1=0 : -M-ep <= x-y <= 0,
//
//
//        final GRBLinExpr minus_M_one_minus_z = new GRBLinExpr();//-M(1-v1)-ep = -M+Mv1 -ep
//        minus_M_one_minus_z.addConstant(-1d*M );
//        minus_M_one_minus_z.addTerm(1d*M, t1_grb);
//        minus_M_one_minus_z.addConstant(-1d*ep);
//
//        final GRBLinExpr zero_grb = new GRBLinExpr();
//        zero_grb.addConstant(0);
//
//        static_grb_model.addConstr( minus_M_one_minus_z, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, zero_grb, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_2" );
//
//
//        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
//        //v2 =1 : 0 <= x-y <= ep
//        //v2=0 : 0 <= x-y <= M+ep
//
//
//        final GRBLinExpr ex_val  = new GRBLinExpr();//M(1-v2)+ep = M-Mv1+ep
//        ex_val.addConstant(M);
//        ex_val.addTerm(-1d*M, t2_grb);
//        ex_val.addConstant(ep);
//
//
//        static_grb_model.addConstr( zero_grb, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, ex_val, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_2" );
//
//
//
//        //Working on Current-phase = @1, and @1.
//
//        GRBVar cur_expr_grbvar =curr_phase.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        GRBVar enum_val_grbvar = e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(cur_expr_grbvar,GRB.EQUAL,enum_val_grbvar,EXPR.name_map.get(curr_phase.toString())+"=="+EXPR.name_map.get(new RDDL.INT_CONST_EXPR(e1.enum_to_int(hmtypes,hm_variables)).toString()));
//
//
//        System.out.println("dkfkdfjdk");
//
//
//
//
//
//
//
//
//
//
//








        //z = [ x == y ]
        // z = v1 OR v2
        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
        //v1=1 : -ep <= x-y <= 0
        //v1=0 : -M-ep <= x-y <= 0,

        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
        //v2 =1 : 0 <= x-y <= ep
        //v2=0 : 0 <= x-y <= M+ep


    }

    public static void testCaseGurobiConGreat() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());



        //This is checking the case when X1,X2,X3 are integers.

        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.CONTINUOUS);
        type_map.put(p_x2._pName,GRB.CONTINUOUS);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();





        //checking equality "==", X1==X2
        COMP_EXPR x1_comp_x2 = new COMP_EXPR(p_x1,p_x2,">");
        COMP_EXPR x1_x2    = new COMP_EXPR(p_x1,p_x2,"==");

        //Adding Default values.
        EXPR def_val1=new RDDL.REAL_CONST_EXPR(1.0);
        EXPR def_val2=new RDDL.REAL_CONST_EXPR(2.0);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));




        GRBVar eq_var =x1_comp_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        grb_cache.get(x1_comp_x2).get(GRB.DoubleAttr.X);

        System.out.println("KDjfkdjfkd");



//
//        /////////////////////////////////////////////////////////
//        //die-value(?d) == @1
//
//        //LVAR Arraylist ;
//        LVAR l1 = new LVAR("?d");
//        ArrayList<LVAR> lvars_1 = new ArrayList<>();  lvars_1.add(l1);
//
//        //pvar_expr...
//        PVAR_EXPR die_value_p  = new PVAR_EXPR("die-value",lvars_1);
//        PVAR_EXPR nf_pname     = new PVAR_EXPR("NF",lvars_1);
//        PVAR_EXPR roll_p       = new PVAR_EXPR("roll",lvars_1);
//
//        ENUM_VAL e1 = new ENUM_VAL("@1");
//        ENUM_VAL e2 = new ENUM_VAL("@2");
//        ENUM_VAL e3 = new ENUM_VAL("@3");
//        ENUM_VAL e4 = new ENUM_VAL("@4");
//        ENUM_VAL e5 = new ENUM_VAL("@5");
//        ENUM_VAL e6 = new ENUM_VAL("@6");
//
//
//        //Checking ENUM_VAL isConstant()
//        //hmtypes are defined!!!!.
//        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
//        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
//        array_enum.add(e6);
//        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
//
//        //First Hmtype
//        TYPE_NAME s_int = new TYPE_NAME("small-int");
//        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
//        hmtypes.put(s_int,enum_t_def);
//
//        //second hmtype
//        TYPE_NAME die_t = new TYPE_NAME("die");
//        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
//        hmtypes.put(die_t,die_o_def);
//
//
//        //hmvariables are defined!!!!.
//        //First fluent variable
//        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
//        PVAR_NAME die_value = new PVAR_NAME("die-value");
//        ArrayList<String> ar_string = new ArrayList<>();
//        ar_string.add("die");
//        RDDL.PVARIABLE_STATE_DEF p_s_def = new RDDL.PVARIABLE_STATE_DEF("die-value",false, "small-int", ar_string, e1);
//        hm_variables.put(die_value_p._pName,p_s_def);
//        //Second Non-fluent Variable.
//
//        ArrayList<String> ar_string_1 = new ArrayList<>();
//        ar_string_1.add("die");
//        RDDL.PVARIABLE_STATE_DEF nf_s_def = new RDDL.PVARIABLE_STATE_DEF("NF",true, "small-int", ar_string_1, e1);
//        hm_variables.put(nf_pname._pName,nf_s_def);
//
//        //Third Action Fluent.
//        ArrayList<String> ar_string_2 = new ArrayList<>();
//        ar_string_2.add("roll");
//        RDDL.PVARIABLE_ACTION_DEF ac_def = new RDDL.PVARIABLE_ACTION_DEF("roll","bool",ar_string_2,false);
//        hm_variables.put(roll_p._pName,ac_def);
//
//
//
//        //defining objects////////////////////////////////////////
//        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
//        TYPE_NAME die_type = new TYPE_NAME("die");
//        LCONST lconst_d1   = new OBJECT_VAL("$d1");
//        LCONST lconst_d2   = new OBJECT_VAL("$d2");
//        LCONST lconst_d3   = new OBJECT_VAL("$d3");
//        ArrayList<Object> temp_objects = new ArrayList<>();
//        temp_objects.add(lconst_d1); temp_objects.add(lconst_d2); temp_objects.add(lconst_d3);
//        OBJECTS_DEF ob = new OBJECTS_DEF("die",temp_objects);
//        objects.put(die_type,ob);
//        /////////////////////////////////////////////////////////////////
//        //Defining Constants
//        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
//        Map<ArrayList<LCONST>,Object> cons1 = new HashMap<>();
//        for(int i = 1; i<4 ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(new OBJECT_VAL("$d"+Integer.valueOf(i).toString())); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons1.put(lconst_array,new ENUM_VAL("@"+Integer.valueOf(i).toString()));
//        }
//        constants.put(nf_pname._pName,cons1);
//        //////////////////////////////////////////////////////////////////
//
//
//
//        LTYPED_VAR qd1 = new LTYPED_VAR("?d","die");
//        ArrayList<LTYPED_VAR> array_ltyped = new ArrayList<>();     array_ltyped.add(qd1);
//
//
//        COMP_EXPR e_1  = new COMP_EXPR(die_value_p,e1,"==");
//        COMP_EXPR e_2  = new COMP_EXPR(nf_pname,e1,"==");
//        CONN_EXPR e_3  = new CONN_EXPR(e_1,e_2,"|");
//        QUANT_EXPR e_4 = new QUANT_EXPR("forall",array_ltyped,e_3);
//
//        //e_2.substitute(null,null,objects,hmtypes, hm_variables);
//
//        Map<LVAR, LCONST> subs = new HashMap<>();
//        LVAR a_lvar = new LVAR("?d");
//        LCONST a_cont = new OBJECT_VAL("$d1");
//        subs.put(a_lvar,a_cont);
//        EXPR sub_e_2 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_e_2.getDoubleValue(constants,objects,hmtypes,hm_variables,  null);
//        EXPR sub_expr = nf_pname.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_expr.equals(new ENUM_VAL("@1"));
////        EXPR sub_e1 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//
//        //Adding type_map :
//        Map<PVAR_NAME,Character> type_map = new HashMap<>();
//        type_map.put(die_value_p._pName,GRB.INTEGER);
//
//
//        //This Piece of code for testing Discerte Expression.
//
//
//        //GRBVar gvar = sub_e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//
//        //new RDDL.Discrete("small-int")
//
//        //Defining 1/6
//
//        OPER_EXPR div_6 = new OPER_EXPR(new RDDL.INT_CONST_EXPR(1),new RDDL.INT_CONST_EXPR(6),"/");
//        ArrayList<EXPR> discrete_array = new ArrayList<>();
//        for(int i=1 ; i< 7 ; i++){
//            discrete_array.add(new ENUM_VAL("@"+String.valueOf(i)));
//            discrete_array.add(div_6);
//        }
//
//        Discrete dis = new Discrete("small-int",discrete_array);
//
//        RDDL.IF_EXPR if_else_expr = new RDDL.IF_EXPR(roll_p,dis,die_value_p);
//        RandomDataGenerator rand = new RandomDataGenerator();
//        EXPR test_dis_sample = dis.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //EXPR test_1 =( ((OPER_EXPR) ((OPER_EXPR) ((OPER_EXPR)((OPER_EXPR) ((OPER_EXPR) test_dis_sample)._e1)._e1)._e1)._e1)._e1);
//        //test_1.getDoubleValue(constants,objects,hmtypes,hm_variables);
//        EXPR test_2 = new OPER_EXPR(e1,new REAL_CONST_EXPR(2.0),"*");
//        //test_2.isPiecewiseLinear(constants,objects,hmtypes,hm_variables);
//        //test_2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        System.out.println("dkfjkdjfkdjfkjd");
//        EXPR die_sub_expr = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        //die_sub_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.write("testing.lp");
//
//        ///////////////////////////////////////////////////////////        //FOrall Expression
//        //NF2 adding king
//        ENUM_VAL k_1 = new ENUM_VAL("@Blue");
//        ENUM_VAL k_2 = new ENUM_VAL("@Green");
//        ENUM_VAL k_3 = new ENUM_VAL("@Red");
//        ENUM_VAL k_4 = new ENUM_VAL("@Yellow");
//        ENUM_VAL k_5 = new ENUM_VAL("@Purple");
//        ENUM_VAL k_6 = new ENUM_VAL("@Pink");
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        ArrayList<ENUM_VAL> array_enum1 = new ArrayList<>(); array_enum.add(e1);
//        array_enum1.add(k_1); array_enum1.add(k_2); array_enum1.add(k_3); array_enum1.add(k_4);
//
//
//        //First Hmtype
//        TYPE_NAME color_type = new TYPE_NAME("color");
//        TYPE_DEF enum_color_def = new RDDL.ENUM_TYPE_DEF("color", array_enum1);
//        hmtypes.put(color_type,enum_color_def);
//
//
//        LVAR l_col = new LVAR("?c");
//        ArrayList<LVAR> lvars_col_1 = new ArrayList<>();  lvars_col_1.add(l_col);
//        PVAR_EXPR a_to =new PVAR_EXPR("a-to",lvars_col_1);
//        PVAR_EXPR b_to = new PVAR_EXPR("B-TO",lvars_col_1);
//
//        ArrayList<String> co_string = new ArrayList<>();
//        co_string.add("color");
//
//        RDDL.PVARIABLE_STATE_DEF a_to_def = new RDDL.PVARIABLE_STATE_DEF("a-to",false, "bool", co_string, false);
//        hm_variables.put(a_to._pName,a_to_def);
//
//        RDDL.PVARIABLE_STATE_DEF b_to_def = new RDDL.PVARIABLE_STATE_DEF("B-TO",true, "bool", co_string, false);
//        hm_variables.put(b_to._pName,b_to_def);
//
//        Map<ArrayList<LCONST>,Object> cons2 = new HashMap<>();
//        ArrayList<ENUM_VAL> enum_array = new ArrayList<>();
//        enum_array.add(k_1); enum_array.add(k_2);enum_array.add(k_3);enum_array.add(k_4);
//        for(int i = 0; i<enum_array.size() ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(enum_array.get(i)); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons2.put(lconst_array,true);
//        }
//        constants.put(b_to._pName,cons2);
//
//        LTYPED_VAR colr_type = new LTYPED_VAR("?c","color");
//        ArrayList<LTYPED_VAR> array_ltyped1 = new ArrayList<>();     array_ltyped1.add(colr_type);
//
//
//        EXPR f2 = new CONN_EXPR(a_to,b_to,"=>");
//        EXPR f1 = new QUANT_EXPR("forall",array_ltyped1,f2);
//        EXPR p2 = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        ArrayList<BOOL_EXPR> n_list = new ArrayList<>();
//        n_list.add((BOOL_EXPR) p2); n_list.add(new RDDL.BOOL_CONST_EXPR(true));
//        EXPR f4 = new CONN_EXPR(n_list,"=>");
//        //EXPR f3 = new CONN_EXPR(new RDDL.BOOL_CONST_EXPR(true),new RDDL.BOOL_CONST_EXPR(true),"^");
//
//        EXPR s1 =f1.substitute(Collections.EMPTY_MAP,constants,objects,hmtypes,hm_variables);
//
//        ArrayList<EXPR> discrete_array1 = new ArrayList<>();
//        for(int i=0 ; i< array_enum1.size() ; i++){
//            discrete_array1.add(array_enum1.get(i));
//            discrete_array1.add(div_6);
//        }
//
//        Discrete dis1 = new Discrete("color",discrete_array1);
//        EXPR sam1 =dis1.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //sam1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        EXPR c_m_va = new OPER_EXPR(k_1,new RDDL.BOOL_CONST_EXPR(true),"*");
//        //c_m_va.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //Current Phase Code
//        PVAR_EXPR curr_phase =new PVAR_EXPR("Current-Phase",new ArrayList());
////        ArrayList<String> cur_list = new ArrayList<>();
////        cur_list.add("die");
//        RDDL.PVARIABLE_STATE_DEF cur_phase_psd = new RDDL.PVARIABLE_STATE_DEF("Current-Phase",true, "small-int", new ArrayList(), e1);
//        hm_variables.put(curr_phase._pName,cur_phase_psd);
//
//        COMP_EXPR curr_expr = new COMP_EXPR(curr_phase,e1,"==");
//
//        System.out.println("dkjfkdjfkd");
//
//        PVAR_EXPR temp1_pvar = new PVAR_EXPR("temp1",new ArrayList());
//        PVAR_EXPR temp2_pvar = new PVAR_EXPR("temp2",new ArrayList());
//
//        type_map.put(temp1_pvar._pName,GRB.BINARY);
//        type_map.put(temp2_pvar._pName,GRB.BINARY);
//        type_map.put(curr_phase._pName,GRB.INTEGER);
//        RDDL.PVARIABLE_STATE_DEF temp1_psd = new RDDL.PVARIABLE_STATE_DEF("temp1",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp1_pvar._pName,temp1_psd);
//        RDDL.PVARIABLE_STATE_DEF temp2_psd = new RDDL.PVARIABLE_STATE_DEF("temp2",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp2_pvar._pName,temp2_psd);
//
//
//
//
//        CONN_EXPR or_expr =new CONN_EXPR(temp1_pvar, temp2_pvar,"|");
//
//        //z = v1 or v2
//        GRBVar z = curr_expr.getGRBVar(curr_expr, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar or_grbvar =or_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(z,GRB.EQUAL,or_grbvar,EXPR.name_map.get(curr_expr.toString()) +"==" +EXPR.name_map.get(or_expr.toString()) );
//
//
//        GRBVar t1_grb = temp1_pvar.getGRBVar(temp1_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar t2_grb = temp2_pvar.getGRBVar(temp2_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//
//
//
//        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
//        //v1=1 : -ep <= x-y <= 0
//        //v1=0 : -M-ep <= x-y <= 0,
//
//
//        final GRBLinExpr minus_M_one_minus_z = new GRBLinExpr();//-M(1-v1)-ep = -M+Mv1 -ep
//        minus_M_one_minus_z.addConstant(-1d*M );
//        minus_M_one_minus_z.addTerm(1d*M, t1_grb);
//        minus_M_one_minus_z.addConstant(-1d*ep);
//
//        final GRBLinExpr zero_grb = new GRBLinExpr();
//        zero_grb.addConstant(0);
//
//        static_grb_model.addConstr( minus_M_one_minus_z, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, zero_grb, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_2" );
//
//
//        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
//        //v2 =1 : 0 <= x-y <= ep
//        //v2=0 : 0 <= x-y <= M+ep
//
//
//        final GRBLinExpr ex_val  = new GRBLinExpr();//M(1-v2)+ep = M-Mv1+ep
//        ex_val.addConstant(M);
//        ex_val.addTerm(-1d*M, t2_grb);
//        ex_val.addConstant(ep);
//
//
//        static_grb_model.addConstr( zero_grb, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, ex_val, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_2" );
//
//
//
//        //Working on Current-phase = @1, and @1.
//
//        GRBVar cur_expr_grbvar =curr_phase.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        GRBVar enum_val_grbvar = e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(cur_expr_grbvar,GRB.EQUAL,enum_val_grbvar,EXPR.name_map.get(curr_phase.toString())+"=="+EXPR.name_map.get(new RDDL.INT_CONST_EXPR(e1.enum_to_int(hmtypes,hm_variables)).toString()));
//
//
//        System.out.println("dkfkdfjdk");
//
//
//
//
//
//
//
//
//
//
//








        //z = [ x == y ]
        // z = v1 OR v2
        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
        //v1=1 : -ep <= x-y <= 0
        //v1=0 : -M-ep <= x-y <= 0,

        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
        //v2 =1 : 0 <= x-y <= ep
        //v2=0 : 0 <= x-y <= M+ep


    }


    public static void testCaseGurobiIntLess() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());



        //This is checking the case when X1,X2,X3 are integers.

        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.INTEGER);
        type_map.put(p_x2._pName,GRB.INTEGER);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();





        //checking equality "==", X1==X2
        COMP_EXPR x1_comp_x2 = new COMP_EXPR(p_x1,p_x2,"<");
        COMP_EXPR x1_x2    = new COMP_EXPR(p_x1,p_x2,"==");

        //Adding Default values.
        EXPR def_val1=new RDDL.INT_CONST_EXPR(1);
        EXPR def_val2=new RDDL.INT_CONST_EXPR(1);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));




        GRBVar eq_var =x1_comp_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        grb_cache.get(x1_comp_x2).get(GRB.DoubleAttr.X);

        System.out.println("KDjfkdjfkd");



//
//        /////////////////////////////////////////////////////////
//        //die-value(?d) == @1
//
//        //LVAR Arraylist ;
//        LVAR l1 = new LVAR("?d");
//        ArrayList<LVAR> lvars_1 = new ArrayList<>();  lvars_1.add(l1);
//
//        //pvar_expr...
//        PVAR_EXPR die_value_p  = new PVAR_EXPR("die-value",lvars_1);
//        PVAR_EXPR nf_pname     = new PVAR_EXPR("NF",lvars_1);
//        PVAR_EXPR roll_p       = new PVAR_EXPR("roll",lvars_1);
//
//        ENUM_VAL e1 = new ENUM_VAL("@1");
//        ENUM_VAL e2 = new ENUM_VAL("@2");
//        ENUM_VAL e3 = new ENUM_VAL("@3");
//        ENUM_VAL e4 = new ENUM_VAL("@4");
//        ENUM_VAL e5 = new ENUM_VAL("@5");
//        ENUM_VAL e6 = new ENUM_VAL("@6");
//
//
//        //Checking ENUM_VAL isConstant()
//        //hmtypes are defined!!!!.
//        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
//        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
//        array_enum.add(e6);
//        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
//
//        //First Hmtype
//        TYPE_NAME s_int = new TYPE_NAME("small-int");
//        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
//        hmtypes.put(s_int,enum_t_def);
//
//        //second hmtype
//        TYPE_NAME die_t = new TYPE_NAME("die");
//        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
//        hmtypes.put(die_t,die_o_def);
//
//
//        //hmvariables are defined!!!!.
//        //First fluent variable
//        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
//        PVAR_NAME die_value = new PVAR_NAME("die-value");
//        ArrayList<String> ar_string = new ArrayList<>();
//        ar_string.add("die");
//        RDDL.PVARIABLE_STATE_DEF p_s_def = new RDDL.PVARIABLE_STATE_DEF("die-value",false, "small-int", ar_string, e1);
//        hm_variables.put(die_value_p._pName,p_s_def);
//        //Second Non-fluent Variable.
//
//        ArrayList<String> ar_string_1 = new ArrayList<>();
//        ar_string_1.add("die");
//        RDDL.PVARIABLE_STATE_DEF nf_s_def = new RDDL.PVARIABLE_STATE_DEF("NF",true, "small-int", ar_string_1, e1);
//        hm_variables.put(nf_pname._pName,nf_s_def);
//
//        //Third Action Fluent.
//        ArrayList<String> ar_string_2 = new ArrayList<>();
//        ar_string_2.add("roll");
//        RDDL.PVARIABLE_ACTION_DEF ac_def = new RDDL.PVARIABLE_ACTION_DEF("roll","bool",ar_string_2,false);
//        hm_variables.put(roll_p._pName,ac_def);
//
//
//
//        //defining objects////////////////////////////////////////
//        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
//        TYPE_NAME die_type = new TYPE_NAME("die");
//        LCONST lconst_d1   = new OBJECT_VAL("$d1");
//        LCONST lconst_d2   = new OBJECT_VAL("$d2");
//        LCONST lconst_d3   = new OBJECT_VAL("$d3");
//        ArrayList<Object> temp_objects = new ArrayList<>();
//        temp_objects.add(lconst_d1); temp_objects.add(lconst_d2); temp_objects.add(lconst_d3);
//        OBJECTS_DEF ob = new OBJECTS_DEF("die",temp_objects);
//        objects.put(die_type,ob);
//        /////////////////////////////////////////////////////////////////
//        //Defining Constants
//        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
//        Map<ArrayList<LCONST>,Object> cons1 = new HashMap<>();
//        for(int i = 1; i<4 ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(new OBJECT_VAL("$d"+Integer.valueOf(i).toString())); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons1.put(lconst_array,new ENUM_VAL("@"+Integer.valueOf(i).toString()));
//        }
//        constants.put(nf_pname._pName,cons1);
//        //////////////////////////////////////////////////////////////////
//
//
//
//        LTYPED_VAR qd1 = new LTYPED_VAR("?d","die");
//        ArrayList<LTYPED_VAR> array_ltyped = new ArrayList<>();     array_ltyped.add(qd1);
//
//
//        COMP_EXPR e_1  = new COMP_EXPR(die_value_p,e1,"==");
//        COMP_EXPR e_2  = new COMP_EXPR(nf_pname,e1,"==");
//        CONN_EXPR e_3  = new CONN_EXPR(e_1,e_2,"|");
//        QUANT_EXPR e_4 = new QUANT_EXPR("forall",array_ltyped,e_3);
//
//        //e_2.substitute(null,null,objects,hmtypes, hm_variables);
//
//        Map<LVAR, LCONST> subs = new HashMap<>();
//        LVAR a_lvar = new LVAR("?d");
//        LCONST a_cont = new OBJECT_VAL("$d1");
//        subs.put(a_lvar,a_cont);
//        EXPR sub_e_2 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_e_2.getDoubleValue(constants,objects,hmtypes,hm_variables,  null);
//        EXPR sub_expr = nf_pname.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_expr.equals(new ENUM_VAL("@1"));
////        EXPR sub_e1 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//
//        //Adding type_map :
//        Map<PVAR_NAME,Character> type_map = new HashMap<>();
//        type_map.put(die_value_p._pName,GRB.INTEGER);
//
//
//        //This Piece of code for testing Discerte Expression.
//
//
//        //GRBVar gvar = sub_e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//
//        //new RDDL.Discrete("small-int")
//
//        //Defining 1/6
//
//        OPER_EXPR div_6 = new OPER_EXPR(new RDDL.INT_CONST_EXPR(1),new RDDL.INT_CONST_EXPR(6),"/");
//        ArrayList<EXPR> discrete_array = new ArrayList<>();
//        for(int i=1 ; i< 7 ; i++){
//            discrete_array.add(new ENUM_VAL("@"+String.valueOf(i)));
//            discrete_array.add(div_6);
//        }
//
//        Discrete dis = new Discrete("small-int",discrete_array);
//
//        RDDL.IF_EXPR if_else_expr = new RDDL.IF_EXPR(roll_p,dis,die_value_p);
//        RandomDataGenerator rand = new RandomDataGenerator();
//        EXPR test_dis_sample = dis.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //EXPR test_1 =( ((OPER_EXPR) ((OPER_EXPR) ((OPER_EXPR)((OPER_EXPR) ((OPER_EXPR) test_dis_sample)._e1)._e1)._e1)._e1)._e1);
//        //test_1.getDoubleValue(constants,objects,hmtypes,hm_variables);
//        EXPR test_2 = new OPER_EXPR(e1,new REAL_CONST_EXPR(2.0),"*");
//        //test_2.isPiecewiseLinear(constants,objects,hmtypes,hm_variables);
//        //test_2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        System.out.println("dkfjkdjfkdjfkjd");
//        EXPR die_sub_expr = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        //die_sub_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.write("testing.lp");
//
//        ///////////////////////////////////////////////////////////        //FOrall Expression
//        //NF2 adding king
//        ENUM_VAL k_1 = new ENUM_VAL("@Blue");
//        ENUM_VAL k_2 = new ENUM_VAL("@Green");
//        ENUM_VAL k_3 = new ENUM_VAL("@Red");
//        ENUM_VAL k_4 = new ENUM_VAL("@Yellow");
//        ENUM_VAL k_5 = new ENUM_VAL("@Purple");
//        ENUM_VAL k_6 = new ENUM_VAL("@Pink");
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        ArrayList<ENUM_VAL> array_enum1 = new ArrayList<>(); array_enum.add(e1);
//        array_enum1.add(k_1); array_enum1.add(k_2); array_enum1.add(k_3); array_enum1.add(k_4);
//
//
//        //First Hmtype
//        TYPE_NAME color_type = new TYPE_NAME("color");
//        TYPE_DEF enum_color_def = new RDDL.ENUM_TYPE_DEF("color", array_enum1);
//        hmtypes.put(color_type,enum_color_def);
//
//
//        LVAR l_col = new LVAR("?c");
//        ArrayList<LVAR> lvars_col_1 = new ArrayList<>();  lvars_col_1.add(l_col);
//        PVAR_EXPR a_to =new PVAR_EXPR("a-to",lvars_col_1);
//        PVAR_EXPR b_to = new PVAR_EXPR("B-TO",lvars_col_1);
//
//        ArrayList<String> co_string = new ArrayList<>();
//        co_string.add("color");
//
//        RDDL.PVARIABLE_STATE_DEF a_to_def = new RDDL.PVARIABLE_STATE_DEF("a-to",false, "bool", co_string, false);
//        hm_variables.put(a_to._pName,a_to_def);
//
//        RDDL.PVARIABLE_STATE_DEF b_to_def = new RDDL.PVARIABLE_STATE_DEF("B-TO",true, "bool", co_string, false);
//        hm_variables.put(b_to._pName,b_to_def);
//
//        Map<ArrayList<LCONST>,Object> cons2 = new HashMap<>();
//        ArrayList<ENUM_VAL> enum_array = new ArrayList<>();
//        enum_array.add(k_1); enum_array.add(k_2);enum_array.add(k_3);enum_array.add(k_4);
//        for(int i = 0; i<enum_array.size() ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(enum_array.get(i)); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons2.put(lconst_array,true);
//        }
//        constants.put(b_to._pName,cons2);
//
//        LTYPED_VAR colr_type = new LTYPED_VAR("?c","color");
//        ArrayList<LTYPED_VAR> array_ltyped1 = new ArrayList<>();     array_ltyped1.add(colr_type);
//
//
//        EXPR f2 = new CONN_EXPR(a_to,b_to,"=>");
//        EXPR f1 = new QUANT_EXPR("forall",array_ltyped1,f2);
//        EXPR p2 = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        ArrayList<BOOL_EXPR> n_list = new ArrayList<>();
//        n_list.add((BOOL_EXPR) p2); n_list.add(new RDDL.BOOL_CONST_EXPR(true));
//        EXPR f4 = new CONN_EXPR(n_list,"=>");
//        //EXPR f3 = new CONN_EXPR(new RDDL.BOOL_CONST_EXPR(true),new RDDL.BOOL_CONST_EXPR(true),"^");
//
//        EXPR s1 =f1.substitute(Collections.EMPTY_MAP,constants,objects,hmtypes,hm_variables);
//
//        ArrayList<EXPR> discrete_array1 = new ArrayList<>();
//        for(int i=0 ; i< array_enum1.size() ; i++){
//            discrete_array1.add(array_enum1.get(i));
//            discrete_array1.add(div_6);
//        }
//
//        Discrete dis1 = new Discrete("color",discrete_array1);
//        EXPR sam1 =dis1.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //sam1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        EXPR c_m_va = new OPER_EXPR(k_1,new RDDL.BOOL_CONST_EXPR(true),"*");
//        //c_m_va.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //Current Phase Code
//        PVAR_EXPR curr_phase =new PVAR_EXPR("Current-Phase",new ArrayList());
////        ArrayList<String> cur_list = new ArrayList<>();
////        cur_list.add("die");
//        RDDL.PVARIABLE_STATE_DEF cur_phase_psd = new RDDL.PVARIABLE_STATE_DEF("Current-Phase",true, "small-int", new ArrayList(), e1);
//        hm_variables.put(curr_phase._pName,cur_phase_psd);
//
//        COMP_EXPR curr_expr = new COMP_EXPR(curr_phase,e1,"==");
//
//        System.out.println("dkjfkdjfkd");
//
//        PVAR_EXPR temp1_pvar = new PVAR_EXPR("temp1",new ArrayList());
//        PVAR_EXPR temp2_pvar = new PVAR_EXPR("temp2",new ArrayList());
//
//        type_map.put(temp1_pvar._pName,GRB.BINARY);
//        type_map.put(temp2_pvar._pName,GRB.BINARY);
//        type_map.put(curr_phase._pName,GRB.INTEGER);
//        RDDL.PVARIABLE_STATE_DEF temp1_psd = new RDDL.PVARIABLE_STATE_DEF("temp1",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp1_pvar._pName,temp1_psd);
//        RDDL.PVARIABLE_STATE_DEF temp2_psd = new RDDL.PVARIABLE_STATE_DEF("temp2",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp2_pvar._pName,temp2_psd);
//
//
//
//
//        CONN_EXPR or_expr =new CONN_EXPR(temp1_pvar, temp2_pvar,"|");
//
//        //z = v1 or v2
//        GRBVar z = curr_expr.getGRBVar(curr_expr, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar or_grbvar =or_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(z,GRB.EQUAL,or_grbvar,EXPR.name_map.get(curr_expr.toString()) +"==" +EXPR.name_map.get(or_expr.toString()) );
//
//
//        GRBVar t1_grb = temp1_pvar.getGRBVar(temp1_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar t2_grb = temp2_pvar.getGRBVar(temp2_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//
//
//
//        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
//        //v1=1 : -ep <= x-y <= 0
//        //v1=0 : -M-ep <= x-y <= 0,
//
//
//        final GRBLinExpr minus_M_one_minus_z = new GRBLinExpr();//-M(1-v1)-ep = -M+Mv1 -ep
//        minus_M_one_minus_z.addConstant(-1d*M );
//        minus_M_one_minus_z.addTerm(1d*M, t1_grb);
//        minus_M_one_minus_z.addConstant(-1d*ep);
//
//        final GRBLinExpr zero_grb = new GRBLinExpr();
//        zero_grb.addConstant(0);
//
//        static_grb_model.addConstr( minus_M_one_minus_z, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, zero_grb, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_2" );
//
//
//        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
//        //v2 =1 : 0 <= x-y <= ep
//        //v2=0 : 0 <= x-y <= M+ep
//
//
//        final GRBLinExpr ex_val  = new GRBLinExpr();//M(1-v2)+ep = M-Mv1+ep
//        ex_val.addConstant(M);
//        ex_val.addTerm(-1d*M, t2_grb);
//        ex_val.addConstant(ep);
//
//
//        static_grb_model.addConstr( zero_grb, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, ex_val, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_2" );
//
//
//
//        //Working on Current-phase = @1, and @1.
//
//        GRBVar cur_expr_grbvar =curr_phase.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        GRBVar enum_val_grbvar = e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(cur_expr_grbvar,GRB.EQUAL,enum_val_grbvar,EXPR.name_map.get(curr_phase.toString())+"=="+EXPR.name_map.get(new RDDL.INT_CONST_EXPR(e1.enum_to_int(hmtypes,hm_variables)).toString()));
//
//
//        System.out.println("dkfkdfjdk");
//
//
//
//
//
//
//
//
//
//
//








        //z = [ x == y ]
        // z = v1 OR v2
        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
        //v1=1 : -ep <= x-y <= 0
        //v1=0 : -M-ep <= x-y <= 0,

        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
        //v2 =1 : 0 <= x-y <= ep
        //v2=0 : 0 <= x-y <= M+ep


    }





    public static void testCaseGurobiRealLess() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());



        //This is checking the case when X1,X2,X3 are integers.

        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.CONTINUOUS);
        type_map.put(p_x2._pName,GRB.CONTINUOUS);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();





        //checking equality "==", X1==X2
        COMP_EXPR x1_comp_x2 = new COMP_EXPR(p_x1,p_x2,"<");
        COMP_EXPR x1_x2    = new COMP_EXPR(p_x1,p_x2,"==");

        //Adding Default values.
        EXPR def_val1=new RDDL.REAL_CONST_EXPR(1.0);
        EXPR def_val2=new RDDL.REAL_CONST_EXPR(1.0);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));




        GRBVar eq_var =x1_comp_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val =grb_cache.get(x1_comp_x2).get(GRB.DoubleAttr.X);
        System.out.println("########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() +"    "+x1_comp_x2.toString()+ "   " +String.valueOf(val) );

        System.out.println("KDjfkdjfkd");



//
//        /////////////////////////////////////////////////////////
//        //die-value(?d) == @1
//
//        //LVAR Arraylist ;
//        LVAR l1 = new LVAR("?d");
//        ArrayList<LVAR> lvars_1 = new ArrayList<>();  lvars_1.add(l1);
//
//        //pvar_expr...
//        PVAR_EXPR die_value_p  = new PVAR_EXPR("die-value",lvars_1);
//        PVAR_EXPR nf_pname     = new PVAR_EXPR("NF",lvars_1);
//        PVAR_EXPR roll_p       = new PVAR_EXPR("roll",lvars_1);
//
//        ENUM_VAL e1 = new ENUM_VAL("@1");
//        ENUM_VAL e2 = new ENUM_VAL("@2");
//        ENUM_VAL e3 = new ENUM_VAL("@3");
//        ENUM_VAL e4 = new ENUM_VAL("@4");
//        ENUM_VAL e5 = new ENUM_VAL("@5");
//        ENUM_VAL e6 = new ENUM_VAL("@6");
//
//
//        //Checking ENUM_VAL isConstant()
//        //hmtypes are defined!!!!.
//        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
//        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
//        array_enum.add(e6);
//        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
//
//        //First Hmtype
//        TYPE_NAME s_int = new TYPE_NAME("small-int");
//        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
//        hmtypes.put(s_int,enum_t_def);
//
//        //second hmtype
//        TYPE_NAME die_t = new TYPE_NAME("die");
//        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
//        hmtypes.put(die_t,die_o_def);
//
//
//        //hmvariables are defined!!!!.
//        //First fluent variable
//        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
//        PVAR_NAME die_value = new PVAR_NAME("die-value");
//        ArrayList<String> ar_string = new ArrayList<>();
//        ar_string.add("die");
//        RDDL.PVARIABLE_STATE_DEF p_s_def = new RDDL.PVARIABLE_STATE_DEF("die-value",false, "small-int", ar_string, e1);
//        hm_variables.put(die_value_p._pName,p_s_def);
//        //Second Non-fluent Variable.
//
//        ArrayList<String> ar_string_1 = new ArrayList<>();
//        ar_string_1.add("die");
//        RDDL.PVARIABLE_STATE_DEF nf_s_def = new RDDL.PVARIABLE_STATE_DEF("NF",true, "small-int", ar_string_1, e1);
//        hm_variables.put(nf_pname._pName,nf_s_def);
//
//        //Third Action Fluent.
//        ArrayList<String> ar_string_2 = new ArrayList<>();
//        ar_string_2.add("roll");
//        RDDL.PVARIABLE_ACTION_DEF ac_def = new RDDL.PVARIABLE_ACTION_DEF("roll","bool",ar_string_2,false);
//        hm_variables.put(roll_p._pName,ac_def);
//
//
//
//        //defining objects////////////////////////////////////////
//        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
//        TYPE_NAME die_type = new TYPE_NAME("die");
//        LCONST lconst_d1   = new OBJECT_VAL("$d1");
//        LCONST lconst_d2   = new OBJECT_VAL("$d2");
//        LCONST lconst_d3   = new OBJECT_VAL("$d3");
//        ArrayList<Object> temp_objects = new ArrayList<>();
//        temp_objects.add(lconst_d1); temp_objects.add(lconst_d2); temp_objects.add(lconst_d3);
//        OBJECTS_DEF ob = new OBJECTS_DEF("die",temp_objects);
//        objects.put(die_type,ob);
//        /////////////////////////////////////////////////////////////////
//        //Defining Constants
//        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
//        Map<ArrayList<LCONST>,Object> cons1 = new HashMap<>();
//        for(int i = 1; i<4 ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(new OBJECT_VAL("$d"+Integer.valueOf(i).toString())); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons1.put(lconst_array,new ENUM_VAL("@"+Integer.valueOf(i).toString()));
//        }
//        constants.put(nf_pname._pName,cons1);
//        //////////////////////////////////////////////////////////////////
//
//
//
//        LTYPED_VAR qd1 = new LTYPED_VAR("?d","die");
//        ArrayList<LTYPED_VAR> array_ltyped = new ArrayList<>();     array_ltyped.add(qd1);
//
//
//        COMP_EXPR e_1  = new COMP_EXPR(die_value_p,e1,"==");
//        COMP_EXPR e_2  = new COMP_EXPR(nf_pname,e1,"==");
//        CONN_EXPR e_3  = new CONN_EXPR(e_1,e_2,"|");
//        QUANT_EXPR e_4 = new QUANT_EXPR("forall",array_ltyped,e_3);
//
//        //e_2.substitute(null,null,objects,hmtypes, hm_variables);
//
//        Map<LVAR, LCONST> subs = new HashMap<>();
//        LVAR a_lvar = new LVAR("?d");
//        LCONST a_cont = new OBJECT_VAL("$d1");
//        subs.put(a_lvar,a_cont);
//        EXPR sub_e_2 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_e_2.getDoubleValue(constants,objects,hmtypes,hm_variables,  null);
//        EXPR sub_expr = nf_pname.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_expr.equals(new ENUM_VAL("@1"));
////        EXPR sub_e1 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//
//        //Adding type_map :
//        Map<PVAR_NAME,Character> type_map = new HashMap<>();
//        type_map.put(die_value_p._pName,GRB.INTEGER);
//
//
//        //This Piece of code for testing Discerte Expression.
//
//
//        //GRBVar gvar = sub_e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//
//        //new RDDL.Discrete("small-int")
//
//        //Defining 1/6
//
//        OPER_EXPR div_6 = new OPER_EXPR(new RDDL.INT_CONST_EXPR(1),new RDDL.INT_CONST_EXPR(6),"/");
//        ArrayList<EXPR> discrete_array = new ArrayList<>();
//        for(int i=1 ; i< 7 ; i++){
//            discrete_array.add(new ENUM_VAL("@"+String.valueOf(i)));
//            discrete_array.add(div_6);
//        }
//
//        Discrete dis = new Discrete("small-int",discrete_array);
//
//        RDDL.IF_EXPR if_else_expr = new RDDL.IF_EXPR(roll_p,dis,die_value_p);
//        RandomDataGenerator rand = new RandomDataGenerator();
//        EXPR test_dis_sample = dis.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //EXPR test_1 =( ((OPER_EXPR) ((OPER_EXPR) ((OPER_EXPR)((OPER_EXPR) ((OPER_EXPR) test_dis_sample)._e1)._e1)._e1)._e1)._e1);
//        //test_1.getDoubleValue(constants,objects,hmtypes,hm_variables);
//        EXPR test_2 = new OPER_EXPR(e1,new REAL_CONST_EXPR(2.0),"*");
//        //test_2.isPiecewiseLinear(constants,objects,hmtypes,hm_variables);
//        //test_2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        System.out.println("dkfjkdjfkdjfkjd");
//        EXPR die_sub_expr = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        //die_sub_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.write("testing.lp");
//
//        ///////////////////////////////////////////////////////////        //FOrall Expression
//        //NF2 adding king
//        ENUM_VAL k_1 = new ENUM_VAL("@Blue");
//        ENUM_VAL k_2 = new ENUM_VAL("@Green");
//        ENUM_VAL k_3 = new ENUM_VAL("@Red");
//        ENUM_VAL k_4 = new ENUM_VAL("@Yellow");
//        ENUM_VAL k_5 = new ENUM_VAL("@Purple");
//        ENUM_VAL k_6 = new ENUM_VAL("@Pink");
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        ArrayList<ENUM_VAL> array_enum1 = new ArrayList<>(); array_enum.add(e1);
//        array_enum1.add(k_1); array_enum1.add(k_2); array_enum1.add(k_3); array_enum1.add(k_4);
//
//
//        //First Hmtype
//        TYPE_NAME color_type = new TYPE_NAME("color");
//        TYPE_DEF enum_color_def = new RDDL.ENUM_TYPE_DEF("color", array_enum1);
//        hmtypes.put(color_type,enum_color_def);
//
//
//        LVAR l_col = new LVAR("?c");
//        ArrayList<LVAR> lvars_col_1 = new ArrayList<>();  lvars_col_1.add(l_col);
//        PVAR_EXPR a_to =new PVAR_EXPR("a-to",lvars_col_1);
//        PVAR_EXPR b_to = new PVAR_EXPR("B-TO",lvars_col_1);
//
//        ArrayList<String> co_string = new ArrayList<>();
//        co_string.add("color");
//
//        RDDL.PVARIABLE_STATE_DEF a_to_def = new RDDL.PVARIABLE_STATE_DEF("a-to",false, "bool", co_string, false);
//        hm_variables.put(a_to._pName,a_to_def);
//
//        RDDL.PVARIABLE_STATE_DEF b_to_def = new RDDL.PVARIABLE_STATE_DEF("B-TO",true, "bool", co_string, false);
//        hm_variables.put(b_to._pName,b_to_def);
//
//        Map<ArrayList<LCONST>,Object> cons2 = new HashMap<>();
//        ArrayList<ENUM_VAL> enum_array = new ArrayList<>();
//        enum_array.add(k_1); enum_array.add(k_2);enum_array.add(k_3);enum_array.add(k_4);
//        for(int i = 0; i<enum_array.size() ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(enum_array.get(i)); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons2.put(lconst_array,true);
//        }
//        constants.put(b_to._pName,cons2);
//
//        LTYPED_VAR colr_type = new LTYPED_VAR("?c","color");
//        ArrayList<LTYPED_VAR> array_ltyped1 = new ArrayList<>();     array_ltyped1.add(colr_type);
//
//
//        EXPR f2 = new CONN_EXPR(a_to,b_to,"=>");
//        EXPR f1 = new QUANT_EXPR("forall",array_ltyped1,f2);
//        EXPR p2 = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        ArrayList<BOOL_EXPR> n_list = new ArrayList<>();
//        n_list.add((BOOL_EXPR) p2); n_list.add(new RDDL.BOOL_CONST_EXPR(true));
//        EXPR f4 = new CONN_EXPR(n_list,"=>");
//        //EXPR f3 = new CONN_EXPR(new RDDL.BOOL_CONST_EXPR(true),new RDDL.BOOL_CONST_EXPR(true),"^");
//
//        EXPR s1 =f1.substitute(Collections.EMPTY_MAP,constants,objects,hmtypes,hm_variables);
//
//        ArrayList<EXPR> discrete_array1 = new ArrayList<>();
//        for(int i=0 ; i< array_enum1.size() ; i++){
//            discrete_array1.add(array_enum1.get(i));
//            discrete_array1.add(div_6);
//        }
//
//        Discrete dis1 = new Discrete("color",discrete_array1);
//        EXPR sam1 =dis1.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //sam1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        EXPR c_m_va = new OPER_EXPR(k_1,new RDDL.BOOL_CONST_EXPR(true),"*");
//        //c_m_va.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //Current Phase Code
//        PVAR_EXPR curr_phase =new PVAR_EXPR("Current-Phase",new ArrayList());
////        ArrayList<String> cur_list = new ArrayList<>();
////        cur_list.add("die");
//        RDDL.PVARIABLE_STATE_DEF cur_phase_psd = new RDDL.PVARIABLE_STATE_DEF("Current-Phase",true, "small-int", new ArrayList(), e1);
//        hm_variables.put(curr_phase._pName,cur_phase_psd);
//
//        COMP_EXPR curr_expr = new COMP_EXPR(curr_phase,e1,"==");
//
//        System.out.println("dkjfkdjfkd");
//
//        PVAR_EXPR temp1_pvar = new PVAR_EXPR("temp1",new ArrayList());
//        PVAR_EXPR temp2_pvar = new PVAR_EXPR("temp2",new ArrayList());
//
//        type_map.put(temp1_pvar._pName,GRB.BINARY);
//        type_map.put(temp2_pvar._pName,GRB.BINARY);
//        type_map.put(curr_phase._pName,GRB.INTEGER);
//        RDDL.PVARIABLE_STATE_DEF temp1_psd = new RDDL.PVARIABLE_STATE_DEF("temp1",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp1_pvar._pName,temp1_psd);
//        RDDL.PVARIABLE_STATE_DEF temp2_psd = new RDDL.PVARIABLE_STATE_DEF("temp2",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp2_pvar._pName,temp2_psd);
//
//
//
//
//        CONN_EXPR or_expr =new CONN_EXPR(temp1_pvar, temp2_pvar,"|");
//
//        //z = v1 or v2
//        GRBVar z = curr_expr.getGRBVar(curr_expr, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar or_grbvar =or_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(z,GRB.EQUAL,or_grbvar,EXPR.name_map.get(curr_expr.toString()) +"==" +EXPR.name_map.get(or_expr.toString()) );
//
//
//        GRBVar t1_grb = temp1_pvar.getGRBVar(temp1_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar t2_grb = temp2_pvar.getGRBVar(temp2_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//
//
//
//        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
//        //v1=1 : -ep <= x-y <= 0
//        //v1=0 : -M-ep <= x-y <= 0,
//
//
//        final GRBLinExpr minus_M_one_minus_z = new GRBLinExpr();//-M(1-v1)-ep = -M+Mv1 -ep
//        minus_M_one_minus_z.addConstant(-1d*M );
//        minus_M_one_minus_z.addTerm(1d*M, t1_grb);
//        minus_M_one_minus_z.addConstant(-1d*ep);
//
//        final GRBLinExpr zero_grb = new GRBLinExpr();
//        zero_grb.addConstant(0);
//
//        static_grb_model.addConstr( minus_M_one_minus_z, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, zero_grb, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_2" );
//
//
//        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
//        //v2 =1 : 0 <= x-y <= ep
//        //v2=0 : 0 <= x-y <= M+ep
//
//
//        final GRBLinExpr ex_val  = new GRBLinExpr();//M(1-v2)+ep = M-Mv1+ep
//        ex_val.addConstant(M);
//        ex_val.addTerm(-1d*M, t2_grb);
//        ex_val.addConstant(ep);
//
//
//        static_grb_model.addConstr( zero_grb, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, ex_val, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_2" );
//
//
//
//        //Working on Current-phase = @1, and @1.
//
//        GRBVar cur_expr_grbvar =curr_phase.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        GRBVar enum_val_grbvar = e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(cur_expr_grbvar,GRB.EQUAL,enum_val_grbvar,EXPR.name_map.get(curr_phase.toString())+"=="+EXPR.name_map.get(new RDDL.INT_CONST_EXPR(e1.enum_to_int(hmtypes,hm_variables)).toString()));
//
//
//        System.out.println("dkfkdfjdk");
//
//
//
//
//
//
//
//
//
//
//








        //z = [ x == y ]
        // z = v1 OR v2
        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
        //v1=1 : -ep <= x-y <= 0
        //v1=0 : -M-ep <= x-y <= 0,

        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
        //v2 =1 : 0 <= x-y <= ep
        //v2=0 : 0 <= x-y <= M+ep


    }





    public static void testCaseGurobiRealAnd() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());



        //This is checking the case when X1,X2,X3 are integers.

        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.CONTINUOUS);
        type_map.put(p_x2._pName,GRB.CONTINUOUS);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();





        //checking equality "==", X1==X2
        COMP_EXPR x1_comp_x2 = new COMP_EXPR(p_x1,p_x2,"<");
        COMP_EXPR x1_x2    = new COMP_EXPR(p_x1,p_x2,"==");

        COMP_EXPR x2_comp_x1 = new COMP_EXPR(p_x2,p_x1,"==");

        CONN_EXPR x1_conn_x2 = new CONN_EXPR(x1_comp_x2,x2_comp_x1,"^");

        //Adding Default values.
        EXPR def_val1=new RDDL.REAL_CONST_EXPR(1.0);
        EXPR def_val2=new RDDL.REAL_CONST_EXPR(1.0);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));




        GRBVar eq_var1 =x1_comp_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar eq_var2 = x2_comp_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar con_var1 = x1_conn_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);

        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_comp_x2).get(GRB.DoubleAttr.X);
        double val2 =grb_cache.get(x1_comp_x2).get(GRB.DoubleAttr.X);

        System.out.println("########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() +"    "+x1_comp_x2.toString()+ "   " +String.valueOf(val1) );

        System.out.println("#########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() +"    "+x2_comp_x1.toString()+ "   " +String.valueOf(val2) );


        System.out.println("KDjfkdjfkd");



//
//        /////////////////////////////////////////////////////////
//        //die-value(?d) == @1
//
//        //LVAR Arraylist ;
//        LVAR l1 = new LVAR("?d");
//        ArrayList<LVAR> lvars_1 = new ArrayList<>();  lvars_1.add(l1);
//
//        //pvar_expr...
//        PVAR_EXPR die_value_p  = new PVAR_EXPR("die-value",lvars_1);
//        PVAR_EXPR nf_pname     = new PVAR_EXPR("NF",lvars_1);
//        PVAR_EXPR roll_p       = new PVAR_EXPR("roll",lvars_1);
//
//        ENUM_VAL e1 = new ENUM_VAL("@1");
//        ENUM_VAL e2 = new ENUM_VAL("@2");
//        ENUM_VAL e3 = new ENUM_VAL("@3");
//        ENUM_VAL e4 = new ENUM_VAL("@4");
//        ENUM_VAL e5 = new ENUM_VAL("@5");
//        ENUM_VAL e6 = new ENUM_VAL("@6");
//
//
//        //Checking ENUM_VAL isConstant()
//        //hmtypes are defined!!!!.
//        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
//        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
//        array_enum.add(e6);
//        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
//
//        //First Hmtype
//        TYPE_NAME s_int = new TYPE_NAME("small-int");
//        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
//        hmtypes.put(s_int,enum_t_def);
//
//        //second hmtype
//        TYPE_NAME die_t = new TYPE_NAME("die");
//        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
//        hmtypes.put(die_t,die_o_def);
//
//
//        //hmvariables are defined!!!!.
//        //First fluent variable
//        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
//        PVAR_NAME die_value = new PVAR_NAME("die-value");
//        ArrayList<String> ar_string = new ArrayList<>();
//        ar_string.add("die");
//        RDDL.PVARIABLE_STATE_DEF p_s_def = new RDDL.PVARIABLE_STATE_DEF("die-value",false, "small-int", ar_string, e1);
//        hm_variables.put(die_value_p._pName,p_s_def);
//        //Second Non-fluent Variable.
//
//        ArrayList<String> ar_string_1 = new ArrayList<>();
//        ar_string_1.add("die");
//        RDDL.PVARIABLE_STATE_DEF nf_s_def = new RDDL.PVARIABLE_STATE_DEF("NF",true, "small-int", ar_string_1, e1);
//        hm_variables.put(nf_pname._pName,nf_s_def);
//
//        //Third Action Fluent.
//        ArrayList<String> ar_string_2 = new ArrayList<>();
//        ar_string_2.add("roll");
//        RDDL.PVARIABLE_ACTION_DEF ac_def = new RDDL.PVARIABLE_ACTION_DEF("roll","bool",ar_string_2,false);
//        hm_variables.put(roll_p._pName,ac_def);
//
//
//
//        //defining objects////////////////////////////////////////
//        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
//        TYPE_NAME die_type = new TYPE_NAME("die");
//        LCONST lconst_d1   = new OBJECT_VAL("$d1");
//        LCONST lconst_d2   = new OBJECT_VAL("$d2");
//        LCONST lconst_d3   = new OBJECT_VAL("$d3");
//        ArrayList<Object> temp_objects = new ArrayList<>();
//        temp_objects.add(lconst_d1); temp_objects.add(lconst_d2); temp_objects.add(lconst_d3);
//        OBJECTS_DEF ob = new OBJECTS_DEF("die",temp_objects);
//        objects.put(die_type,ob);
//        /////////////////////////////////////////////////////////////////
//        //Defining Constants
//        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
//        Map<ArrayList<LCONST>,Object> cons1 = new HashMap<>();
//        for(int i = 1; i<4 ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(new OBJECT_VAL("$d"+Integer.valueOf(i).toString())); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons1.put(lconst_array,new ENUM_VAL("@"+Integer.valueOf(i).toString()));
//        }
//        constants.put(nf_pname._pName,cons1);
//        //////////////////////////////////////////////////////////////////
//
//
//
//        LTYPED_VAR qd1 = new LTYPED_VAR("?d","die");
//        ArrayList<LTYPED_VAR> array_ltyped = new ArrayList<>();     array_ltyped.add(qd1);
//
//
//        COMP_EXPR e_1  = new COMP_EXPR(die_value_p,e1,"==");
//        COMP_EXPR e_2  = new COMP_EXPR(nf_pname,e1,"==");
//        CONN_EXPR e_3  = new CONN_EXPR(e_1,e_2,"|");
//        QUANT_EXPR e_4 = new QUANT_EXPR("forall",array_ltyped,e_3);
//
//        //e_2.substitute(null,null,objects,hmtypes, hm_variables);
//
//        Map<LVAR, LCONST> subs = new HashMap<>();
//        LVAR a_lvar = new LVAR("?d");
//        LCONST a_cont = new OBJECT_VAL("$d1");
//        subs.put(a_lvar,a_cont);
//        EXPR sub_e_2 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_e_2.getDoubleValue(constants,objects,hmtypes,hm_variables,  null);
//        EXPR sub_expr = nf_pname.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_expr.equals(new ENUM_VAL("@1"));
////        EXPR sub_e1 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//
//        //Adding type_map :
//        Map<PVAR_NAME,Character> type_map = new HashMap<>();
//        type_map.put(die_value_p._pName,GRB.INTEGER);
//
//
//        //This Piece of code for testing Discerte Expression.
//
//
//        //GRBVar gvar = sub_e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//
//        //new RDDL.Discrete("small-int")
//
//        //Defining 1/6
//
//        OPER_EXPR div_6 = new OPER_EXPR(new RDDL.INT_CONST_EXPR(1),new RDDL.INT_CONST_EXPR(6),"/");
//        ArrayList<EXPR> discrete_array = new ArrayList<>();
//        for(int i=1 ; i< 7 ; i++){
//            discrete_array.add(new ENUM_VAL("@"+String.valueOf(i)));
//            discrete_array.add(div_6);
//        }
//
//        Discrete dis = new Discrete("small-int",discrete_array);
//
//        RDDL.IF_EXPR if_else_expr = new RDDL.IF_EXPR(roll_p,dis,die_value_p);
//        RandomDataGenerator rand = new RandomDataGenerator();
//        EXPR test_dis_sample = dis.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //EXPR test_1 =( ((OPER_EXPR) ((OPER_EXPR) ((OPER_EXPR)((OPER_EXPR) ((OPER_EXPR) test_dis_sample)._e1)._e1)._e1)._e1)._e1);
//        //test_1.getDoubleValue(constants,objects,hmtypes,hm_variables);
//        EXPR test_2 = new OPER_EXPR(e1,new REAL_CONST_EXPR(2.0),"*");
//        //test_2.isPiecewiseLinear(constants,objects,hmtypes,hm_variables);
//        //test_2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        System.out.println("dkfjkdjfkdjfkjd");
//        EXPR die_sub_expr = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        //die_sub_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.write("testing.lp");
//
//        ///////////////////////////////////////////////////////////        //FOrall Expression
//        //NF2 adding king
//        ENUM_VAL k_1 = new ENUM_VAL("@Blue");
//        ENUM_VAL k_2 = new ENUM_VAL("@Green");
//        ENUM_VAL k_3 = new ENUM_VAL("@Red");
//        ENUM_VAL k_4 = new ENUM_VAL("@Yellow");
//        ENUM_VAL k_5 = new ENUM_VAL("@Purple");
//        ENUM_VAL k_6 = new ENUM_VAL("@Pink");
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        ArrayList<ENUM_VAL> array_enum1 = new ArrayList<>(); array_enum.add(e1);
//        array_enum1.add(k_1); array_enum1.add(k_2); array_enum1.add(k_3); array_enum1.add(k_4);
//
//
//        //First Hmtype
//        TYPE_NAME color_type = new TYPE_NAME("color");
//        TYPE_DEF enum_color_def = new RDDL.ENUM_TYPE_DEF("color", array_enum1);
//        hmtypes.put(color_type,enum_color_def);
//
//
//        LVAR l_col = new LVAR("?c");
//        ArrayList<LVAR> lvars_col_1 = new ArrayList<>();  lvars_col_1.add(l_col);
//        PVAR_EXPR a_to =new PVAR_EXPR("a-to",lvars_col_1);
//        PVAR_EXPR b_to = new PVAR_EXPR("B-TO",lvars_col_1);
//
//        ArrayList<String> co_string = new ArrayList<>();
//        co_string.add("color");
//
//        RDDL.PVARIABLE_STATE_DEF a_to_def = new RDDL.PVARIABLE_STATE_DEF("a-to",false, "bool", co_string, false);
//        hm_variables.put(a_to._pName,a_to_def);
//
//        RDDL.PVARIABLE_STATE_DEF b_to_def = new RDDL.PVARIABLE_STATE_DEF("B-TO",true, "bool", co_string, false);
//        hm_variables.put(b_to._pName,b_to_def);
//
//        Map<ArrayList<LCONST>,Object> cons2 = new HashMap<>();
//        ArrayList<ENUM_VAL> enum_array = new ArrayList<>();
//        enum_array.add(k_1); enum_array.add(k_2);enum_array.add(k_3);enum_array.add(k_4);
//        for(int i = 0; i<enum_array.size() ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(enum_array.get(i)); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons2.put(lconst_array,true);
//        }
//        constants.put(b_to._pName,cons2);
//
//        LTYPED_VAR colr_type = new LTYPED_VAR("?c","color");
//        ArrayList<LTYPED_VAR> array_ltyped1 = new ArrayList<>();     array_ltyped1.add(colr_type);
//
//
//        EXPR f2 = new CONN_EXPR(a_to,b_to,"=>");
//        EXPR f1 = new QUANT_EXPR("forall",array_ltyped1,f2);
//        EXPR p2 = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        ArrayList<BOOL_EXPR> n_list = new ArrayList<>();
//        n_list.add((BOOL_EXPR) p2); n_list.add(new RDDL.BOOL_CONST_EXPR(true));
//        EXPR f4 = new CONN_EXPR(n_list,"=>");
//        //EXPR f3 = new CONN_EXPR(new RDDL.BOOL_CONST_EXPR(true),new RDDL.BOOL_CONST_EXPR(true),"^");
//
//        EXPR s1 =f1.substitute(Collections.EMPTY_MAP,constants,objects,hmtypes,hm_variables);
//
//        ArrayList<EXPR> discrete_array1 = new ArrayList<>();
//        for(int i=0 ; i< array_enum1.size() ; i++){
//            discrete_array1.add(array_enum1.get(i));
//            discrete_array1.add(div_6);
//        }
//
//        Discrete dis1 = new Discrete("color",discrete_array1);
//        EXPR sam1 =dis1.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //sam1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        EXPR c_m_va = new OPER_EXPR(k_1,new RDDL.BOOL_CONST_EXPR(true),"*");
//        //c_m_va.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //Current Phase Code
//        PVAR_EXPR curr_phase =new PVAR_EXPR("Current-Phase",new ArrayList());
////        ArrayList<String> cur_list = new ArrayList<>();
////        cur_list.add("die");
//        RDDL.PVARIABLE_STATE_DEF cur_phase_psd = new RDDL.PVARIABLE_STATE_DEF("Current-Phase",true, "small-int", new ArrayList(), e1);
//        hm_variables.put(curr_phase._pName,cur_phase_psd);
//
//        COMP_EXPR curr_expr = new COMP_EXPR(curr_phase,e1,"==");
//
//        System.out.println("dkjfkdjfkd");
//
//        PVAR_EXPR temp1_pvar = new PVAR_EXPR("temp1",new ArrayList());
//        PVAR_EXPR temp2_pvar = new PVAR_EXPR("temp2",new ArrayList());
//
//        type_map.put(temp1_pvar._pName,GRB.BINARY);
//        type_map.put(temp2_pvar._pName,GRB.BINARY);
//        type_map.put(curr_phase._pName,GRB.INTEGER);
//        RDDL.PVARIABLE_STATE_DEF temp1_psd = new RDDL.PVARIABLE_STATE_DEF("temp1",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp1_pvar._pName,temp1_psd);
//        RDDL.PVARIABLE_STATE_DEF temp2_psd = new RDDL.PVARIABLE_STATE_DEF("temp2",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp2_pvar._pName,temp2_psd);
//
//
//
//
//        CONN_EXPR or_expr =new CONN_EXPR(temp1_pvar, temp2_pvar,"|");
//
//        //z = v1 or v2
//        GRBVar z = curr_expr.getGRBVar(curr_expr, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar or_grbvar =or_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(z,GRB.EQUAL,or_grbvar,EXPR.name_map.get(curr_expr.toString()) +"==" +EXPR.name_map.get(or_expr.toString()) );
//
//
//        GRBVar t1_grb = temp1_pvar.getGRBVar(temp1_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar t2_grb = temp2_pvar.getGRBVar(temp2_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//
//
//
//        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
//        //v1=1 : -ep <= x-y <= 0
//        //v1=0 : -M-ep <= x-y <= 0,
//
//
//        final GRBLinExpr minus_M_one_minus_z = new GRBLinExpr();//-M(1-v1)-ep = -M+Mv1 -ep
//        minus_M_one_minus_z.addConstant(-1d*M );
//        minus_M_one_minus_z.addTerm(1d*M, t1_grb);
//        minus_M_one_minus_z.addConstant(-1d*ep);
//
//        final GRBLinExpr zero_grb = new GRBLinExpr();
//        zero_grb.addConstant(0);
//
//        static_grb_model.addConstr( minus_M_one_minus_z, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, zero_grb, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_2" );
//
//
//        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
//        //v2 =1 : 0 <= x-y <= ep
//        //v2=0 : 0 <= x-y <= M+ep
//
//
//        final GRBLinExpr ex_val  = new GRBLinExpr();//M(1-v2)+ep = M-Mv1+ep
//        ex_val.addConstant(M);
//        ex_val.addTerm(-1d*M, t2_grb);
//        ex_val.addConstant(ep);
//
//
//        static_grb_model.addConstr( zero_grb, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, ex_val, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_2" );
//
//
//
//        //Working on Current-phase = @1, and @1.
//
//        GRBVar cur_expr_grbvar =curr_phase.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        GRBVar enum_val_grbvar = e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(cur_expr_grbvar,GRB.EQUAL,enum_val_grbvar,EXPR.name_map.get(curr_phase.toString())+"=="+EXPR.name_map.get(new RDDL.INT_CONST_EXPR(e1.enum_to_int(hmtypes,hm_variables)).toString()));
//
//
//        System.out.println("dkfkdfjdk");
//
//
//
//
//
//
//
//
//
//
//








        //z = [ x == y ]
        // z = v1 OR v2
        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
        //v1=1 : -ep <= x-y <= 0
        //v1=0 : -M-ep <= x-y <= 0,

        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
        //v2 =1 : 0 <= x-y <= ep
        //v2=0 : 0 <= x-y <= M+ep


    }




    public static void testCaseGurobiAnd() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());



        //This is checking the case when X1,X2,X3 are integers.

        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "bool", new ArrayList(), true);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "bool", new ArrayList(), true);
        hm_variables.put(p_x2._pName,x2_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.BINARY);
        type_map.put(p_x2._pName,GRB.BINARY);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();



        CONN_EXPR x1_conn_x2 = new CONN_EXPR(p_x1,p_x2,"^");











        //checking equality "==", X1==X2


        //Adding Default values.
        EXPR def_val1=new RDDL.BOOL_CONST_EXPR(true);
        EXPR def_val2=new RDDL.BOOL_CONST_EXPR(true);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));




        GRBVar eq_var1 =x1_conn_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);

        //static_grb_model.setObjective( new GRBLinExpr() );
        //GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_conn_x2).get(GRB.DoubleAttr.X);


        System.out.println("########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() +"    "+x1_conn_x2.toString()+ "   " +String.valueOf(val1) );





//
//        /////////////////////////////////////////////////////////
//        //die-value(?d) == @1
//
//        //LVAR Arraylist ;
//        LVAR l1 = new LVAR("?d");
//        ArrayList<LVAR> lvars_1 = new ArrayList<>();  lvars_1.add(l1);
//
//        //pvar_expr...
//        PVAR_EXPR die_value_p  = new PVAR_EXPR("die-value",lvars_1);
//        PVAR_EXPR nf_pname     = new PVAR_EXPR("NF",lvars_1);
//        PVAR_EXPR roll_p       = new PVAR_EXPR("roll",lvars_1);
//
//        ENUM_VAL e1 = new ENUM_VAL("@1");
//        ENUM_VAL e2 = new ENUM_VAL("@2");
//        ENUM_VAL e3 = new ENUM_VAL("@3");
//        ENUM_VAL e4 = new ENUM_VAL("@4");
//        ENUM_VAL e5 = new ENUM_VAL("@5");
//        ENUM_VAL e6 = new ENUM_VAL("@6");
//
//
//        //Checking ENUM_VAL isConstant()
//        //hmtypes are defined!!!!.
//        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
//        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
//        array_enum.add(e6);
//        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
//
//        //First Hmtype
//        TYPE_NAME s_int = new TYPE_NAME("small-int");
//        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
//        hmtypes.put(s_int,enum_t_def);
//
//        //second hmtype
//        TYPE_NAME die_t = new TYPE_NAME("die");
//        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
//        hmtypes.put(die_t,die_o_def);
//
//
//        //hmvariables are defined!!!!.
//        //First fluent variable
//        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
//        PVAR_NAME die_value = new PVAR_NAME("die-value");
//        ArrayList<String> ar_string = new ArrayList<>();
//        ar_string.add("die");
//        RDDL.PVARIABLE_STATE_DEF p_s_def = new RDDL.PVARIABLE_STATE_DEF("die-value",false, "small-int", ar_string, e1);
//        hm_variables.put(die_value_p._pName,p_s_def);
//        //Second Non-fluent Variable.
//
//        ArrayList<String> ar_string_1 = new ArrayList<>();
//        ar_string_1.add("die");
//        RDDL.PVARIABLE_STATE_DEF nf_s_def = new RDDL.PVARIABLE_STATE_DEF("NF",true, "small-int", ar_string_1, e1);
//        hm_variables.put(nf_pname._pName,nf_s_def);
//
//        //Third Action Fluent.
//        ArrayList<String> ar_string_2 = new ArrayList<>();
//        ar_string_2.add("roll");
//        RDDL.PVARIABLE_ACTION_DEF ac_def = new RDDL.PVARIABLE_ACTION_DEF("roll","bool",ar_string_2,false);
//        hm_variables.put(roll_p._pName,ac_def);
//
//
//
//        //defining objects////////////////////////////////////////
//        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
//        TYPE_NAME die_type = new TYPE_NAME("die");
//        LCONST lconst_d1   = new OBJECT_VAL("$d1");
//        LCONST lconst_d2   = new OBJECT_VAL("$d2");
//        LCONST lconst_d3   = new OBJECT_VAL("$d3");
//        ArrayList<Object> temp_objects = new ArrayList<>();
//        temp_objects.add(lconst_d1); temp_objects.add(lconst_d2); temp_objects.add(lconst_d3);
//        OBJECTS_DEF ob = new OBJECTS_DEF("die",temp_objects);
//        objects.put(die_type,ob);
//        /////////////////////////////////////////////////////////////////
//        //Defining Constants
//        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
//        Map<ArrayList<LCONST>,Object> cons1 = new HashMap<>();
//        for(int i = 1; i<4 ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(new OBJECT_VAL("$d"+Integer.valueOf(i).toString())); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons1.put(lconst_array,new ENUM_VAL("@"+Integer.valueOf(i).toString()));
//        }
//        constants.put(nf_pname._pName,cons1);
//        //////////////////////////////////////////////////////////////////
//
//
//
//        LTYPED_VAR qd1 = new LTYPED_VAR("?d","die");
//        ArrayList<LTYPED_VAR> array_ltyped = new ArrayList<>();     array_ltyped.add(qd1);
//
//
//        COMP_EXPR e_1  = new COMP_EXPR(die_value_p,e1,"==");
//        COMP_EXPR e_2  = new COMP_EXPR(nf_pname,e1,"==");
//        CONN_EXPR e_3  = new CONN_EXPR(e_1,e_2,"|");
//        QUANT_EXPR e_4 = new QUANT_EXPR("forall",array_ltyped,e_3);
//
//        //e_2.substitute(null,null,objects,hmtypes, hm_variables);
//
//        Map<LVAR, LCONST> subs = new HashMap<>();
//        LVAR a_lvar = new LVAR("?d");
//        LCONST a_cont = new OBJECT_VAL("$d1");
//        subs.put(a_lvar,a_cont);
//        EXPR sub_e_2 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_e_2.getDoubleValue(constants,objects,hmtypes,hm_variables,  null);
//        EXPR sub_expr = nf_pname.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_expr.equals(new ENUM_VAL("@1"));
////        EXPR sub_e1 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//
//        //Adding type_map :
//        Map<PVAR_NAME,Character> type_map = new HashMap<>();
//        type_map.put(die_value_p._pName,GRB.INTEGER);
//
//
//        //This Piece of code for testing Discerte Expression.
//
//
//        //GRBVar gvar = sub_e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//
//        //new RDDL.Discrete("small-int")
//
//        //Defining 1/6
//
//        OPER_EXPR div_6 = new OPER_EXPR(new RDDL.INT_CONST_EXPR(1),new RDDL.INT_CONST_EXPR(6),"/");
//        ArrayList<EXPR> discrete_array = new ArrayList<>();
//        for(int i=1 ; i< 7 ; i++){
//            discrete_array.add(new ENUM_VAL("@"+String.valueOf(i)));
//            discrete_array.add(div_6);
//        }
//
//        Discrete dis = new Discrete("small-int",discrete_array);
//
//        RDDL.IF_EXPR if_else_expr = new RDDL.IF_EXPR(roll_p,dis,die_value_p);
//        RandomDataGenerator rand = new RandomDataGenerator();
//        EXPR test_dis_sample = dis.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //EXPR test_1 =( ((OPER_EXPR) ((OPER_EXPR) ((OPER_EXPR)((OPER_EXPR) ((OPER_EXPR) test_dis_sample)._e1)._e1)._e1)._e1)._e1);
//        //test_1.getDoubleValue(constants,objects,hmtypes,hm_variables);
//        EXPR test_2 = new OPER_EXPR(e1,new REAL_CONST_EXPR(2.0),"*");
//        //test_2.isPiecewiseLinear(constants,objects,hmtypes,hm_variables);
//        //test_2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        System.out.println("dkfjkdjfkdjfkjd");
//        EXPR die_sub_expr = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        //die_sub_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.write("testing.lp");
//
//        ///////////////////////////////////////////////////////////        //FOrall Expression
//        //NF2 adding king
//        ENUM_VAL k_1 = new ENUM_VAL("@Blue");
//        ENUM_VAL k_2 = new ENUM_VAL("@Green");
//        ENUM_VAL k_3 = new ENUM_VAL("@Red");
//        ENUM_VAL k_4 = new ENUM_VAL("@Yellow");
//        ENUM_VAL k_5 = new ENUM_VAL("@Purple");
//        ENUM_VAL k_6 = new ENUM_VAL("@Pink");
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        ArrayList<ENUM_VAL> array_enum1 = new ArrayList<>(); array_enum.add(e1);
//        array_enum1.add(k_1); array_enum1.add(k_2); array_enum1.add(k_3); array_enum1.add(k_4);
//
//
//        //First Hmtype
//        TYPE_NAME color_type = new TYPE_NAME("color");
//        TYPE_DEF enum_color_def = new RDDL.ENUM_TYPE_DEF("color", array_enum1);
//        hmtypes.put(color_type,enum_color_def);
//
//
//        LVAR l_col = new LVAR("?c");
//        ArrayList<LVAR> lvars_col_1 = new ArrayList<>();  lvars_col_1.add(l_col);
//        PVAR_EXPR a_to =new PVAR_EXPR("a-to",lvars_col_1);
//        PVAR_EXPR b_to = new PVAR_EXPR("B-TO",lvars_col_1);
//
//        ArrayList<String> co_string = new ArrayList<>();
//        co_string.add("color");
//
//        RDDL.PVARIABLE_STATE_DEF a_to_def = new RDDL.PVARIABLE_STATE_DEF("a-to",false, "bool", co_string, false);
//        hm_variables.put(a_to._pName,a_to_def);
//
//        RDDL.PVARIABLE_STATE_DEF b_to_def = new RDDL.PVARIABLE_STATE_DEF("B-TO",true, "bool", co_string, false);
//        hm_variables.put(b_to._pName,b_to_def);
//
//        Map<ArrayList<LCONST>,Object> cons2 = new HashMap<>();
//        ArrayList<ENUM_VAL> enum_array = new ArrayList<>();
//        enum_array.add(k_1); enum_array.add(k_2);enum_array.add(k_3);enum_array.add(k_4);
//        for(int i = 0; i<enum_array.size() ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(enum_array.get(i)); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons2.put(lconst_array,true);
//        }
//        constants.put(b_to._pName,cons2);
//
//        LTYPED_VAR colr_type = new LTYPED_VAR("?c","color");
//        ArrayList<LTYPED_VAR> array_ltyped1 = new ArrayList<>();     array_ltyped1.add(colr_type);
//
//
//        EXPR f2 = new CONN_EXPR(a_to,b_to,"=>");
//        EXPR f1 = new QUANT_EXPR("forall",array_ltyped1,f2);
//        EXPR p2 = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        ArrayList<BOOL_EXPR> n_list = new ArrayList<>();
//        n_list.add((BOOL_EXPR) p2); n_list.add(new RDDL.BOOL_CONST_EXPR(true));
//        EXPR f4 = new CONN_EXPR(n_list,"=>");
//        //EXPR f3 = new CONN_EXPR(new RDDL.BOOL_CONST_EXPR(true),new RDDL.BOOL_CONST_EXPR(true),"^");
//
//        EXPR s1 =f1.substitute(Collections.EMPTY_MAP,constants,objects,hmtypes,hm_variables);
//
//        ArrayList<EXPR> discrete_array1 = new ArrayList<>();
//        for(int i=0 ; i< array_enum1.size() ; i++){
//            discrete_array1.add(array_enum1.get(i));
//            discrete_array1.add(div_6);
//        }
//
//        Discrete dis1 = new Discrete("color",discrete_array1);
//        EXPR sam1 =dis1.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //sam1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        EXPR c_m_va = new OPER_EXPR(k_1,new RDDL.BOOL_CONST_EXPR(true),"*");
//        //c_m_va.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //Current Phase Code
//        PVAR_EXPR curr_phase =new PVAR_EXPR("Current-Phase",new ArrayList());
////        ArrayList<String> cur_list = new ArrayList<>();
////        cur_list.add("die");
//        RDDL.PVARIABLE_STATE_DEF cur_phase_psd = new RDDL.PVARIABLE_STATE_DEF("Current-Phase",true, "small-int", new ArrayList(), e1);
//        hm_variables.put(curr_phase._pName,cur_phase_psd);
//
//        COMP_EXPR curr_expr = new COMP_EXPR(curr_phase,e1,"==");
//
//        System.out.println("dkjfkdjfkd");
//
//        PVAR_EXPR temp1_pvar = new PVAR_EXPR("temp1",new ArrayList());
//        PVAR_EXPR temp2_pvar = new PVAR_EXPR("temp2",new ArrayList());
//
//        type_map.put(temp1_pvar._pName,GRB.BINARY);
//        type_map.put(temp2_pvar._pName,GRB.BINARY);
//        type_map.put(curr_phase._pName,GRB.INTEGER);
//        RDDL.PVARIABLE_STATE_DEF temp1_psd = new RDDL.PVARIABLE_STATE_DEF("temp1",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp1_pvar._pName,temp1_psd);
//        RDDL.PVARIABLE_STATE_DEF temp2_psd = new RDDL.PVARIABLE_STATE_DEF("temp2",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp2_pvar._pName,temp2_psd);
//
//
//
//
//        CONN_EXPR or_expr =new CONN_EXPR(temp1_pvar, temp2_pvar,"|");
//
//        //z = v1 or v2
//        GRBVar z = curr_expr.getGRBVar(curr_expr, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar or_grbvar =or_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(z,GRB.EQUAL,or_grbvar,EXPR.name_map.get(curr_expr.toString()) +"==" +EXPR.name_map.get(or_expr.toString()) );
//
//
//        GRBVar t1_grb = temp1_pvar.getGRBVar(temp1_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar t2_grb = temp2_pvar.getGRBVar(temp2_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//
//
//
//        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
//        //v1=1 : -ep <= x-y <= 0
//        //v1=0 : -M-ep <= x-y <= 0,
//
//
//        final GRBLinExpr minus_M_one_minus_z = new GRBLinExpr();//-M(1-v1)-ep = -M+Mv1 -ep
//        minus_M_one_minus_z.addConstant(-1d*M );
//        minus_M_one_minus_z.addTerm(1d*M, t1_grb);
//        minus_M_one_minus_z.addConstant(-1d*ep);
//
//        final GRBLinExpr zero_grb = new GRBLinExpr();
//        zero_grb.addConstant(0);
//
//        static_grb_model.addConstr( minus_M_one_minus_z, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, zero_grb, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_2" );
//
//
//        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
//        //v2 =1 : 0 <= x-y <= ep
//        //v2=0 : 0 <= x-y <= M+ep
//
//
//        final GRBLinExpr ex_val  = new GRBLinExpr();//M(1-v2)+ep = M-Mv1+ep
//        ex_val.addConstant(M);
//        ex_val.addTerm(-1d*M, t2_grb);
//        ex_val.addConstant(ep);
//
//
//        static_grb_model.addConstr( zero_grb, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, ex_val, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_2" );
//
//
//
//        //Working on Current-phase = @1, and @1.
//
//        GRBVar cur_expr_grbvar =curr_phase.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        GRBVar enum_val_grbvar = e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(cur_expr_grbvar,GRB.EQUAL,enum_val_grbvar,EXPR.name_map.get(curr_phase.toString())+"=="+EXPR.name_map.get(new RDDL.INT_CONST_EXPR(e1.enum_to_int(hmtypes,hm_variables)).toString()));
//
//
//        System.out.println("dkfkdfjdk");
//
//
//
//
//
//
//
//
//
//
//








        //z = [ x == y ]
        // z = v1 OR v2
        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
        //v1=1 : -ep <= x-y <= 0
        //v1=0 : -M-ep <= x-y <= 0,

        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
        //v2 =1 : 0 <= x-y <= ep
        //v2=0 : 0 <= x-y <= M+ep


    }





    public static void testCaseGurobiOr() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());



        //This is checking the case when X1,X2,X3 are integers.

        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "bool", new ArrayList(), true);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "bool", new ArrayList(), true);
        hm_variables.put(p_x2._pName,x2_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.BINARY);
        type_map.put(p_x2._pName,GRB.BINARY);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();



        CONN_EXPR x1_conn_x2 = new CONN_EXPR(p_x1,p_x2,"=>");











        //checking equality "==", X1==X2


        //Adding Default values.
        EXPR def_val1=new RDDL.BOOL_CONST_EXPR(true);
        EXPR def_val2=new RDDL.BOOL_CONST_EXPR(true);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));




        GRBVar eq_var1 =x1_conn_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);

        //static_grb_model.setObjective( new GRBLinExpr() );
        //GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_conn_x2).get(GRB.DoubleAttr.X);


        System.out.println("########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() +"    "+x1_conn_x2.toString()+ "   " +String.valueOf(val1) );





//
//        /////////////////////////////////////////////////////////
//        //die-value(?d) == @1
//
//        //LVAR Arraylist ;
//        LVAR l1 = new LVAR("?d");
//        ArrayList<LVAR> lvars_1 = new ArrayList<>();  lvars_1.add(l1);
//
//        //pvar_expr...
//        PVAR_EXPR die_value_p  = new PVAR_EXPR("die-value",lvars_1);
//        PVAR_EXPR nf_pname     = new PVAR_EXPR("NF",lvars_1);
//        PVAR_EXPR roll_p       = new PVAR_EXPR("roll",lvars_1);
//
//        ENUM_VAL e1 = new ENUM_VAL("@1");
//        ENUM_VAL e2 = new ENUM_VAL("@2");
//        ENUM_VAL e3 = new ENUM_VAL("@3");
//        ENUM_VAL e4 = new ENUM_VAL("@4");
//        ENUM_VAL e5 = new ENUM_VAL("@5");
//        ENUM_VAL e6 = new ENUM_VAL("@6");
//
//
//        //Checking ENUM_VAL isConstant()
//        //hmtypes are defined!!!!.
//        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
//        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
//        array_enum.add(e6);
//        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
//
//        //First Hmtype
//        TYPE_NAME s_int = new TYPE_NAME("small-int");
//        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
//        hmtypes.put(s_int,enum_t_def);
//
//        //second hmtype
//        TYPE_NAME die_t = new TYPE_NAME("die");
//        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
//        hmtypes.put(die_t,die_o_def);
//
//
//        //hmvariables are defined!!!!.
//        //First fluent variable
//        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
//        PVAR_NAME die_value = new PVAR_NAME("die-value");
//        ArrayList<String> ar_string = new ArrayList<>();
//        ar_string.add("die");
//        RDDL.PVARIABLE_STATE_DEF p_s_def = new RDDL.PVARIABLE_STATE_DEF("die-value",false, "small-int", ar_string, e1);
//        hm_variables.put(die_value_p._pName,p_s_def);
//        //Second Non-fluent Variable.
//
//        ArrayList<String> ar_string_1 = new ArrayList<>();
//        ar_string_1.add("die");
//        RDDL.PVARIABLE_STATE_DEF nf_s_def = new RDDL.PVARIABLE_STATE_DEF("NF",true, "small-int", ar_string_1, e1);
//        hm_variables.put(nf_pname._pName,nf_s_def);
//
//        //Third Action Fluent.
//        ArrayList<String> ar_string_2 = new ArrayList<>();
//        ar_string_2.add("roll");
//        RDDL.PVARIABLE_ACTION_DEF ac_def = new RDDL.PVARIABLE_ACTION_DEF("roll","bool",ar_string_2,false);
//        hm_variables.put(roll_p._pName,ac_def);
//
//
//
//        //defining objects////////////////////////////////////////
//        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
//        TYPE_NAME die_type = new TYPE_NAME("die");
//        LCONST lconst_d1   = new OBJECT_VAL("$d1");
//        LCONST lconst_d2   = new OBJECT_VAL("$d2");
//        LCONST lconst_d3   = new OBJECT_VAL("$d3");
//        ArrayList<Object> temp_objects = new ArrayList<>();
//        temp_objects.add(lconst_d1); temp_objects.add(lconst_d2); temp_objects.add(lconst_d3);
//        OBJECTS_DEF ob = new OBJECTS_DEF("die",temp_objects);
//        objects.put(die_type,ob);
//        /////////////////////////////////////////////////////////////////
//        //Defining Constants
//        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
//        Map<ArrayList<LCONST>,Object> cons1 = new HashMap<>();
//        for(int i = 1; i<4 ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(new OBJECT_VAL("$d"+Integer.valueOf(i).toString())); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons1.put(lconst_array,new ENUM_VAL("@"+Integer.valueOf(i).toString()));
//        }
//        constants.put(nf_pname._pName,cons1);
//        //////////////////////////////////////////////////////////////////
//
//
//
//        LTYPED_VAR qd1 = new LTYPED_VAR("?d","die");
//        ArrayList<LTYPED_VAR> array_ltyped = new ArrayList<>();     array_ltyped.add(qd1);
//
//
//        COMP_EXPR e_1  = new COMP_EXPR(die_value_p,e1,"==");
//        COMP_EXPR e_2  = new COMP_EXPR(nf_pname,e1,"==");
//        CONN_EXPR e_3  = new CONN_EXPR(e_1,e_2,"|");
//        QUANT_EXPR e_4 = new QUANT_EXPR("forall",array_ltyped,e_3);
//
//        //e_2.substitute(null,null,objects,hmtypes, hm_variables);
//
//        Map<LVAR, LCONST> subs = new HashMap<>();
//        LVAR a_lvar = new LVAR("?d");
//        LCONST a_cont = new OBJECT_VAL("$d1");
//        subs.put(a_lvar,a_cont);
//        EXPR sub_e_2 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_e_2.getDoubleValue(constants,objects,hmtypes,hm_variables,  null);
//        EXPR sub_expr = nf_pname.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_expr.equals(new ENUM_VAL("@1"));
////        EXPR sub_e1 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//
//        //Adding type_map :
//        Map<PVAR_NAME,Character> type_map = new HashMap<>();
//        type_map.put(die_value_p._pName,GRB.INTEGER);
//
//
//        //This Piece of code for testing Discerte Expression.
//
//
//        //GRBVar gvar = sub_e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//
//        //new RDDL.Discrete("small-int")
//
//        //Defining 1/6
//
//        OPER_EXPR div_6 = new OPER_EXPR(new RDDL.INT_CONST_EXPR(1),new RDDL.INT_CONST_EXPR(6),"/");
//        ArrayList<EXPR> discrete_array = new ArrayList<>();
//        for(int i=1 ; i< 7 ; i++){
//            discrete_array.add(new ENUM_VAL("@"+String.valueOf(i)));
//            discrete_array.add(div_6);
//        }
//
//        Discrete dis = new Discrete("small-int",discrete_array);
//
//        RDDL.IF_EXPR if_else_expr = new RDDL.IF_EXPR(roll_p,dis,die_value_p);
//        RandomDataGenerator rand = new RandomDataGenerator();
//        EXPR test_dis_sample = dis.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //EXPR test_1 =( ((OPER_EXPR) ((OPER_EXPR) ((OPER_EXPR)((OPER_EXPR) ((OPER_EXPR) test_dis_sample)._e1)._e1)._e1)._e1)._e1);
//        //test_1.getDoubleValue(constants,objects,hmtypes,hm_variables);
//        EXPR test_2 = new OPER_EXPR(e1,new REAL_CONST_EXPR(2.0),"*");
//        //test_2.isPiecewiseLinear(constants,objects,hmtypes,hm_variables);
//        //test_2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        System.out.println("dkfjkdjfkdjfkjd");
//        EXPR die_sub_expr = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        //die_sub_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.write("testing.lp");
//
//        ///////////////////////////////////////////////////////////        //FOrall Expression
//        //NF2 adding king
//        ENUM_VAL k_1 = new ENUM_VAL("@Blue");
//        ENUM_VAL k_2 = new ENUM_VAL("@Green");
//        ENUM_VAL k_3 = new ENUM_VAL("@Red");
//        ENUM_VAL k_4 = new ENUM_VAL("@Yellow");
//        ENUM_VAL k_5 = new ENUM_VAL("@Purple");
//        ENUM_VAL k_6 = new ENUM_VAL("@Pink");
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        ArrayList<ENUM_VAL> array_enum1 = new ArrayList<>(); array_enum.add(e1);
//        array_enum1.add(k_1); array_enum1.add(k_2); array_enum1.add(k_3); array_enum1.add(k_4);
//
//
//        //First Hmtype
//        TYPE_NAME color_type = new TYPE_NAME("color");
//        TYPE_DEF enum_color_def = new RDDL.ENUM_TYPE_DEF("color", array_enum1);
//        hmtypes.put(color_type,enum_color_def);
//
//
//        LVAR l_col = new LVAR("?c");
//        ArrayList<LVAR> lvars_col_1 = new ArrayList<>();  lvars_col_1.add(l_col);
//        PVAR_EXPR a_to =new PVAR_EXPR("a-to",lvars_col_1);
//        PVAR_EXPR b_to = new PVAR_EXPR("B-TO",lvars_col_1);
//
//        ArrayList<String> co_string = new ArrayList<>();
//        co_string.add("color");
//
//        RDDL.PVARIABLE_STATE_DEF a_to_def = new RDDL.PVARIABLE_STATE_DEF("a-to",false, "bool", co_string, false);
//        hm_variables.put(a_to._pName,a_to_def);
//
//        RDDL.PVARIABLE_STATE_DEF b_to_def = new RDDL.PVARIABLE_STATE_DEF("B-TO",true, "bool", co_string, false);
//        hm_variables.put(b_to._pName,b_to_def);
//
//        Map<ArrayList<LCONST>,Object> cons2 = new HashMap<>();
//        ArrayList<ENUM_VAL> enum_array = new ArrayList<>();
//        enum_array.add(k_1); enum_array.add(k_2);enum_array.add(k_3);enum_array.add(k_4);
//        for(int i = 0; i<enum_array.size() ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(enum_array.get(i)); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons2.put(lconst_array,true);
//        }
//        constants.put(b_to._pName,cons2);
//
//        LTYPED_VAR colr_type = new LTYPED_VAR("?c","color");
//        ArrayList<LTYPED_VAR> array_ltyped1 = new ArrayList<>();     array_ltyped1.add(colr_type);
//
//
//        EXPR f2 = new CONN_EXPR(a_to,b_to,"=>");
//        EXPR f1 = new QUANT_EXPR("forall",array_ltyped1,f2);
//        EXPR p2 = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        ArrayList<BOOL_EXPR> n_list = new ArrayList<>();
//        n_list.add((BOOL_EXPR) p2); n_list.add(new RDDL.BOOL_CONST_EXPR(true));
//        EXPR f4 = new CONN_EXPR(n_list,"=>");
//        //EXPR f3 = new CONN_EXPR(new RDDL.BOOL_CONST_EXPR(true),new RDDL.BOOL_CONST_EXPR(true),"^");
//
//        EXPR s1 =f1.substitute(Collections.EMPTY_MAP,constants,objects,hmtypes,hm_variables);
//
//        ArrayList<EXPR> discrete_array1 = new ArrayList<>();
//        for(int i=0 ; i< array_enum1.size() ; i++){
//            discrete_array1.add(array_enum1.get(i));
//            discrete_array1.add(div_6);
//        }
//
//        Discrete dis1 = new Discrete("color",discrete_array1);
//        EXPR sam1 =dis1.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //sam1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        EXPR c_m_va = new OPER_EXPR(k_1,new RDDL.BOOL_CONST_EXPR(true),"*");
//        //c_m_va.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //Current Phase Code
//        PVAR_EXPR curr_phase =new PVAR_EXPR("Current-Phase",new ArrayList());
////        ArrayList<String> cur_list = new ArrayList<>();
////        cur_list.add("die");
//        RDDL.PVARIABLE_STATE_DEF cur_phase_psd = new RDDL.PVARIABLE_STATE_DEF("Current-Phase",true, "small-int", new ArrayList(), e1);
//        hm_variables.put(curr_phase._pName,cur_phase_psd);
//
//        COMP_EXPR curr_expr = new COMP_EXPR(curr_phase,e1,"==");
//
//        System.out.println("dkjfkdjfkd");
//
//        PVAR_EXPR temp1_pvar = new PVAR_EXPR("temp1",new ArrayList());
//        PVAR_EXPR temp2_pvar = new PVAR_EXPR("temp2",new ArrayList());
//
//        type_map.put(temp1_pvar._pName,GRB.BINARY);
//        type_map.put(temp2_pvar._pName,GRB.BINARY);
//        type_map.put(curr_phase._pName,GRB.INTEGER);
//        RDDL.PVARIABLE_STATE_DEF temp1_psd = new RDDL.PVARIABLE_STATE_DEF("temp1",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp1_pvar._pName,temp1_psd);
//        RDDL.PVARIABLE_STATE_DEF temp2_psd = new RDDL.PVARIABLE_STATE_DEF("temp2",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp2_pvar._pName,temp2_psd);
//
//
//
//
//        CONN_EXPR or_expr =new CONN_EXPR(temp1_pvar, temp2_pvar,"|");
//
//        //z = v1 or v2
//        GRBVar z = curr_expr.getGRBVar(curr_expr, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar or_grbvar =or_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(z,GRB.EQUAL,or_grbvar,EXPR.name_map.get(curr_expr.toString()) +"==" +EXPR.name_map.get(or_expr.toString()) );
//
//
//        GRBVar t1_grb = temp1_pvar.getGRBVar(temp1_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar t2_grb = temp2_pvar.getGRBVar(temp2_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//
//
//
//        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
//        //v1=1 : -ep <= x-y <= 0
//        //v1=0 : -M-ep <= x-y <= 0,
//
//
//        final GRBLinExpr minus_M_one_minus_z = new GRBLinExpr();//-M(1-v1)-ep = -M+Mv1 -ep
//        minus_M_one_minus_z.addConstant(-1d*M );
//        minus_M_one_minus_z.addTerm(1d*M, t1_grb);
//        minus_M_one_minus_z.addConstant(-1d*ep);
//
//        final GRBLinExpr zero_grb = new GRBLinExpr();
//        zero_grb.addConstant(0);
//
//        static_grb_model.addConstr( minus_M_one_minus_z, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, zero_grb, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_2" );
//
//
//        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
//        //v2 =1 : 0 <= x-y <= ep
//        //v2=0 : 0 <= x-y <= M+ep
//
//
//        final GRBLinExpr ex_val  = new GRBLinExpr();//M(1-v2)+ep = M-Mv1+ep
//        ex_val.addConstant(M);
//        ex_val.addTerm(-1d*M, t2_grb);
//        ex_val.addConstant(ep);
//
//
//        static_grb_model.addConstr( zero_grb, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, ex_val, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_2" );
//
//
//
//        //Working on Current-phase = @1, and @1.
//
//        GRBVar cur_expr_grbvar =curr_phase.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        GRBVar enum_val_grbvar = e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(cur_expr_grbvar,GRB.EQUAL,enum_val_grbvar,EXPR.name_map.get(curr_phase.toString())+"=="+EXPR.name_map.get(new RDDL.INT_CONST_EXPR(e1.enum_to_int(hmtypes,hm_variables)).toString()));
//
//
//        System.out.println("dkfkdfjdk");
//
//
//
//
//
//
//
//
//
//
//








        //z = [ x == y ]
        // z = v1 OR v2
        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
        //v1=1 : -ep <= x-y <= 0
        //v1=0 : -M-ep <= x-y <= 0,

        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
        //v2 =1 : 0 <= x-y <= ep
        //v2=0 : 0 <= x-y <= M+ep


    }




    public static void testCaseGurobiNot() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());



        //This is checking the case when X1,X2,X3 are integers.

        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "bool", new ArrayList(), true);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "bool", new ArrayList(), true);
        hm_variables.put(p_x2._pName,x2_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.BINARY);
        type_map.put(p_x2._pName,GRB.BINARY);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();



        RDDL.NEG_EXPR notX1 = new RDDL.NEG_EXPR(p_x1);









        //checking equality "==", X1==X2


        //Adding Default values.
        EXPR def_val1=new RDDL.BOOL_CONST_EXPR(false);
        //EXPR def_val2=new RDDL.BOOL_CONST_EXPR(true);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
//        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));




        GRBVar eq_var1 =notX1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);

        //static_grb_model.setObjective( new GRBLinExpr() );
        //GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(notX1).get(GRB.DoubleAttr.X);


        System.out.println("########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() +"   "+ notX1.toString()+ "   " +String.valueOf(val1) );





//
//        /////////////////////////////////////////////////////////
//        //die-value(?d) == @1
//
//        //LVAR Arraylist ;
//        LVAR l1 = new LVAR("?d");
//        ArrayList<LVAR> lvars_1 = new ArrayList<>();  lvars_1.add(l1);
//
//        //pvar_expr...
//        PVAR_EXPR die_value_p  = new PVAR_EXPR("die-value",lvars_1);
//        PVAR_EXPR nf_pname     = new PVAR_EXPR("NF",lvars_1);
//        PVAR_EXPR roll_p       = new PVAR_EXPR("roll",lvars_1);
//
//        ENUM_VAL e1 = new ENUM_VAL("@1");
//        ENUM_VAL e2 = new ENUM_VAL("@2");
//        ENUM_VAL e3 = new ENUM_VAL("@3");
//        ENUM_VAL e4 = new ENUM_VAL("@4");
//        ENUM_VAL e5 = new ENUM_VAL("@5");
//        ENUM_VAL e6 = new ENUM_VAL("@6");
//
//
//        //Checking ENUM_VAL isConstant()
//        //hmtypes are defined!!!!.
//        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
//        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
//        array_enum.add(e6);
//        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
//
//        //First Hmtype
//        TYPE_NAME s_int = new TYPE_NAME("small-int");
//        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
//        hmtypes.put(s_int,enum_t_def);
//
//        //second hmtype
//        TYPE_NAME die_t = new TYPE_NAME("die");
//        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
//        hmtypes.put(die_t,die_o_def);
//
//
//        //hmvariables are defined!!!!.
//        //First fluent variable
//        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
//        PVAR_NAME die_value = new PVAR_NAME("die-value");
//        ArrayList<String> ar_string = new ArrayList<>();
//        ar_string.add("die");
//        RDDL.PVARIABLE_STATE_DEF p_s_def = new RDDL.PVARIABLE_STATE_DEF("die-value",false, "small-int", ar_string, e1);
//        hm_variables.put(die_value_p._pName,p_s_def);
//        //Second Non-fluent Variable.
//
//        ArrayList<String> ar_string_1 = new ArrayList<>();
//        ar_string_1.add("die");
//        RDDL.PVARIABLE_STATE_DEF nf_s_def = new RDDL.PVARIABLE_STATE_DEF("NF",true, "small-int", ar_string_1, e1);
//        hm_variables.put(nf_pname._pName,nf_s_def);
//
//        //Third Action Fluent.
//        ArrayList<String> ar_string_2 = new ArrayList<>();
//        ar_string_2.add("roll");
//        RDDL.PVARIABLE_ACTION_DEF ac_def = new RDDL.PVARIABLE_ACTION_DEF("roll","bool",ar_string_2,false);
//        hm_variables.put(roll_p._pName,ac_def);
//
//
//
//        //defining objects////////////////////////////////////////
//        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
//        TYPE_NAME die_type = new TYPE_NAME("die");
//        LCONST lconst_d1   = new OBJECT_VAL("$d1");
//        LCONST lconst_d2   = new OBJECT_VAL("$d2");
//        LCONST lconst_d3   = new OBJECT_VAL("$d3");
//        ArrayList<Object> temp_objects = new ArrayList<>();
//        temp_objects.add(lconst_d1); temp_objects.add(lconst_d2); temp_objects.add(lconst_d3);
//        OBJECTS_DEF ob = new OBJECTS_DEF("die",temp_objects);
//        objects.put(die_type,ob);
//        /////////////////////////////////////////////////////////////////
//        //Defining Constants
//        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
//        Map<ArrayList<LCONST>,Object> cons1 = new HashMap<>();
//        for(int i = 1; i<4 ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(new OBJECT_VAL("$d"+Integer.valueOf(i).toString())); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons1.put(lconst_array,new ENUM_VAL("@"+Integer.valueOf(i).toString()));
//        }
//        constants.put(nf_pname._pName,cons1);
//        //////////////////////////////////////////////////////////////////
//
//
//
//        LTYPED_VAR qd1 = new LTYPED_VAR("?d","die");
//        ArrayList<LTYPED_VAR> array_ltyped = new ArrayList<>();     array_ltyped.add(qd1);
//
//
//        COMP_EXPR e_1  = new COMP_EXPR(die_value_p,e1,"==");
//        COMP_EXPR e_2  = new COMP_EXPR(nf_pname,e1,"==");
//        CONN_EXPR e_3  = new CONN_EXPR(e_1,e_2,"|");
//        QUANT_EXPR e_4 = new QUANT_EXPR("forall",array_ltyped,e_3);
//
//        //e_2.substitute(null,null,objects,hmtypes, hm_variables);
//
//        Map<LVAR, LCONST> subs = new HashMap<>();
//        LVAR a_lvar = new LVAR("?d");
//        LCONST a_cont = new OBJECT_VAL("$d1");
//        subs.put(a_lvar,a_cont);
//        EXPR sub_e_2 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_e_2.getDoubleValue(constants,objects,hmtypes,hm_variables,  null);
//        EXPR sub_expr = nf_pname.substitute(subs,constants,objects,hmtypes,hm_variables);
//        sub_expr.equals(new ENUM_VAL("@1"));
////        EXPR sub_e1 =e_2.substitute(subs,constants,objects,hmtypes,hm_variables);
//
//        //Adding type_map :
//        Map<PVAR_NAME,Character> type_map = new HashMap<>();
//        type_map.put(die_value_p._pName,GRB.INTEGER);
//
//
//        //This Piece of code for testing Discerte Expression.
//
//
//        //GRBVar gvar = sub_e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//
//        //new RDDL.Discrete("small-int")
//
//        //Defining 1/6
//
//        OPER_EXPR div_6 = new OPER_EXPR(new RDDL.INT_CONST_EXPR(1),new RDDL.INT_CONST_EXPR(6),"/");
//        ArrayList<EXPR> discrete_array = new ArrayList<>();
//        for(int i=1 ; i< 7 ; i++){
//            discrete_array.add(new ENUM_VAL("@"+String.valueOf(i)));
//            discrete_array.add(div_6);
//        }
//
//        Discrete dis = new Discrete("small-int",discrete_array);
//
//        RDDL.IF_EXPR if_else_expr = new RDDL.IF_EXPR(roll_p,dis,die_value_p);
//        RandomDataGenerator rand = new RandomDataGenerator();
//        EXPR test_dis_sample = dis.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //EXPR test_1 =( ((OPER_EXPR) ((OPER_EXPR) ((OPER_EXPR)((OPER_EXPR) ((OPER_EXPR) test_dis_sample)._e1)._e1)._e1)._e1)._e1);
//        //test_1.getDoubleValue(constants,objects,hmtypes,hm_variables);
//        EXPR test_2 = new OPER_EXPR(e1,new REAL_CONST_EXPR(2.0),"*");
//        //test_2.isPiecewiseLinear(constants,objects,hmtypes,hm_variables);
//        //test_2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        System.out.println("dkfjkdjfkdjfkjd");
//        EXPR die_sub_expr = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        //die_sub_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.write("testing.lp");
//
//        ///////////////////////////////////////////////////////////        //FOrall Expression
//        //NF2 adding king
//        ENUM_VAL k_1 = new ENUM_VAL("@Blue");
//        ENUM_VAL k_2 = new ENUM_VAL("@Green");
//        ENUM_VAL k_3 = new ENUM_VAL("@Red");
//        ENUM_VAL k_4 = new ENUM_VAL("@Yellow");
//        ENUM_VAL k_5 = new ENUM_VAL("@Purple");
//        ENUM_VAL k_6 = new ENUM_VAL("@Pink");
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        ArrayList<ENUM_VAL> array_enum1 = new ArrayList<>(); array_enum.add(e1);
//        array_enum1.add(k_1); array_enum1.add(k_2); array_enum1.add(k_3); array_enum1.add(k_4);
//
//
//        //First Hmtype
//        TYPE_NAME color_type = new TYPE_NAME("color");
//        TYPE_DEF enum_color_def = new RDDL.ENUM_TYPE_DEF("color", array_enum1);
//        hmtypes.put(color_type,enum_color_def);
//
//
//        LVAR l_col = new LVAR("?c");
//        ArrayList<LVAR> lvars_col_1 = new ArrayList<>();  lvars_col_1.add(l_col);
//        PVAR_EXPR a_to =new PVAR_EXPR("a-to",lvars_col_1);
//        PVAR_EXPR b_to = new PVAR_EXPR("B-TO",lvars_col_1);
//
//        ArrayList<String> co_string = new ArrayList<>();
//        co_string.add("color");
//
//        RDDL.PVARIABLE_STATE_DEF a_to_def = new RDDL.PVARIABLE_STATE_DEF("a-to",false, "bool", co_string, false);
//        hm_variables.put(a_to._pName,a_to_def);
//
//        RDDL.PVARIABLE_STATE_DEF b_to_def = new RDDL.PVARIABLE_STATE_DEF("B-TO",true, "bool", co_string, false);
//        hm_variables.put(b_to._pName,b_to_def);
//
//        Map<ArrayList<LCONST>,Object> cons2 = new HashMap<>();
//        ArrayList<ENUM_VAL> enum_array = new ArrayList<>();
//        enum_array.add(k_1); enum_array.add(k_2);enum_array.add(k_3);enum_array.add(k_4);
//        for(int i = 0; i<enum_array.size() ; i++){
//            ArrayList<LCONST> lconst_array = new ArrayList<>();
//            lconst_array.add(enum_array.get(i)); //lconst_array.add(new OBJECT_VAL("$t2")); lconst_array.add(new OBJECT_VAL("$t3"));
//            cons2.put(lconst_array,true);
//        }
//        constants.put(b_to._pName,cons2);
//
//        LTYPED_VAR colr_type = new LTYPED_VAR("?c","color");
//        ArrayList<LTYPED_VAR> array_ltyped1 = new ArrayList<>();     array_ltyped1.add(colr_type);
//
//
//        EXPR f2 = new CONN_EXPR(a_to,b_to,"=>");
//        EXPR f1 = new QUANT_EXPR("forall",array_ltyped1,f2);
//        EXPR p2 = die_value_p.substitute(subs,constants,objects,hmtypes,hm_variables);
//        ArrayList<BOOL_EXPR> n_list = new ArrayList<>();
//        n_list.add((BOOL_EXPR) p2); n_list.add(new RDDL.BOOL_CONST_EXPR(true));
//        EXPR f4 = new CONN_EXPR(n_list,"=>");
//        //EXPR f3 = new CONN_EXPR(new RDDL.BOOL_CONST_EXPR(true),new RDDL.BOOL_CONST_EXPR(true),"^");
//
//        EXPR s1 =f1.substitute(Collections.EMPTY_MAP,constants,objects,hmtypes,hm_variables);
//
//        ArrayList<EXPR> discrete_array1 = new ArrayList<>();
//        for(int i=0 ; i< array_enum1.size() ; i++){
//            discrete_array1.add(array_enum1.get(i));
//            discrete_array1.add(div_6);
//        }
//
//        Discrete dis1 = new Discrete("color",discrete_array1);
//        EXPR sam1 =dis1.sampleDeterminization(rand,constants,objects,hmtypes,hm_variables);
//        //sam1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        EXPR c_m_va = new OPER_EXPR(k_1,new RDDL.BOOL_CONST_EXPR(true),"*");
//        //c_m_va.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //Current Phase Code
//        PVAR_EXPR curr_phase =new PVAR_EXPR("Current-Phase",new ArrayList());
////        ArrayList<String> cur_list = new ArrayList<>();
////        cur_list.add("die");
//        RDDL.PVARIABLE_STATE_DEF cur_phase_psd = new RDDL.PVARIABLE_STATE_DEF("Current-Phase",true, "small-int", new ArrayList(), e1);
//        hm_variables.put(curr_phase._pName,cur_phase_psd);
//
//        COMP_EXPR curr_expr = new COMP_EXPR(curr_phase,e1,"==");
//
//        System.out.println("dkjfkdjfkd");
//
//        PVAR_EXPR temp1_pvar = new PVAR_EXPR("temp1",new ArrayList());
//        PVAR_EXPR temp2_pvar = new PVAR_EXPR("temp2",new ArrayList());
//
//        type_map.put(temp1_pvar._pName,GRB.BINARY);
//        type_map.put(temp2_pvar._pName,GRB.BINARY);
//        type_map.put(curr_phase._pName,GRB.INTEGER);
//        RDDL.PVARIABLE_STATE_DEF temp1_psd = new RDDL.PVARIABLE_STATE_DEF("temp1",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp1_pvar._pName,temp1_psd);
//        RDDL.PVARIABLE_STATE_DEF temp2_psd = new RDDL.PVARIABLE_STATE_DEF("temp2",true, "bool", new ArrayList(), false);
//        hm_variables.put(temp2_pvar._pName,temp2_psd);
//
//
//
//
//        CONN_EXPR or_expr =new CONN_EXPR(temp1_pvar, temp2_pvar,"|");
//
//        //z = v1 or v2
//        GRBVar z = curr_expr.getGRBVar(curr_expr, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar or_grbvar =or_expr.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(z,GRB.EQUAL,or_grbvar,EXPR.name_map.get(curr_expr.toString()) +"==" +EXPR.name_map.get(or_expr.toString()) );
//
//
//        GRBVar t1_grb = temp1_pvar.getGRBVar(temp1_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//        GRBVar t2_grb = temp2_pvar.getGRBVar(temp2_pvar, static_grb_model, constants, objects, type_map , hmtypes, hm_variables);
//
//
//
//        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
//        //v1=1 : -ep <= x-y <= 0
//        //v1=0 : -M-ep <= x-y <= 0,
//
//
//        final GRBLinExpr minus_M_one_minus_z = new GRBLinExpr();//-M(1-v1)-ep = -M+Mv1 -ep
//        minus_M_one_minus_z.addConstant(-1d*M );
//        minus_M_one_minus_z.addTerm(1d*M, t1_grb);
//        minus_M_one_minus_z.addConstant(-1d*ep);
//
//        final GRBLinExpr zero_grb = new GRBLinExpr();
//        zero_grb.addConstant(0);
//
//        static_grb_model.addConstr( minus_M_one_minus_z, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, zero_grb, EXPR.name_map.get(temp1_pvar.toString())+"_LEQ_2" );
//
//
//        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
//        //v2 =1 : 0 <= x-y <= ep
//        //v2=0 : 0 <= x-y <= M+ep
//
//
//        final GRBLinExpr ex_val  = new GRBLinExpr();//M(1-v2)+ep = M-Mv1+ep
//        ex_val.addConstant(M);
//        ex_val.addTerm(-1d*M, t2_grb);
//        ex_val.addConstant(ep);
//
//
//        static_grb_model.addConstr( zero_grb, GRB.LESS_EQUAL, z, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_1" );
//        static_grb_model.addConstr( z, GRB.LESS_EQUAL, ex_val, EXPR.name_map.get(temp2_pvar.toString())+"_LEQ_2" );
//
//
//
//        //Working on Current-phase = @1, and @1.
//
//        GRBVar cur_expr_grbvar =curr_phase.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//        GRBVar enum_val_grbvar = e1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
//
//        static_grb_model.addConstr(cur_expr_grbvar,GRB.EQUAL,enum_val_grbvar,EXPR.name_map.get(curr_phase.toString())+"=="+EXPR.name_map.get(new RDDL.INT_CONST_EXPR(e1.enum_to_int(hmtypes,hm_variables)).toString()));
//
//
//        System.out.println("dkfkdfjdk");
//
//
//
//
//
//
//
//
//
//
//








        //z = [ x == y ]
        // z = v1 OR v2
        //v1 : -M(1-v1) -ep<= x-y <= 0, v1 in 0,1
        //v1=1 : -ep <= x-y <= 0
        //v1=0 : -M-ep <= x-y <= 0,

        //v2 : 0 <= x-y <= M(1-v2)+ep, v2 in 0, 1
        //v2 =1 : 0 <= x-y <= ep
        //v2=0 : 0 <= x-y <= M+ep


    }


    public static void testCaseGurobiMinInt() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());


        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.INTEGER);
        type_map.put(p_x2._pName,GRB.INTEGER);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();

        OPER_EXPR x1_oper_x2 = new OPER_EXPR(p_x1,p_x2,"min");

        //Adding Default values.
        EXPR def_val1=new RDDL.INT_CONST_EXPR(200);
        EXPR def_val2=new RDDL.INT_CONST_EXPR(200);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));

        x1_oper_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);



        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_oper_x2).get(GRB.DoubleAttr.X);

        System.out.println("#########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() +"    "+x1_oper_x2.toString()+ "   " +String.valueOf(val1) );











    }


    public static void testCaseGurobiMaxInt() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());


        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.INTEGER);
        type_map.put(p_x2._pName,GRB.INTEGER);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();

        OPER_EXPR x1_oper_x2 = new OPER_EXPR(p_x1,p_x2,"max");

        //Adding Default values.
        EXPR def_val1=new RDDL.INT_CONST_EXPR(-200);
        EXPR def_val2=new RDDL.INT_CONST_EXPR(-1000);
        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));

        x1_oper_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);



        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_oper_x2).get(GRB.DoubleAttr.X);

        System.out.println("#########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() +"    "+x1_oper_x2.toString()+ "   " +String.valueOf(val1) );











    }



    public static void testCaseGurobiFunMinReal() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());
        PVAR_EXPR p_x3 = new PVAR_EXPR("X3",new ArrayList());
        PVAR_EXPR p_x4 = new PVAR_EXPR("X4",new ArrayList());



        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);

        RDDL.PVARIABLE_STATE_DEF x3_s_def = new RDDL.PVARIABLE_STATE_DEF("X3",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x3._pName,x3_s_def);


        RDDL.PVARIABLE_STATE_DEF x4_s_def = new RDDL.PVARIABLE_STATE_DEF("X4",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x4._pName,x4_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.CONTINUOUS);
        type_map.put(p_x2._pName,GRB.CONTINUOUS);
        type_map.put(p_x3._pName,GRB.CONTINUOUS);
        type_map.put(p_x4._pName,GRB.CONTINUOUS);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
        ArrayList<EXPR>ar_exprs  = new ArrayList<EXPR>();
        ar_exprs.add(p_x1); ar_exprs.add(p_x2); ar_exprs.add(p_x3);
        RDDL.FUN_EXPR x1_fun_x2_x3 = new RDDL.FUN_EXPR("min",ar_exprs);
        COMP_EXPR x4_comp_fun = new COMP_EXPR(p_x4,x1_fun_x2_x3,"==");

        //Adding Default values.
        EXPR def_val1=new RDDL.REAL_CONST_EXPR(200.0);
        EXPR def_val2=new RDDL.REAL_CONST_EXPR(1000.0);
        EXPR def_val3=new RDDL.REAL_CONST_EXPR(20000.0);
        EXPR def_val4=new RDDL.REAL_CONST_EXPR(200.0);

        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));

        GRBVar x3_var_lhs = p_x3.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x3_var_rhs = def_val3.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x3_var_lhs,GRB.EQUAL,x3_var_rhs, name_map.get(p_x3.toString()) +"="+ name_map.get(def_val3.toString()));

        GRBVar x4_var_lhs = p_x4.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x4_var_rhs = def_val4.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x4_var_lhs,GRB.EQUAL,x4_var_rhs, name_map.get(p_x4.toString()) +"="+ name_map.get(def_val4.toString()));



        x1_fun_x2_x3.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        x4_comp_fun.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_fun_x2_x3).get(GRB.DoubleAttr.X);
        double val2 =grb_cache.get(x4_comp_fun).get(GRB.DoubleAttr.X);

        System.out.println("#########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() + "   x3 : "+ def_val3.toString()+"    "+x1_fun_x2_x3.toString()+ "   " +String.valueOf(val1) );
        System.out.println("x4 : "+def_val4.toString() +"   " + x4_comp_fun.toString() + "   "+ String.valueOf(val2));











    }



    public static void testCaseGurobiFunMaxReal() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());
        PVAR_EXPR p_x3 = new PVAR_EXPR("X3",new ArrayList());
        PVAR_EXPR p_x4 = new PVAR_EXPR("X4",new ArrayList());



        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);

        RDDL.PVARIABLE_STATE_DEF x3_s_def = new RDDL.PVARIABLE_STATE_DEF("X3",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x3._pName,x3_s_def);


        RDDL.PVARIABLE_STATE_DEF x4_s_def = new RDDL.PVARIABLE_STATE_DEF("X4",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x4._pName,x4_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.CONTINUOUS);
        type_map.put(p_x2._pName,GRB.CONTINUOUS);
        type_map.put(p_x3._pName,GRB.CONTINUOUS);
        type_map.put(p_x4._pName,GRB.CONTINUOUS);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
        ArrayList<EXPR>ar_exprs  = new ArrayList<EXPR>();
        ar_exprs.add(p_x1); ar_exprs.add(p_x2); ar_exprs.add(p_x3);
        RDDL.FUN_EXPR x1_fun_x2_x3 = new RDDL.FUN_EXPR("max",ar_exprs);
        COMP_EXPR x4_comp_fun = new COMP_EXPR(p_x4,x1_fun_x2_x3,"==");

        //Adding Default values.
        EXPR def_val1=new RDDL.REAL_CONST_EXPR(200.0);
        EXPR def_val2=new RDDL.REAL_CONST_EXPR(1000.0);
        EXPR def_val3=new RDDL.REAL_CONST_EXPR(20000.0);
        EXPR def_val4=new RDDL.REAL_CONST_EXPR(20000.0);

        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));

        GRBVar x3_var_lhs = p_x3.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x3_var_rhs = def_val3.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x3_var_lhs,GRB.EQUAL,x3_var_rhs, name_map.get(p_x3.toString()) +"="+ name_map.get(def_val3.toString()));

        GRBVar x4_var_lhs = p_x4.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x4_var_rhs = def_val4.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x4_var_lhs,GRB.EQUAL,x4_var_rhs, name_map.get(p_x4.toString()) +"="+ name_map.get(def_val4.toString()));



        x1_fun_x2_x3.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        x4_comp_fun.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_fun_x2_x3).get(GRB.DoubleAttr.X);
        double val2 =grb_cache.get(x4_comp_fun).get(GRB.DoubleAttr.X);

        System.out.println("#########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() + "   x3 : "+ def_val3.toString()+"    "+x1_fun_x2_x3.toString()+ "   " +String.valueOf(val1) );
        System.out.println("x4 : "+def_val4.toString() +"   " + x4_comp_fun.toString() + "   "+ String.valueOf(val2));











    }



    public static void testCaseGurobiFunMaxInt() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());
        PVAR_EXPR p_x3 = new PVAR_EXPR("X3",new ArrayList());
        PVAR_EXPR p_x4 = new PVAR_EXPR("X4",new ArrayList());



        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);

        RDDL.PVARIABLE_STATE_DEF x3_s_def = new RDDL.PVARIABLE_STATE_DEF("X3",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x3._pName,x3_s_def);


        RDDL.PVARIABLE_STATE_DEF x4_s_def = new RDDL.PVARIABLE_STATE_DEF("X4",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x4._pName,x4_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.INTEGER);
        type_map.put(p_x2._pName,GRB.INTEGER);
        type_map.put(p_x3._pName,GRB.INTEGER);
        type_map.put(p_x4._pName,GRB.INTEGER);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
        ArrayList<EXPR>ar_exprs  = new ArrayList<EXPR>();
        ar_exprs.add(p_x1); ar_exprs.add(p_x2); ar_exprs.add(p_x3);
        RDDL.FUN_EXPR x1_fun_x2_x3 = new RDDL.FUN_EXPR("max",ar_exprs);
        COMP_EXPR x4_comp_fun = new COMP_EXPR(p_x4,x1_fun_x2_x3,"==");

        //Adding Default values.
        EXPR def_val1=new RDDL.INT_CONST_EXPR(200);
        EXPR def_val2=new RDDL.INT_CONST_EXPR(1000);
        EXPR def_val3=new RDDL.INT_CONST_EXPR(20000);
        EXPR def_val4=new RDDL.INT_CONST_EXPR(20000);

        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));

        GRBVar x3_var_lhs = p_x3.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x3_var_rhs = def_val3.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x3_var_lhs,GRB.EQUAL,x3_var_rhs, name_map.get(p_x3.toString()) +"="+ name_map.get(def_val3.toString()));

        GRBVar x4_var_lhs = p_x4.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x4_var_rhs = def_val4.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x4_var_lhs,GRB.EQUAL,x4_var_rhs, name_map.get(p_x4.toString()) +"="+ name_map.get(def_val4.toString()));



        x1_fun_x2_x3.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        x4_comp_fun.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_fun_x2_x3).get(GRB.DoubleAttr.X);
        double val2 =grb_cache.get(x4_comp_fun).get(GRB.DoubleAttr.X);

        System.out.println("#########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() + "   x3 : "+ def_val3.toString()+"    "+x1_fun_x2_x3.toString()+ "   " +String.valueOf(val1) );
        System.out.println("x4 : "+def_val4.toString() +"   " + x4_comp_fun.toString() + "   "+ String.valueOf(val2));











    }



    public static void testCaseGurobiFunMinInt() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());
        PVAR_EXPR p_x3 = new PVAR_EXPR("X3",new ArrayList());
        PVAR_EXPR p_x4 = new PVAR_EXPR("X4",new ArrayList());



        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);

        RDDL.PVARIABLE_STATE_DEF x3_s_def = new RDDL.PVARIABLE_STATE_DEF("X3",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x3._pName,x3_s_def);


        RDDL.PVARIABLE_STATE_DEF x4_s_def = new RDDL.PVARIABLE_STATE_DEF("X4",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x4._pName,x4_s_def);

        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.INTEGER);
        type_map.put(p_x2._pName,GRB.INTEGER);
        type_map.put(p_x3._pName,GRB.INTEGER);
        type_map.put(p_x4._pName,GRB.INTEGER);


        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
        ArrayList<EXPR>ar_exprs  = new ArrayList<EXPR>();
        ar_exprs.add(p_x1); ar_exprs.add(p_x2); ar_exprs.add(p_x3);
        RDDL.FUN_EXPR x1_fun_x2_x3 = new RDDL.FUN_EXPR("min",ar_exprs);
        COMP_EXPR x4_comp_fun = new COMP_EXPR(p_x4,x1_fun_x2_x3,"==");

        //Adding Default values.
        EXPR def_val1=new RDDL.INT_CONST_EXPR(200);
        EXPR def_val2=new RDDL.INT_CONST_EXPR(1000);
        EXPR def_val3=new RDDL.INT_CONST_EXPR(20000);
        EXPR def_val4=new RDDL.INT_CONST_EXPR(2000);

        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));

        GRBVar x3_var_lhs = p_x3.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x3_var_rhs = def_val3.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x3_var_lhs,GRB.EQUAL,x3_var_rhs, name_map.get(p_x3.toString()) +"="+ name_map.get(def_val3.toString()));

        GRBVar x4_var_lhs = p_x4.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x4_var_rhs = def_val4.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x4_var_lhs,GRB.EQUAL,x4_var_rhs, name_map.get(p_x4.toString()) +"="+ name_map.get(def_val4.toString()));



        x1_fun_x2_x3.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        x4_comp_fun.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_fun_x2_x3).get(GRB.DoubleAttr.X);
        double val2 =grb_cache.get(x4_comp_fun).get(GRB.DoubleAttr.X);

        System.out.println("#########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() + "   x3 : "+ def_val3.toString()+"    "+x1_fun_x2_x3.toString()+ "   " +String.valueOf(val1) );
        System.out.println("x4 : "+def_val4.toString() +"   " + x4_comp_fun.toString() + "   "+ String.valueOf(val2));











    }


    public static void testCaseGurobiFunAbsInt() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());



        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);


        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.INTEGER);
        type_map.put(p_x2._pName,GRB.INTEGER);



        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
        ArrayList<EXPR>ar_exprs  = new ArrayList<EXPR>();
        ar_exprs.add(p_x1);
        RDDL.FUN_EXPR x1_fun = new RDDL.FUN_EXPR("abs",ar_exprs);
        COMP_EXPR x1_comp_fun = new COMP_EXPR(p_x2,x1_fun,"==");

        //Adding Default values.
        EXPR def_val1=new RDDL.INT_CONST_EXPR(-200);
        EXPR def_val2=new RDDL.INT_CONST_EXPR(1000);


        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));



        x1_fun.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        x1_comp_fun.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_fun).get(GRB.DoubleAttr.X);
        double val2 =grb_cache.get(x1_comp_fun).get(GRB.DoubleAttr.X);

        System.out.println("#########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() +  "    "+x1_comp_fun.toString()+ "   " +String.valueOf(val2) +"    "+x1_fun.toString()+ "  "+ String.valueOf(val1) );












    }



    public static void testCaseGurobiFunAbsReal() throws Exception{

        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());



        TYPE_NAME int_type = new TYPE_NAME("int");
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "int", new ArrayList(), 1);
        hm_variables.put(p_x2._pName,x2_s_def);


        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.CONTINUOUS);
        type_map.put(p_x2._pName,GRB.CONTINUOUS);



        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
        ArrayList<EXPR>ar_exprs  = new ArrayList<EXPR>();
        ar_exprs.add(p_x1);
        RDDL.FUN_EXPR x1_fun = new RDDL.FUN_EXPR("abs",ar_exprs);
        COMP_EXPR x1_comp_fun = new COMP_EXPR(p_x2,x1_fun,"==");

        //Adding Default values.
        EXPR def_val1=new RDDL.REAL_CONST_EXPR(-200.0);
        EXPR def_val2=new RDDL.REAL_CONST_EXPR(200.0);


        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get(def_val1.toString()));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get(def_val2.toString()));



        x1_fun.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        x1_comp_fun.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_fun).get(GRB.DoubleAttr.X);
        double val2 =grb_cache.get(x1_comp_fun).get(GRB.DoubleAttr.X);

        System.out.println("#########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() +  "    "+x1_comp_fun.toString()+ "   " +String.valueOf(val2) +"    "+x1_fun.toString()+ "  "+ String.valueOf(val1) );












    }


    public static void testCaseGurobiEnumIntEq() throws Exception{
        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());

        //This seciton is for ENUM
        ENUM_VAL e1 = new ENUM_VAL("@1");
        ENUM_VAL e2 = new ENUM_VAL("@2");
        ENUM_VAL e3 = new ENUM_VAL("@3");
        ENUM_VAL e4 = new ENUM_VAL("@4");
        ENUM_VAL e5 = new ENUM_VAL("@5");
        ENUM_VAL e6 = new ENUM_VAL("@6");

        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
        array_enum.add(e6);
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();

        //First Hmtype
        TYPE_NAME s_int = new TYPE_NAME("small-int");
        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
        hmtypes.put(s_int,enum_t_def);
        //End of ENUM_Section


        TYPE_NAME int_type = new TYPE_NAME("int");
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "small-int", new ArrayList(), e1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "small-int", new ArrayList(), e1);
        hm_variables.put(p_x2._pName,x2_s_def);


        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.INTEGER);
        type_map.put(p_x2._pName,GRB.INTEGER);



        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
        COMP_EXPR x1_comp_x2 = new COMP_EXPR(p_x1,p_x2,"==");



        //Adding Default values.
        EXPR def_val1=e1;
        EXPR def_val2=e2;


        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get( new RDDL.INT_CONST_EXPR(((ENUM_VAL)def_val1).enum_to_int(hmtypes,hm_variables)).toString() ));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get( new RDDL.INT_CONST_EXPR(((ENUM_VAL)def_val2).enum_to_int(hmtypes,hm_variables)).toString() ));



        x1_comp_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_comp_x2).get(GRB.DoubleAttr.X);

        System.out.println("#########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() +  "    "+x1_comp_x2.toString()+ "   " +String.valueOf(val1) );











    }



    public static void testCaseGurobiEnumNonIntEq() throws Exception{
        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());

        //This seciton is for ENUM
        ENUM_VAL e1 = new ENUM_VAL("@Red");
        ENUM_VAL e2 = new ENUM_VAL("@Blue");
        ENUM_VAL e3 = new ENUM_VAL("@Yellow");
        ENUM_VAL e4 = new ENUM_VAL("@Purple");
        ENUM_VAL e5 = new ENUM_VAL("@Pink");
        ENUM_VAL e6 = new ENUM_VAL("@Orange");

        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
        array_enum.add(e6);
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();

        //First Hmtype
        TYPE_NAME s_int = new TYPE_NAME("small-int");
        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
        hmtypes.put(s_int,enum_t_def);
        //End of ENUM_Section


        TYPE_NAME int_type = new TYPE_NAME("int");
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "small-int", new ArrayList(), e1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "small-int", new ArrayList(), e1);
        hm_variables.put(p_x2._pName,x2_s_def);


        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.INTEGER);
        type_map.put(p_x2._pName,GRB.INTEGER);



        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
        COMP_EXPR x1_comp_x2 = new COMP_EXPR(p_x1,p_x2,"==");



        //Adding Default values.
        EXPR def_val1=e1;
        EXPR def_val2=e1;


        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get( new RDDL.INT_CONST_EXPR(((ENUM_VAL)def_val1).enum_to_int(hmtypes,hm_variables)).toString() ));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get( new RDDL.INT_CONST_EXPR(((ENUM_VAL)def_val2).enum_to_int(hmtypes,hm_variables)).toString() ));



        x1_comp_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        //static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        //GRBLinExpr all_sum = new GRBLinExpr();
        //all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_x2));
        //static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_comp_x2).get(GRB.DoubleAttr.X);

        System.out.println("#########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() +  "    "+x1_comp_x2.toString()+ "   " +String.valueOf(val1) );











    }




    public static void testCaseGurobiEnumNoIntNotEq() throws Exception{
        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());

        //This seciton is for ENUM
        ENUM_VAL e1 = new ENUM_VAL("@Red");
        ENUM_VAL e2 = new ENUM_VAL("@Blue");
        ENUM_VAL e3 = new ENUM_VAL("@Yellow");
        ENUM_VAL e4 = new ENUM_VAL("@Purple");
        ENUM_VAL e5 = new ENUM_VAL("@Pink");
        ENUM_VAL e6 = new ENUM_VAL("@Orange");

        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
        array_enum.add(e6);
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();

        //First Hmtype
        TYPE_NAME s_int = new TYPE_NAME("small-int");
        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
        hmtypes.put(s_int,enum_t_def);
        //End of ENUM_Section


        TYPE_NAME int_type = new TYPE_NAME("int");
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "small-int", new ArrayList(), e1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "small-int", new ArrayList(), e1);
        hm_variables.put(p_x2._pName,x2_s_def);


        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.INTEGER);
        type_map.put(p_x2._pName,GRB.INTEGER);



        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
        COMP_EXPR x1_comp_x2 = new COMP_EXPR(p_x1,p_x2,"~=");
        COMP_EXPR x1_eq_x2   = new COMP_EXPR(p_x1,p_x2,"==");


        //Adding Default values.
        EXPR def_val1=e1;
        EXPR def_val2=e2;


        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get( new RDDL.INT_CONST_EXPR(((ENUM_VAL)def_val1).enum_to_int(hmtypes,hm_variables)).toString() ));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get( new RDDL.INT_CONST_EXPR(((ENUM_VAL)def_val2).enum_to_int(hmtypes,hm_variables)).toString() ));



        x1_comp_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        GRBLinExpr all_sum = new GRBLinExpr();
        all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_eq_x2));
        static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_comp_x2).get(GRB.DoubleAttr.X);

        System.out.println("#########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() +  "    "+x1_comp_x2.toString()+ "   " +String.valueOf(val1) );











    }



    public static void testCaseGurobiEnumIntNotEq() throws Exception{
        initalizeGurobi();
        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());

        //This seciton is for ENUM
        ENUM_VAL e1 = new ENUM_VAL("@1");
        ENUM_VAL e2 = new ENUM_VAL("@2");
        ENUM_VAL e3 = new ENUM_VAL("@3");
        ENUM_VAL e4 = new ENUM_VAL("@4");
        ENUM_VAL e5 = new ENUM_VAL("@5");
        ENUM_VAL e6 = new ENUM_VAL("@6");

        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
        array_enum.add(e6);
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();

        //First Hmtype
        TYPE_NAME s_int = new TYPE_NAME("small-int");
        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
        hmtypes.put(s_int,enum_t_def);
        //End of ENUM_Section


        TYPE_NAME int_type = new TYPE_NAME("int");
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "small-int", new ArrayList(), e1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "small-int", new ArrayList(), e1);
        hm_variables.put(p_x2._pName,x2_s_def);


        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.INTEGER);
        type_map.put(p_x2._pName,GRB.INTEGER);



        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
        COMP_EXPR x1_comp_x2 = new COMP_EXPR(p_x1,p_x2,"~=");
        COMP_EXPR x1_eq_x2   = new COMP_EXPR(p_x1,p_x2,"==");


        //Adding Default values.
        EXPR def_val1=e1;
        EXPR def_val2=e2;


        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get( new RDDL.INT_CONST_EXPR(((ENUM_VAL)def_val1).enum_to_int(hmtypes,hm_variables)).toString() ));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get( new RDDL.INT_CONST_EXPR(((ENUM_VAL)def_val2).enum_to_int(hmtypes,hm_variables)).toString() ));



        x1_comp_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        GRBLinExpr all_sum = new GRBLinExpr();
        all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_eq_x2));
        static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_comp_x2).get(GRB.DoubleAttr.X);

        System.out.println("#########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() +  "    "+x1_comp_x2.toString()+ "   " +String.valueOf(val1) );











    }









    public static void testCaseIndicator() throws Exception{
        initalizeGurobi();

        //a*b where a=indicator(x==1) and b=x
//        obj is max x
//        such that I(x < 0)*x +5 <= 3
//        x= -2  is also a solution
//        will check












        PVAR_EXPR p_x1 = new PVAR_EXPR("X1",new ArrayList());
        PVAR_EXPR p_x2 = new PVAR_EXPR("X2",new ArrayList());

        //This seciton is for ENUM
        ENUM_VAL e1 = new ENUM_VAL("@1");
        ENUM_VAL e2 = new ENUM_VAL("@2");
        ENUM_VAL e3 = new ENUM_VAL("@3");
        ENUM_VAL e4 = new ENUM_VAL("@4");
        ENUM_VAL e5 = new ENUM_VAL("@5");
        ENUM_VAL e6 = new ENUM_VAL("@6");

        ArrayList<ENUM_VAL> array_enum = new ArrayList<>(); array_enum.add(e1);
        array_enum.add(e2); array_enum.add(e3); array_enum.add(e4); array_enum.add(e5);
        array_enum.add(e6);
        HashMap<TYPE_NAME, TYPE_DEF> hmtypes = new HashMap<>();

        //First Hmtype
        TYPE_NAME s_int = new TYPE_NAME("small-int");
        TYPE_DEF enum_t_def = new RDDL.ENUM_TYPE_DEF("small-int", array_enum);
        hmtypes.put(s_int,enum_t_def);
        //End of ENUM_Section


        TYPE_NAME int_type = new TYPE_NAME("int");
        RDDL.OBJECT_TYPE_DEF die_o_def = new RDDL.OBJECT_TYPE_DEF("die");
        hmtypes.put(int_type,die_o_def);

        //Adding hm_variables
        HashMap<PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables = new HashMap<>();
        RDDL.PVARIABLE_STATE_DEF x1_s_def = new RDDL.PVARIABLE_STATE_DEF("X1",false, "small-int", new ArrayList(), e1);
        hm_variables.put(p_x1._pName,x1_s_def);

        RDDL.PVARIABLE_STATE_DEF x2_s_def = new RDDL.PVARIABLE_STATE_DEF("X2",false, "small-int", new ArrayList(), e1);
        hm_variables.put(p_x2._pName,x2_s_def);


        //adding type_map
        Map<PVAR_NAME,Character> type_map = new HashMap<>();
        type_map.put(p_x1._pName,GRB.INTEGER);
        type_map.put(p_x2._pName,GRB.INTEGER);



        //Adding constants and objects
        Map<PVAR_NAME, Map<ArrayList<LCONST>, Object>> constants = new HashMap<>();
        Map<TYPE_NAME, OBJECTS_DEF> objects = new HashMap<>();
        COMP_EXPR x1_comp_x2 = new COMP_EXPR(p_x1,p_x2,"~=");
        COMP_EXPR x1_eq_x2   = new COMP_EXPR(p_x1,p_x2,"==");


        //Adding Default values.
        EXPR def_val1=e1;
        EXPR def_val2=e2;


        GRBVar x1_var_lhs = p_x1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x1_var_rhs = def_val1.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x1_var_lhs,GRB.EQUAL,x1_var_rhs, name_map.get(p_x1.toString()) +"="+ name_map.get( new RDDL.INT_CONST_EXPR(((ENUM_VAL)def_val1).enum_to_int(hmtypes,hm_variables)).toString() ));
        GRBVar x2_var_lhs = p_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        GRBVar x2_var_rhs = def_val2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);
        static_grb_model.addConstr(x2_var_lhs,GRB.EQUAL,x2_var_rhs, name_map.get(p_x2.toString()) +"="+ name_map.get( new RDDL.INT_CONST_EXPR(((ENUM_VAL)def_val2).enum_to_int(hmtypes,hm_variables)).toString() ));



        x1_comp_x2.getGRBConstr(GRB.EQUAL,static_grb_model,constants,objects,type_map,hmtypes,hm_variables);


        static_grb_model.setObjective( new GRBLinExpr() );
        GRBExpr old_obj = static_grb_model.getObjective();
        GRBLinExpr all_sum = new GRBLinExpr();
        all_sum.addTerm(1.0d,EXPR.grb_cache.get(x1_eq_x2));
        static_grb_model.setObjective(all_sum);
        static_grb_model.write("testing_eq_int.lp");

        static_grb_model.optimize();
        double val1 =grb_cache.get(x1_comp_x2).get(GRB.DoubleAttr.X);

        System.out.println("#########################################################");
        System.out.println("This is the value of  x1 : " +def_val1.toString() + "   x2 : "+ def_val2.toString() +  "    "+x1_comp_x2.toString()+ "   " +String.valueOf(val1) );











    }






    public static void main(String[] args)throws Exception{


        //This is checking the case when X1,X2,X3 are integers.

        testCaseGurobiOPER();





    }


}
