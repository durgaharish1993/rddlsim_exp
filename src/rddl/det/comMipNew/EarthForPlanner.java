package rddl.det.comMipNew;

import gurobi.GRB;
import org.apache.commons.math3.random.RandomDataGenerator;
import org.apache.commons.math3.random.RandomGenerator;
import org.rosuda.JRI.REXP;
import org.rosuda.JRI.Rengine;
import rddl.EvalException;
import rddl.RDDL;
import rddl.State;
import util.Pair;

import java.io.*;
import java.lang.reflect.Array;
import java.text.NumberFormat;
import java.util.*;

import rddl.RDDL.EXPR;

import javax.script.ScriptException;

public class EarthForPlanner {

    protected boolean PRINT_TO_R_FILE = true;
    protected String R_FILE_NAME = "model.R";
    protected  Writer writer = null;
    protected Writer output_writer = null;
    protected boolean WRITE_NPWL_TO_PWL_EARTH =true;
    protected String EARTH_OUT_FILE_NAME="earth_reconstruction_file.txt";
    protected String REAL     = "real";
    protected String INTEGER  = "int";
    protected String ENUM     = "enum";
    protected String BOOL     = "bool";


    public  EarthForPlanner() throws Exception {
        if(WRITE_NPWL_TO_PWL_EARTH){
            output_writer = new BufferedWriter(new OutputStreamWriter(
                    new FileOutputStream(EARTH_OUT_FILE_NAME), "utf-8"));
            output_writer.write("---------------Start of the R Approximation \n");
        }

    }


    public EXPR fitPWL(RDDL.PVAR_NAME target_pVar, ArrayList<RDDL.LCONST> target_lconsts, boolean reward_type, EXPR e, State s, ArrayList[] buffers, HashMap<RDDL.PVAR_NAME, Character> type_map, HashMap<RDDL.PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables, HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes, RandomDataGenerator random) throws Exception {

        if(WRITE_NPWL_TO_PWL_EARTH){
            output_writer.write("-----New Expression Reconstruction--------" +"\n");
            output_writer.write(">>>>>>>>Expression ::: " + e.toString() +"\n");
            output_writer.write(">>>>>>>>TargetPvar ::: " + target_pVar.toString() +"\n");
            output_writer.write(">>>>>>>>TargetLconsts ::: " + target_lconsts.toString()+"\n");
            output_writer.write(">>>>>>>>reward/not ::: " + (reward_type ? "true":"false") +"\n");
            output_writer.flush();
        }

        ArrayList<ArrayList<HashMap<RDDL.PVAR_NAME, HashMap<ArrayList<RDDL.LCONST>, Object>>>> buffer_state = buffers[0];
        ArrayList<ArrayList<ArrayList<RDDL.PVAR_INST_DEF>>> buffer_action = buffers[1];
        ArrayList<ArrayList<Double>> buffer_reward   = buffers[2];
        if(!reward_type){
            if(target_pVar._bPrimed){
                target_pVar = target_pVar._pvarUnprimed;
            }
        }

        ArrayList<Object> val = generateFeatureTargetEarth(e, s,target_pVar,target_lconsts,reward_type,buffer_state, buffer_action, buffer_reward, type_map, hm_variables, hmtypes, random);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        TreeMap<String, Pair<RDDL.PVAR_NAME, ArrayList<RDDL.LCONST>>> variables = (TreeMap<String, Pair<RDDL.PVAR_NAME, ArrayList<RDDL.LCONST>>>) val.get(0);
        HashMap<String, String> feat_type_map = (HashMap<String, String>) val.get(1);
        ArrayList<HashMap<String, String>> input_data = (ArrayList<HashMap<String, String>>) val.get(2);
        String target_type = (String) val.get(3);
        ArrayList<String> target_data = (ArrayList<String>) val.get(4);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if(WRITE_NPWL_TO_PWL_EARTH){
            output_writer.write(">>>>>>>>Target Type ::: " + target_type.toString()+"\n");
            output_writer.flush();
        }
        ArrayList<Object> r_strings = constructRData(variables, input_data, target_data, feat_type_map, target_type);
        //////////////////////////////////////////////////////////////////////////////////
        HashMap<String, String> input_Feat_R_array = (HashMap<String, String>) r_strings.get(0);
        String output_R_array = (String) r_strings.get(1);
        HashMap<String, TreeSet<String>> input_factors = (HashMap<String, TreeSet<String>>) r_strings.get(2);
        TreeSet<String> output_factors = (TreeSet<String>) r_strings.get(3);
        ////////////////////////////////////////////////////////////////////////////////
        EXPR final_expr = null;

        //Degenerate case.
        if (output_factors.size() == 1) {
            String str_val = null;
            for (String key : output_factors) {
                str_val = key;
            }
            if (reward_type) {
                final_expr = new RDDL.REAL_CONST_EXPR(Double.valueOf(str_val));

            } else {
                if (target_type.equals(BOOL)) {
                    final_expr = new RDDL.BOOL_CONST_EXPR(str_val == "1" ? true : false);
                } else if (target_type.equals(INTEGER)) {
                    final_expr = new RDDL.INT_CONST_EXPR(Integer.valueOf(str_val));
                } else if (target_type.equals(REAL)) {
                    final_expr = new RDDL.REAL_CONST_EXPR(Double.valueOf(str_val));
                } else if (target_type.equals(ENUM)) {
                    assert (hmtypes.get(hm_variables.get(target_pVar)._typeRange) instanceof RDDL.ENUM_TYPE_DEF);
                    int index = Integer.valueOf(str_val);
                    final_expr = (RDDL.ENUM_VAL) ((RDDL.ENUM_TYPE_DEF) hmtypes.get(hm_variables.get(target_pVar)._typeRange))._alPossibleValues.get(index);
                }
            }
        }
        if (final_expr != null) {
            if(WRITE_NPWL_TO_PWL_EARTH){
                output_writer.write(">>>>>>>>Earth Output Start" + "\n");
                output_writer.write(">>>>>>>>Earth Output End" + "\n");
                output_writer.write(">>>>>>>>Reconstructed EXPR  ::: " + final_expr.toString() +"\n");
                output_writer.write("----------------------------------------------------------------------\n");
                output_writer.flush();
            }
            return final_expr;
        }
        ////////////////////////////////////////////////////////// we construct the value of the target.

        //  -1
        //  + 0.1 * Feat____x1___d1__END__
        String [] earth_PWL = runEarth(variables,input_Feat_R_array,output_R_array,feat_type_map,target_type,input_factors,output_factors,hm_variables,hmtypes);
        if(WRITE_NPWL_TO_PWL_EARTH){
            output_writer.write(">>>>>>>>Earth Output Beginning " + "\n");
            for(String str_val : earth_PWL){
                output_writer.write(str_val.toString() + "\n");
            }
            output_writer.write(">>>>>>>>Earth Output End" + "\n");
            output_writer.flush();
        }

        //EXPR fin_expr =reconstruct_expr_new(earth_PWL,variables,feat_type_map,target_type,hm_variables,hmtypes);
//        ArrayList<String> temp_array = new ArrayList<>();
//        temp_array.add("  0.05025126\n  + 0.9497487 * Feat____dievalueseen___5__END__1\n");
//        temp_array.add("1.2\n + 0.54*Feat____dievalueseen___2__END__");
        ArrayList<EXPR> input_exprs = new ArrayList<>();
        for(String earth_str : earth_PWL){

            EXPR temp_expr_mapp = reconstruct_expr_new(earth_str, variables, feat_type_map, target_type, hm_variables, hmtypes);
            input_exprs.add(temp_expr_mapp);
        }


        final_expr = get_final_target(input_exprs,output_factors,reward_type,target_pVar,type_map,hm_variables,hmtypes);
        if(WRITE_NPWL_TO_PWL_EARTH){
            output_writer.write(">>>>>>>>Reconstructed EXPR  ::: " + final_expr.toString() +"\n");
            output_writer.write("----------------------------------------------------------------------\n");
        }

        return final_expr;


    }

    protected EXPR get_final_target(ArrayList<EXPR> input_exprs, TreeSet<String>output_factors, boolean reward_type, RDDL.PVAR_NAME target_pVar,
                          HashMap<RDDL.PVAR_NAME, Character> type_map, HashMap<RDDL.PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables, HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes) throws Exception{


        EXPR final_expr = null;
        if(reward_type){

            final_expr = input_exprs.get(0);
            for(int i=1 ; i<input_exprs.size() ; i++){
                EXPR temp_expr = input_exprs.get(i);
                final_expr = new RDDL.OPER_EXPR(final_expr,temp_expr,"+");
            }

        }else{

            if(type_map.get(target_pVar).equals(GRB.BINARY)){
                //This is for reconstructing Binary CPT's
                assert input_exprs.size()==1;
                EXPR temp_expr = input_exprs.get(0);
                RDDL.BOOL_EXPR t1 =new RDDL.COMP_EXPR(temp_expr,new RDDL.REAL_CONST_EXPR(0.5),">");
//                EXPR t2 = new RDDL.IF_EXPR(t1,new RDDL.BOOL_CONST_EXPR(true), new RDDL.BOOL_CONST_EXPR(false));
                final_expr = t1;
            }else if((hmtypes.get(hm_variables.get(target_pVar)._typeRange) instanceof RDDL.ENUM_TYPE_DEF)){
                //This is for Reconstructing Enum's CPT's
                assert output_factors.size()==input_exprs.size();
                int count =0 ;
                RDDL.FUN_EXPR max_exprs = new RDDL.FUN_EXPR("max",input_exprs);
                ArrayList<RDDL.COMP_EXPR> comp_exprs = new ArrayList<>();
                int ord_index =0;
                ArrayList<EXPR> enum_expr_list = new ArrayList<>();
                for(String ord : output_factors){
                    int enum_index = Integer.valueOf(ord);
                    RDDL.ENUM_VAL enum_expr =(RDDL.ENUM_VAL) ((RDDL.ENUM_TYPE_DEF) hmtypes.get(hm_variables.get(target_pVar)._typeRange))._alPossibleValues.get(enum_index);
                    RDDL.COMP_EXPR t1 = new RDDL.COMP_EXPR(max_exprs,input_exprs.get(ord_index),"==");
                    comp_exprs.add(t1);
                    enum_expr_list.add(enum_expr);
                    ord_index+=1;

                }
                RDDL.ENUM_VAL def_enum_expr = (RDDL.ENUM_VAL) ((RDDL.PVARIABLE_STATE_DEF) hm_variables.get(target_pVar))._oDefValue;
                EXPR if_else_expr = recursiveIFELSE(comp_exprs,enum_expr_list,def_enum_expr);
                final_expr = if_else_expr;
            }
        }

        return final_expr;





    }

    protected EXPR recursiveIFELSE(ArrayList<RDDL.COMP_EXPR>comp_exps, ArrayList<EXPR>input_exprs, RDDL.ENUM_VAL def_enum_val){

        if(comp_exps.size()==1){
            return new RDDL.IF_EXPR(comp_exps.get(0),input_exprs.get(0),def_enum_val);
        }

        ArrayList<RDDL.COMP_EXPR>temp_comp = new ArrayList<RDDL.COMP_EXPR>(comp_exps.subList(1,comp_exps.size()));
        ArrayList<EXPR> temp_exprs = new ArrayList<EXPR>(input_exprs.subList(1,input_exprs.size()));

        return new RDDL.IF_EXPR(comp_exps.get(0),input_exprs.get(0), recursiveIFELSE(temp_comp,temp_exprs,def_enum_val));




    }


    protected HashMap<String, String> getInputVector(TreeMap<String, Pair<RDDL.PVAR_NAME, ArrayList<RDDL.LCONST>>> variables, HashMap<RDDL.PVAR_NAME, ArrayList<RDDL.LCONST>> lvar_lcont_map,
                                                     HashMap<RDDL.PVAR_NAME, HashMap<ArrayList<RDDL.LCONST>, Object>> state_value,
                                                     ArrayList<RDDL.PVAR_INST_DEF> action_value, HashMap<RDDL.PVAR_NAME,
            RDDL.PVARIABLE_DEF> hm_variables, HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes) throws Exception {

        //I need to debug and work.
        //This loop is for ordering the inputVariables via variables.

        HashMap<String, String> feat_values = new HashMap<>();
        for (String key : variables.keySet()) {
            ArrayList<RDDL.LCONST> cur_lconsts = variables.get(key)._o2;
            RDDL.PVAR_NAME cur_pvar = variables.get(key)._o1;
            Object feat_val =_getpVarValue(cur_pvar,cur_lconsts,state_value,action_value,hm_variables,hmtypes);
            String str_feat_val = rddlObj2String(feat_val,hm_variables,hmtypes);
            feat_values.put(key,str_feat_val);

        }


        return feat_values;


    }

    protected Object _getDefaultpVarValue(RDDL.PVAR_NAME pVarName,HashMap<RDDL.PVAR_NAME,RDDL.PVARIABLE_DEF> hm_variables) throws Exception {
        Object out_val = null;

        if (hm_variables.get(pVarName) instanceof RDDL.PVARIABLE_STATE_DEF) {
            out_val = ((RDDL.PVARIABLE_STATE_DEF) hm_variables.get(pVarName))._oDefValue;
        } else if (hm_variables.get(pVarName) instanceof RDDL.PVARIABLE_ACTION_DEF) {
            out_val = ((RDDL.PVARIABLE_ACTION_DEF) hm_variables.get(pVarName))._oDefValue;
        } else {
            throw new Exception("This is no Default value for the type you are looking");
        }
        return out_val;

    }


    protected Object _getpVarValue(RDDL.PVAR_NAME pVarName, ArrayList<RDDL.LCONST> lconsts, HashMap<RDDL.PVAR_NAME,HashMap<ArrayList<RDDL.LCONST>,Object>> state_value,
                                        ArrayList<RDDL.PVAR_INST_DEF> action_value, HashMap<RDDL.PVAR_NAME,RDDL.PVARIABLE_DEF> hm_variables, HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes ) throws Exception {
        Object out_val = null;
        //This is for State Value.
        if (state_value.containsKey(pVarName)) {
            HashMap<ArrayList<RDDL.LCONST>, Object> cur_lconst_object = state_value.get(pVarName);
            if(cur_lconst_object.containsKey(lconsts)){
                out_val = cur_lconst_object.get(lconsts);
            }else if(cur_lconst_object.size()==0){
                if(cur_lconst_object.get(lconsts)==null){
                    out_val = _getDefaultpVarValue(pVarName,hm_variables);
                }else {
                    out_val = cur_lconst_object.get(lconsts);

                }

            }else{//This is getting Default Value Case.
                out_val = _getDefaultpVarValue(pVarName,hm_variables);
            }

        }else {
            //It can be in Action Val...
            boolean found = false;
            for (RDDL.PVAR_INST_DEF array_val : action_value) {
                if (pVarName.equals(array_val._sPredName) && lconsts.equals(array_val._alTerms)) {
                    out_val = array_val._oValue;
                    found = true;
                }
            }
            if(!found){
                out_val = _getDefaultpVarValue(pVarName,hm_variables);
            }

        }


        return out_val;
    }


    //Helper function,
    protected static String _getRFeat(Pair<RDDL.PVAR_NAME, ArrayList<RDDL.LCONST>> key_val) {
        //LCONST  are serpated by ___
        //Feature value is separted by ____
        String val = key_val._o1._sPVarName;
        for (int i = 0; i < key_val._o2.size(); i++) {
            String temp_str = key_val._o2.get(i).toString();
            //Assuming $t1,$u2, etc.
            temp_str = temp_str.substring(1, temp_str.length());


            val = val + "___" + temp_str;

        }
        val = val.replaceAll("[-+.^:,]", "");
        val = "Feat" + "____" + val;
        val = val + "__END__";
        return val;


    }


    protected String _getFeatType(RDDL.PVAR_NAME pName, HashMap<RDDL.PVAR_NAME, Character> type_map, HashMap<RDDL.PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables, HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes) throws Exception {

        if ((hmtypes.get(hm_variables.get(pName)._typeRange) instanceof RDDL.ENUM_TYPE_DEF)) {
            return ENUM;
        } else if (type_map.get(pName).equals(GRB.BINARY)) {
            return BOOL;
        } else if (type_map.get(pName).equals(GRB.INTEGER)) {
            return INTEGER;
        } else if (type_map.get(pName).equals(GRB.CONTINUOUS)) {
            return REAL;
        } else {
            try {
                throw new Exception("THis case is not handled in R.");
            } catch (Exception e) {
                e.printStackTrace();
                throw e;
            }
        }


    }



    protected String rddlObj2String(Object target_val, HashMap<RDDL.PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables, HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes) throws Exception {
        if (target_val instanceof Boolean) {
            return (Boolean) target_val ? "1" : "0";
        } else if (target_val instanceof RDDL.ENUM_VAL) {
            return String.valueOf(((RDDL.ENUM_VAL) target_val).enum_to_int(hmtypes, hm_variables));
        } else if (target_val instanceof Double) {
            return String.valueOf((Double) target_val);
        } else if (target_val instanceof Integer) {
            return String.valueOf((Integer) target_val);
        } else {
            try {
                throw new Exception("THis case is not handled in R.");
            } catch (Exception e) {
                e.printStackTrace();
                throw e;
            }
        }

    }




    protected String _getRfactor(TreeSet<String> factors) {
        String temp_str = "levels=c(";
        for (String key : factors) {
            temp_str = temp_str + key + ",";
        }
        temp_str = temp_str.substring(0, temp_str.length() - 1) + ")";
        return temp_str;
    }

    protected ArrayList<Object> constructRData(TreeMap<String, Pair<RDDL.PVAR_NAME, ArrayList<RDDL.LCONST>>> variables, ArrayList<HashMap<String, String>> input_data, ArrayList<String> target_data, HashMap<String, String> input_type_map, String target_type) {
        //This has to be key of Variables, and
        HashMap<String, String> input_Feat_R_array = new HashMap<>();
        String output_R_array = new String();
        output_R_array = "c(";
        ArrayList<Object> out_val = new ArrayList<>();
        HashMap<String, TreeSet<String>> input_factors = new HashMap<>();
        TreeSet<String> output_factors = new TreeSet<>();
        HashMap<String,ArrayList<String>> input_array = new HashMap();
        ArrayList<String> output_array = new ArrayList<>();
        for(String key : variables.keySet()){
            input_array.put(key,new ArrayList<String>());

        }

        //construction of Input and target vector in R format.
        try {
            assert input_data.size() == target_data.size();
        } catch (AssertionError e) {
            throw e;
        }
        int data_length = input_data.size();

        for (int k = 0; k < data_length; k++) {
            //This is output R data
            String out_temp_str = target_data.get(k);
            if (k == data_length - 1) {
                output_R_array = output_R_array + out_temp_str + " )";
                output_factors.add(out_temp_str);
            } else {
                output_R_array = output_R_array + out_temp_str + ",";
                output_factors.add(out_temp_str);
            }


            //This is for input_features... This will make sure of the order. TreeMap (Important).
            //focus on one input_data.
            HashMap<String, String> temp_input_data = input_data.get(k);
            for (final String feat_name : variables.keySet()) {
                if (temp_input_data.containsKey(feat_name)) {
                    //When Feature is avialble in the data
                    final String input_temp_str1 = temp_input_data.get(feat_name);
                    input_array.get(feat_name).add(input_temp_str1);
                    if (!input_factors.containsKey(feat_name)) {
                        TreeSet<String> temp_hashSet = new TreeSet<String>();
                        temp_hashSet.add(input_temp_str1);
                        input_factors.put(feat_name, temp_hashSet);

                    } else {
                        input_factors.get(feat_name).add(input_temp_str1);
                    }
                    }
                }
            }



        for(String key : input_array.keySet()){
            input_Feat_R_array.put(key,"");
        }

        for(String key : input_array.keySet()){
            String temp_str = "";

            for(int i=0 ;i<input_array.get(key).size();i++){
                if(i==0){
                    temp_str =  "c("+input_array.get(key).get(i);
                }else{
                    temp_str = temp_str + ","+ input_array.get(key).get(i);
                }

            }
            input_Feat_R_array.put(key,temp_str + ")");
        }

        out_val.add(input_Feat_R_array);
        out_val.add(output_R_array);
        out_val.add(input_factors);
        out_val.add(output_factors);
        return out_val;

    }

    public ArrayList<Object> generateFeatureTargetEarth(EXPR e, State s, RDDL.PVAR_NAME target_Pvar, ArrayList<RDDL.LCONST>target_lconsts,boolean reward_type, ArrayList<ArrayList<HashMap<RDDL.PVAR_NAME, HashMap<ArrayList<RDDL.LCONST>, Object>>>> buffer_state, ArrayList<ArrayList<ArrayList<RDDL.PVAR_INST_DEF>>> buffer_action, ArrayList<ArrayList<Double>> buffer_reward,
                                                        HashMap<RDDL.PVAR_NAME, Character> type_map, HashMap<RDDL.PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables, HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes, RandomDataGenerator random) throws Exception {

        TreeMap<String, Pair<RDDL.PVAR_NAME, ArrayList<RDDL.LCONST>>> variables = new TreeMap<>();
        HashMap<RDDL.PVAR_NAME, ArrayList<RDDL.LCONST>> var_lconsts = new HashMap<>();
        ArrayList<HashMap<String, String>> input_data = new ArrayList<>();
        ArrayList<String> target_data = new ArrayList<>();
        HashMap<String, String> feat_type_map = new HashMap<>();
        String target_type = null;

        HashSet<Pair> Gfluents = new HashSet<>();
        HashMap<RDDL.LVAR, RDDL.LCONST> subs1 = new HashMap<>();
        try{
            e.collectGFluents(subs1, s, Gfluents);
            for (Pair key : Gfluents) {
                // "<rlevel, [$t1]>" this type feature name doesn't work in R.  converting to rlevel_t1_t2....
                String str_key = _getRFeat(key);
                String feat_type = _getFeatType((RDDL.PVAR_NAME) key._o1,type_map, hm_variables, hmtypes);
                if (!feat_type_map.containsKey(str_key))
                    feat_type_map.put(str_key, feat_type);
                if (!variables.containsKey(str_key))
                    variables.put(str_key, key);
            }
        }catch (Exception exc){
            exc.printStackTrace();
        }



        for (int i = 0; i < buffer_state.size(); i++) {
            assert buffer_state.get(i).size()>=2;

            //Current State
            HashMap<RDDL.PVAR_NAME,HashMap<ArrayList<RDDL.LCONST>,Object>>  data_state = buffer_state.get(i).get(0);
            //Next State
            HashMap<RDDL.PVAR_NAME,HashMap<ArrayList<RDDL.LCONST>,Object>>  next_data_state = buffer_state.get(i).get(1);
            //Current Action
            ArrayList<RDDL.PVAR_INST_DEF> data_action = buffer_action.get(i).get(0);
            //Immediate Reward for taking action in state
            Double data_reward = buffer_reward.get(i).get(0);
            HashMap<String, String> si1 = null;
            try {
                //<feat,val>
                si1 = getInputVector(variables, var_lconsts, data_state, data_action, hm_variables, hmtypes);
                Object target_val = null;
                if(reward_type){
                    target_val = data_reward;
                    target_type = REAL;
                }else{
                    //need to change the code for getting target value. ..
                    target_val = _getpVarValue(target_Pvar,target_lconsts,next_data_state,data_action,hm_variables,hmtypes);
                    target_type = _getFeatType(target_Pvar,type_map,hm_variables,hmtypes);
                }
                String target_val_str = rddlObj2String(target_val, hm_variables, hmtypes);
                input_data.add(si1);
                target_data.add(target_val_str);

            } catch (Exception e1) {
                e1.printStackTrace();

            }

        }
        ArrayList final_output_list = new ArrayList<Object>();
        final_output_list.add(variables);
        final_output_list.add(feat_type_map);
        final_output_list.add(input_data);
        final_output_list.add(target_type);
        final_output_list.add(target_data);

        return final_output_list;

    }


    protected String[] runEarth(TreeMap<String, Pair<RDDL.PVAR_NAME, ArrayList<RDDL.LCONST>>> variables, HashMap<String, String> input_features, String output,
                              HashMap<String, String> input_type_map, String target_type, HashMap<String, TreeSet<String>> input_factors, TreeSet<String> target_factors,
                              HashMap<RDDL.PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables, HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes) throws Exception {

        //Starting Rengine.

        Rengine engine = Rengine.getMainEngine();
        if (engine == null)
            engine = new Rengine(new String[]{"--vanilla"}, false, null);

        //Rengine engine  = new Rengine(new String[] {"--no-save"},false,null);
        if (PRINT_TO_R_FILE) {
            writer = new BufferedWriter(new OutputStreamWriter(
                    new FileOutputStream(R_FILE_NAME), "utf-8"));
        }
        engine.eval("library(earth)");
        String feature_format = new String();
        Integer check = 0;

        for (Map.Entry<String, String> entry1 : input_features.entrySet()) {
            String key = entry1.getKey();
            String value = entry1.getValue();
            RDDL.PVAR_NAME temp_pvar = (variables.get(key)._o1);

            if (input_type_map.get(key).equals(ENUM)) {
                ArrayList<RDDL.LCONST> enum_array = ((RDDL.ENUM_TYPE_DEF) hmtypes.get(hm_variables.get(temp_pvar)._typeRange))._alPossibleValues;
                for (int k = 0; k < enum_array.size(); k++) {
                    int e_int = ((RDDL.ENUM_VAL) enum_array.get(k)).enum_to_int(hmtypes, hm_variables);
                    input_factors.get(key).add(String.valueOf(e_int));
                }
                String r_factors = _getRfactor(input_factors.get(key));
                String s1 = key + "<-" + "factor(" + value + "," + r_factors + ")";
                Object temp = engine.eval(s1);
                if (PRINT_TO_R_FILE) {
                    writer.write(s1 + "\n");
                    writer.flush();
                }


            } else if (input_type_map.get(key).equals(BOOL)) {
                for (int k = 0; k < 2; k++) {
                    input_factors.get(key).add(String.valueOf(k));
                }
                String r_factors = _getRfactor(input_factors.get(key));
                String s1 = key + "<-" + "factor(" + value + "," + r_factors + ")";
                Object temp = engine.eval(s1);
                if (PRINT_TO_R_FILE) {
                    writer.write(s1 + "\n");
                    writer.flush();
                }

            } else if (input_type_map.get(key).equals(INTEGER)) {
                String s1 = key + "<-" + value;
                if (PRINT_TO_R_FILE) {
                    writer.write(s1 + "\n");
                    writer.flush();
                }
                Object temp = engine.eval(s1);


            } else if (input_type_map.get(key).equals(REAL)) {
                String s1 = key + "<-" + value ;
                if (PRINT_TO_R_FILE) {
                    writer.write(s1 + "\n");
                    writer.flush();
                }
                Object temp = engine.eval(s1);

            } else {
                try {
                    throw new Exception("THis case is not handled in R.");
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            if (check == 0) {
                feature_format = entry1.getKey();
                check = 1;
            } else {
                feature_format = feature_format.concat(" + " + entry1.getKey());
            }
        }

        String temp_s1 = "target <-" + output;
        if (PRINT_TO_R_FILE) {
            writer.write(temp_s1 + "\n");
            writer.flush();
        }
        engine.eval(temp_s1);
        if (target_type.equals(ENUM)) {
            String r_factors = _getRfactor(target_factors);
            String s1 = "target <-" + "factor( target " + "," + r_factors + ")";
            if (PRINT_TO_R_FILE) {
                writer.write(s1 + "\n");
                writer.flush();
            }
            Object temp = engine.eval(s1);


        } else if (target_type.equals(BOOL)) {
            String r_factors = _getRfactor(target_factors);
            final String s1 = "target <-" + "factor( target " + "," + r_factors + ")";
            if (PRINT_TO_R_FILE) {
                writer.write(s1 + "\n");
                writer.flush();
            }
            Object temp = engine.eval(s1);


        } else if (target_type.equals(INTEGER)) {


        } else if (target_type.equals(REAL)) {

        } else {
            try {
                throw new Exception("THis case is not handled in R.");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        String s1 = "model<-earth( target ~ " + feature_format + ",nprune=3)";
        if (PRINT_TO_R_FILE) {
            writer.write(s1 + "\n");
            writer.flush();
        }

        engine.eval(s1);

        s1 = "print(summary(model))";
        if (PRINT_TO_R_FILE) {
            writer.write(s1 + "\n");
            writer.flush();
        }
        engine.eval(s1);

        s1 = "format(model)";
        if (PRINT_TO_R_FILE) {
            writer.write(s1 + "\n");
            writer.flush();
        }

        Object earth_output1 = engine.eval(s1);
        String[] earth_output =null;
        if(earth_output1!=null){
            earth_output =(String[]) ((REXP)earth_output1).getContent();

        }

        //String earth_output = "kdfjkdjf";
        //Need to be careful or order of input features.
        return earth_output;

    }


    public EXPR reconstruct_expr_new(String earthOutput, TreeMap<String, Pair<RDDL.PVAR_NAME, ArrayList<RDDL.LCONST>>> variables,
                                     HashMap<String, String> input_type_map, String target_type,
                                     HashMap<RDDL.PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables,
                                     HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes) throws Exception {
        //"  0.05025126\n  + 0.9497487 * Feat____dievalueseen___5__END__1\n"
        //"  0.3769495\n  -  0.2593277 * survived\n  + 0.01338067 * h(26-age)\n
        String[] list_output = earthOutput.split("\n");
        TreeSet<String> enum_ordering = new TreeSet<>();
        ArrayList<EXPR> terms_exprs = new ArrayList<>();
        for (int i = 0; i < list_output.length; i++) {
            String temp_str = list_output[i].trim();
            if (!temp_str.contains("*")) {
                double bias = Double.parseDouble(temp_str);
                terms_exprs.add(new RDDL.REAL_CONST_EXPR(bias));
            } else if (temp_str.contains("*")) {
                temp_str = temp_str.replaceAll("\\s", "");
                temp_str = temp_str.replaceAll("\\+", "");
                String[] term_val = temp_str.split("\\*");
                NumberFormat format = NumberFormat.getInstance();
                Double coeffic = format.parse(term_val[0]).doubleValue();
                EXPR coeffic_expr = new RDDL.REAL_CONST_EXPR(coeffic);
                String hinge_str = term_val[1].trim();


                //This is a continous value.
                if (hinge_str.contains("h(")) {
                    hinge_str = hinge_str.replace("h(", "");
                    hinge_str = hinge_str.replace(")", "");
                    String[] hinge_values = hinge_str.split("-");
                    if (variables.containsKey(hinge_values[0])) {
                        //h(t1-19)
                        double con1 = Double.parseDouble(hinge_values[1]);
                        Pair<RDDL.PVAR_NAME, ArrayList<RDDL.LCONST>> mapping = variables.get(hinge_values[0]);
                        EXPR t1 = new RDDL.PVAR_EXPR(mapping._o1._sPVarName, mapping._o2);
                        EXPR t2 = new RDDL.REAL_CONST_EXPR(con1);
                        RDDL.OPER_EXPR t3 = new RDDL.OPER_EXPR(t1, t2, "-");
                        RDDL.OPER_EXPR t4 = new RDDL.OPER_EXPR(new RDDL.REAL_CONST_EXPR(0.0), t3, "max");
                        RDDL.OPER_EXPR t5 = new RDDL.OPER_EXPR(coeffic_expr, t4, "*");
                        terms_exprs.add(t5);
                    } else if(variables.containsKey(hinge_values[1])){
                        double con1 = Double.parseDouble(hinge_values[0]);
                        Pair<RDDL.PVAR_NAME, ArrayList<RDDL.LCONST>> mapping = variables.get(hinge_values[1]);
                        EXPR t1 = new RDDL.PVAR_EXPR(mapping._o1._sPVarName, mapping._o2);
                        EXPR t2 = new RDDL.REAL_CONST_EXPR(con1);
                        RDDL.OPER_EXPR t3 = new RDDL.OPER_EXPR(t2, t1, "-");
                        RDDL.OPER_EXPR t4 = new RDDL.OPER_EXPR(new RDDL.REAL_CONST_EXPR(0.0), t3, "max");
                        RDDL.OPER_EXPR t5 = new RDDL.OPER_EXPR(coeffic_expr, t4, "*");
                        terms_exprs.add(t5);

                    }


                } else{
                    hinge_str = hinge_str.replace("h(", "");
                    hinge_str = hinge_str.replace(")", "");

                    String[] h_v3 = hinge_str.split("__END__");
                    String  another_htype = (h_v3[0] + "__END__");

                    if (variables.containsKey(another_htype)) {
                        String[] hinge_values1 = hinge_str.split("__END__");
                        String key_feat = (hinge_values1[0] + "__END__");
                        if (variables.containsKey(key_feat)) {
                            Pair<RDDL.PVAR_NAME, ArrayList<RDDL.LCONST>> mapping = variables.get(key_feat);
                            if (input_type_map.get(key_feat).equals("bool")) {
                                EXPR t1 = new RDDL.PVAR_EXPR(mapping._o1._sPVarName, mapping._o2);
                                EXPR t2 = new RDDL.OPER_EXPR(coeffic_expr, t1, "*");
                                terms_exprs.add(t2);

                            } else if (input_type_map.get(key_feat).equals("enum")) {
                                enum_ordering.add(hinge_values1[1]);
                                EXPR t1 = new RDDL.PVAR_EXPR(mapping._o1._sPVarName, mapping._o2);
                                int enum_index = Integer.valueOf(hinge_values1[1]);
                                RDDL.ENUM_VAL enum_expr = (RDDL.ENUM_VAL) ((RDDL.ENUM_TYPE_DEF) hmtypes.get(hm_variables.get(mapping._o1._sPVarName)._typeRange))._alPossibleValues.get(enum_index);
                                EXPR t2 = new RDDL.COMP_EXPR(t1, enum_expr, "==");
                                terms_exprs.add(t2);


                            } else if(input_type_map.get(key_feat).equals("real")){
                                EXPR t1 = new RDDL.PVAR_EXPR(mapping._o1._sPVarName, mapping._o2);
                                EXPR t2 = new RDDL.OPER_EXPR(coeffic_expr, t1, "*");
                                terms_exprs.add(t2);


                            }else if(input_type_map.get(key_feat).equals("int")){
                                EXPR t1 = new RDDL.PVAR_EXPR(mapping._o1._sPVarName, mapping._o2);
                                EXPR t2 = new RDDL.OPER_EXPR(coeffic_expr, t1, "*");
                                terms_exprs.add(t2);


                            }



                        }

                    }



                }


                }
            }

            EXPR final_expr = terms_exprs.get(0);
            for(int i=1 ; i<terms_exprs.size() ; i++){
                EXPR temp_expr = terms_exprs.get(i);
                final_expr = new RDDL.OPER_EXPR(final_expr,temp_expr,"+");


            }

        return final_expr;

        }







    protected void testEarthForData(){





    }


//
//
//    public RDDL.EXPR reconstruct_expr(String earthOutput, TreeMap<String,Pair<RDDL.PVAR_NAME,ArrayList<RDDL.LCONST>>> variables,
//                                      HashMap<String,String> input_type_map, String target_type,
//                                      HashMap<RDDL.PVAR_NAME, RDDL.PVARIABLE_DEF> hm_variables,
//                                      HashMap<RDDL.TYPE_NAME, RDDL.TYPE_DEF> hmtypes) throws Exception {
//
//        //Sample Earth Output
//        //Earth Output
//        // 1 +
//        //   1.3 * bf1
//        //
//        // bf1 : h(53.2847-rlevel)
//        //Where h is the hinge Function.
//
//        //"0.04020101\n  + 0.959799 * bf1\n\n
//        // bf1  Feat____dievalueseen___5__END__1\n"
//        HashMap<String,String> binary_variables =null;
//        HashMap<String,String> enum_variables = null;
//        HashMap<String,String> continu_variables = null;
//
//
//        String[] list_output = earthOutput.split("\n");
//        HashMap<String,Double> coefficient_mapping = new HashMap<>();
//        HashMap<String, RDDL.EXPR> hinge_function  = new HashMap<>();
//        Double bias = 0.0;
//        //Parsing things with equations.
//        for(int i=0;i<list_output.length;i++){
//            String temp_str = list_output[i].trim();
//            if(temp_str.equals("")) {
//                continue;
//            }else if( temp_str.contains("e-")  && (!temp_str.contains("*"))){
//                bias =Double.parseDouble(temp_str);
//                continue;
//
//            } else if(temp_str.contains("*")){
//
//                temp_str = temp_str.replaceAll("\\s","");
//                temp_str = temp_str.replaceAll("\\+","");
//                String [] term_val = temp_str.split("\\*");
//                NumberFormat format = NumberFormat.getInstance();
//                Double coeffic = format.parse(term_val[0]).doubleValue();
//                coefficient_mapping.put(term_val[1],coeffic);
//                continue;
//            }else if(temp_str.contains("bf") && temp_str.contains("h(")){
//                String[] term_val =temp_str.split("\\s");
//                String key_val = term_val[0];
//                String hinge_str = term_val[2];
//                hinge_str = hinge_str.replace("h(","");
//                hinge_str = hinge_str.replace(")","");
//                String [] hinge_values = hinge_str.split("-");
//                Double real_val = 0.0;
//                if(variables.containsKey(hinge_values[0])){
//                    real_val = Double.parseDouble(hinge_values[1]);
//                    hm_variables.get(hinge_values[0]);
//                    RDDL.PVAR_EXPR temp_pvar_expr        = new RDDL.PVAR_EXPR(variables.get(hinge_values[0])._o1._sPVarName,variables.get(hinge_values[0])._o2);
//
//                    RDDL.REAL_CONST_EXPR temp_const_expr = new RDDL.REAL_CONST_EXPR(real_val);
//                    RDDL.OPER_EXPR temp_oper_expr        = new RDDL.OPER_EXPR(temp_pvar_expr,temp_const_expr,"-");
//                    RDDL.OPER_EXPR max_oper_expr         = new RDDL.OPER_EXPR(new RDDL.REAL_CONST_EXPR(0.0), temp_oper_expr,"max");
//                    hinge_function.put(key_val,max_oper_expr);
//
//                }
//                if(variables.containsKey(hinge_values[1])){
//                    real_val = Double.parseDouble(hinge_values[0]);
//                    RDDL.PVAR_EXPR temp_pvar_expr        = new RDDL.PVAR_EXPR(variables.get(hinge_values[0])._o1._sPVarName,variables.get(hinge_values[1])._o2);
//                    RDDL.REAL_CONST_EXPR temp_const_expr = new RDDL.REAL_CONST_EXPR(real_val);
//
//                    RDDL.OPER_EXPR temp_oper_expr        = new RDDL.OPER_EXPR(temp_const_expr,temp_pvar_expr,"-");
//                    RDDL.OPER_EXPR max_oper_expr         = new RDDL.OPER_EXPR(new RDDL.REAL_CONST_EXPR(0.0), temp_oper_expr,"max");
//
//                    hinge_function.put(key_val,max_oper_expr);
//
//
//                }
//                continue;
//            }else if(temp_str.contains("bf")){
//                //These are factored types......
//                String[] term_val =temp_str.split("\\s");
//                String key_val = term_val[0];
//                String hinge_str = term_val[2];
//
//                String [] hinge_values = hinge_str.split("__END__");
//                //Feat____dievalueseen___5__
//                String key_feat = (hinge_values[0]+"__END__");
//
//                String type_str = input_type_map.get(key_feat);
//                if(type_str.equals("bool")){
//
//
//                }else if(type_str.equals("enum")){
//
//
//                }
//
//
//
//
//                continue;
//            } else if(!(temp_str.contains("-") || temp_str.contains("+"))){
//
//                bias =Double.parseDouble(temp_str);
//                continue;
//            }
//
//        }
//
//
//
//
//
//
//        RDDL.EXPR final_expr = new RDDL.REAL_CONST_EXPR(bias);
//
//        Integer temp_count = 0;
//        for(String key: coefficient_mapping.keySet()){
//            Double real_value = coefficient_mapping.get(key);
//            RDDL.REAL_CONST_EXPR temp_real_expr= new RDDL.REAL_CONST_EXPR(real_value);
//            RDDL.EXPR temp_oper_expr  = new RDDL.OPER_EXPR(temp_real_expr,hinge_function.get(key),"*");
//            RDDL.EXPR temp_final_expr = final_expr;
//            final_expr = new RDDL.OPER_EXPR(temp_final_expr,temp_oper_expr,"+");
//        }
//        return final_expr;
//
//
//
//    }
//
//
//
//







}

