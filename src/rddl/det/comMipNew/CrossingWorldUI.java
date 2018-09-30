package rddl.det.comMipNew;

import rddl.RDDL;
import util.Pair;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class CrossingWorldUI {
    protected int x_count = 3;
    protected int y_count = 3;
    protected ArrayList<String> x_list = new ArrayList<>();
    protected ArrayList<String> y_list = new ArrayList<>();
    protected HashMap<String,HashMap<RDDL.LCONST,Object>> plan = null;
    protected String horizontal = "-------";
    protected String veritcal   = "|     |";
    protected String goal_text  = "|  G  |";
    protected String agent_text    = "|  A  |";
    protected String obstacle_text = "|  O  |";
    protected String OBSTACLE_CHAR = "O";
    protected String AGENT_CHAR = "A";
    protected String GOAL_CHAR = "G";
    protected Double FILTER_CRITERIA = 0.95;
    protected List<Pair<Integer,Integer>> GOAL_LIST  = Arrays.asList(new Pair<>(3,3));



    public CrossingWorldUI(){
        //this.plan = plan;
    }



    protected String multiply(String val, int x,boolean new_line){
        String out_val  = "";
        for(int i =0 ; i<x;i++){
            out_val += val;
        }
        if(new_line){
            out_val +="\n";
        }

        return out_val;

    }




    protected String generateGrid(HashMap<String,ArrayList<Pair<Integer,Integer>>> all_objects){
        ArrayList<Pair<Integer,Integer>> obstacles = all_objects.get("O");
        ArrayList<Pair<Integer,Integer>> agents = all_objects.get("A");
        ArrayList<Pair<Integer,Integer>> goals  = all_objects.get("G");
        String final_str ="";
        for(int j=y_count ; j>0;j--){
            final_str += multiply(horizontal,x_count,true);

            for(int k=0 ;k<3;k++){
                if(k==0){
                    //Want to write Obstacles
                    String temp_str = "";
                    ArrayList<Integer> get_list = new ArrayList<>();
                    for(int arr_i=0 ; arr_i<obstacles.size();arr_i++){
                        if(obstacles.get(arr_i)._o2.equals(j)){
                            get_list.add(obstacles.get(arr_i)._o1);
                        }
                    }
                    for(int l=1 ; l<=y_count;l++){
                        if(get_list.contains(l)){
                            temp_str+=multiply(obstacle_text,1,false);
                        }else{
                            temp_str+=multiply(veritcal,1,false);
                        }
                    }
                    temp_str+="\n";
                    final_str+=temp_str;
                }else if(k==1){
                    //Want to write Agents
                    String temp_str = "";
                    ArrayList<Integer> get_list = new ArrayList<>();
                    for(int arr_i=0 ; arr_i<agents.size();arr_i++){
                        if(agents.get(arr_i)._o2.equals(j)){
                            get_list.add(agents.get(arr_i)._o1);
                        }
                    }

                    for(int l=1 ; l<=y_count;l++){
                        if(get_list.contains(l)){
                            temp_str+=multiply(agent_text,1,false);
                        }else{
                            temp_str+=multiply(veritcal,1,false);
                        }
                    }
                    temp_str+="\n";
                    final_str+=temp_str;
                }else if(k==2){
                 //Want to Goal Objects.
                    String temp_str = "";
                    ArrayList<Integer> get_list = new ArrayList<>();
                    for(int arr_i=0 ; arr_i<goals.size();arr_i++){
                        if(goals.get(arr_i)._o2.equals(j)){
                            get_list.add(goals.get(arr_i)._o1);
                        }
                    }
                    for(int l=1 ; l<=y_count;l++){
                        if(get_list.contains(l)){
                            temp_str+=multiply(goal_text,1,false);
                        }else{
                            temp_str+=multiply(veritcal,1,false);
                        }
                    }
                    temp_str+="\n";
                    final_str+=temp_str;
                }

            }
        }
        final_str+=multiply(horizontal,x_count,true);
        return final_str;
//        multiply(veritcal,3,true);
//        multiply(veritcal,2,false);
//        multiply(goal_text,1,true);
//        multiply(veritcal,3,true);
//        multiply(horizontal,3,true);
//        multiply(veritcal,3,true);
//        multiply(veritcal,3,true);
//        multiply(veritcal,3,true);
//        multiply(horizontal,3,true);
//        multiply(veritcal,3,true);
//        multiply(veritcal,3,true);
//        multiply(veritcal,3,true);
//        multiply(horizontal,3,true);

    }

    protected String testGenerateGrid(){
        HashMap<String,ArrayList<Pair<Integer,Integer>>> all_objects = new HashMap<>();
        ArrayList<Pair<Integer,Integer>> obstacle_positions = new ArrayList<>();
        obstacle_positions.add(new Pair<>(2,3));
        //obstacle_positions.add(new Pair<>(1,2));
        //obstacle_positions.add(new Pair<>(3,1));

        ArrayList<Pair<Integer,Integer>> agent_positions = new ArrayList<>();
        //agent_positions.add(new Pair<>(2,3));
        //gent_positions.add(new Pair<>(1,2));
        agent_positions.add(new Pair<>(3,1));


        ArrayList<Pair<Integer,Integer>> goal_positions = new ArrayList<>();
        goal_positions.add(new Pair<>(3,3));
        //goal_positions.add(new Pair<>(1,2));
        //goal_positions.add(new Pair<>(3,1));
        all_objects.put("O",obstacle_positions);
        all_objects.put("A",agent_positions);
        all_objects.put("G",goal_positions);
        String output = generateGrid(all_objects);
        return output;

    }

    protected ArrayList<HashMap<String,ArrayList<Pair<Integer,Integer>>>> convertPlanToObjects(HashMap<String,HashMap<RDDL.LCONST,Object>> plan, int length ){
        HashMap<RDDL.LCONST,Object> state_object = plan.get("_State");
        HashMap<RDDL.LCONST,Object> action_object = plan.get("_Action");
        ArrayList<HashMap<String,ArrayList<Pair<Integer,Integer>>>> output = new ArrayList<HashMap<String,ArrayList<Pair<Integer,Integer>>>>();
        for(int i=0 ;i<length;i++){
            HashMap<String,ArrayList<Pair<Integer,Integer>>> temp_obj =new HashMap<String,ArrayList<Pair<Integer,Integer>>>();
            temp_obj.put(OBSTACLE_CHAR,new ArrayList<>());
            temp_obj.put(AGENT_CHAR,new ArrayList<>());
            temp_obj.put(GOAL_CHAR,new ArrayList<>(GOAL_LIST));
            RDDL.LCONST time_obj = new RDDL.OBJECT_VAL("$time"+String.valueOf(i));
            HashMap<RDDL.PVAR_EXPR,Double> state_val  = (HashMap<RDDL.PVAR_EXPR,Double>) state_object.get(time_obj);

            for(RDDL.PVAR_EXPR key : state_val.keySet()){
                if(key._pName.equals(new RDDL.PVAR_NAME("obstacle-at"))){
                    if(state_val.get(key)>FILTER_CRITERIA){
                        Integer temp_x = Integer.valueOf(((RDDL.OBJECT_VAL) key._alTerms.get(0))._sConstValue.substring(1,2));
                        Integer temp_y = Integer.valueOf(((RDDL.OBJECT_VAL) key._alTerms.get(1))._sConstValue.substring(1,2));
                        temp_obj.get(OBSTACLE_CHAR).add(new Pair<Integer,Integer>(temp_x,temp_y));
                    }
                }else if(key._pName.equals(new RDDL.PVAR_NAME("agent-at"))){
                    if(state_val.get(key)>FILTER_CRITERIA){
                        Integer temp_x = Integer.valueOf(((RDDL.OBJECT_VAL) key._alTerms.get(0))._sConstValue.substring(1,2));
                        Integer temp_y = Integer.valueOf(((RDDL.OBJECT_VAL) key._alTerms.get(1))._sConstValue.substring(1,2));
                        temp_obj.get(AGENT_CHAR).add(new Pair<Integer,Integer>(temp_x,temp_y));
                    }

                }

            }
            output.add(temp_obj);


        }


        return output;

    }

    public void runPlan(HashMap<String,HashMap<RDDL.LCONST,Object>> plan,Integer length){
        if(plan==null){
            System.out.print(testGenerateGrid());
            return;
        }

        ArrayList<HashMap<String,ArrayList<Pair<Integer,Integer>>>>  pritable_object = convertPlanToObjects(plan,length);
        for(int i=0 ; i<length;i++){
            System.out.println(generateGrid(pritable_object.get(i)));

        }



    }

    public static void main(String[] args){

        CrossingWorldUI obj = new CrossingWorldUI();
        obj.runPlan(null,null);
    }




}
