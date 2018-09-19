/**
 * RDDL: Implements a random policy for a domain.
 * 
 **/

package rddl.policy;

import java.util.*;

import rddl.*;
import rddl.RDDL.*;
import util.Pair;


public class RandomPolicy extends Policy {
	Random rand_gen;
	final private static int NUM_TRIES = 30;
	final private static int MAX_DEPTH = 10;
	protected ArrayList<Pair<PVAR_NAME, ArrayList<LCONST>>> choices = null;
	
	protected void initializeChoices(State s) throws Exception{
		choices = new ArrayList<>();
		for(final PVAR_NAME p : s._alActionNames){
			for(final ArrayList<LCONST> inst : s.generateAtoms(p)){
				choices.add(new Pair<>(p, inst));
			}
		}
	}
	
	public RandomPolicy () {
		this.rand_gen = new Random();
	}
	
	public RandomPolicy(String instance_name) {

		super(instance_name);
		this.rand_gen = new Random();

	}

	public ArrayList<PVAR_INST_DEF> getActions(State s) throws EvalException {
		ArrayList<PVAR_INST_DEF> ret = new ArrayList<>();
		
		if (this.choices == null){
			try{
				initializeChoices(s);

			}catch (Exception ex){
				ex.printStackTrace();
			}

		}
		Collections.shuffle(choices);

		for( Pair<PVAR_NAME, ArrayList<LCONST>> choice : choices ){
			final PVAR_NAME p = choice._o1;
			final ArrayList<LCONST> inst = choice._o2;
			
			// Get type of action var
			final PVARIABLE_DEF pdef = s._hmPVariables.get(p);
			final TYPE_NAME tdef = pdef._typeRange;

			if( tdef.equals(TYPE_NAME.BOOL_TYPE) ){
				final Boolean val = _random.nextUniform(0,1) > 0.5 ? true : false;    //rand_gen.nextBoolean();
				ret.add(new PVAR_INST_DEF(p._sPVarName, val, inst) );
				try{
					s.checkStateActionConstraints(ret);
				}catch(EvalException exc){
					ret.remove(ret.size()-1);
					ret.add(new PVAR_INST_DEF(p._sPVarName, !val, inst) );
					try{
						s.checkStateActionConstraints(ret);
					}catch(EvalException exc1){
						ret.remove(ret.size()-1);
					}
				}
			}else if( tdef.equals(TYPE_NAME.INT_TYPE) ){
				for( int attempt = 0; attempt < NUM_TRIES; ++attempt ){
					final Integer val = _random.nextInt(1,10); //rand_gen.nextInt();
					ret.add(new PVAR_INST_DEF(p._sPVarName, val, inst) );
					try{
						s.checkStateActionConstraints(ret);
						break;
					}catch(EvalException exc){
						ret.remove(ret.size()-1);
					}
				}
			}else if( tdef.equals(TYPE_NAME.REAL_TYPE) ){
				for( int attempt = 0; attempt < NUM_TRIES; ++attempt ){
					final Double val = _random.nextUniform(1,10);//rand_gen.nextDouble();
					ret.add(new PVAR_INST_DEF(p._sPVarName, val, inst) );
					try{
						s.checkStateActionConstraints(ret);
						break;
					}catch(EvalException exc){
						ret.remove(ret.size()-1);
					}
				}
			}else{
				//enum?
				final ENUM_TYPE_DEF edef = (ENUM_TYPE_DEF)s._hmTypes.get(tdef);
				final ArrayList<ENUM_VAL> enums = new ArrayList<ENUM_VAL>((ArrayList)edef._alPossibleValues);
				for( int attempt = 0; attempt < NUM_TRIES && !enums.isEmpty(); ++attempt ){
					final int rand_index = rand_gen.nextInt(enums.size());
					ret.add(new PVAR_INST_DEF(p._sPVarName, 
							(Object)(enums.get(rand_index)),inst));
					try{
						s.checkStateActionConstraints(ret);
						break;
					}catch(EvalException exc){
						ret.remove(ret.size()-1);
						enums.remove(rand_index);
					}
				}
			}
			//System.out.println(ret);
		}
		
		try{
			s.checkStateActionConstraints(ret);
		}catch(EvalException exc){
			//exhaustive : make list of all legal actions
			try{
				Collections.shuffle(choices);
				ArrayList<ArrayList<PVAR_INST_DEF>> legal_actions = new ArrayList<>();
				getLegalActions(s, new ArrayList<>(), 0, legal_actions);
				ret = legal_actions.get(rand_gen.nextInt(legal_actions.size()));
			}catch(EvalException exc1){
				ret = new ArrayList<PVAR_INST_DEF>();	
			}
		}
		return ret;
	}

	protected void getLegalActions(final State s,
			final ArrayList<PVAR_INST_DEF> cur_action, 
			final int start_index,
			final ArrayList<ArrayList<PVAR_INST_DEF>> legal_actions) throws EvalException{
		//System.out.println("Current Index : " +String.valueOf(start_index) +  "  Action : " +  cur_action.toString() );
		if( start_index < MAX_DEPTH ){
			Pair<PVAR_NAME, ArrayList<LCONST>> choice = choices.get(start_index);
			final PVAR_NAME p = choice._o1;
			final ArrayList<LCONST> instantiations = choice._o2;
			
			// Get type of action var
			final PVARIABLE_DEF pdef = s._hmPVariables.get(p);
			final TYPE_NAME tdef = pdef._typeRange;
			
			
			if( tdef.equals(TYPE_NAME.BOOL_TYPE) ){


				ArrayList<PVAR_INST_DEF> tmp_true = new ArrayList<PVAR_INST_DEF>(cur_action);
				tmp_true.add(new PVAR_INST_DEF(p._sPVarName,Boolean.TRUE,instantiations));
				getLegalActions(s, tmp_true, start_index+1, legal_actions);

				ArrayList<PVAR_INST_DEF> tmp_false = new ArrayList<PVAR_INST_DEF>(cur_action);
				tmp_false.add(new PVAR_INST_DEF(p._sPVarName, Boolean.FALSE, instantiations));
				getLegalActions(s, tmp_false, start_index+1, legal_actions);
				
				//noop choice
				getLegalActions(s, cur_action, start_index+1, legal_actions);
			}
			else if( s._hmTypes.get(tdef) instanceof ENUM_TYPE_DEF ){
				final ENUM_TYPE_DEF edef = (ENUM_TYPE_DEF)s._hmTypes.get(tdef);
				final ArrayList<ENUM_VAL> enums = new ArrayList<ENUM_VAL>((ArrayList)edef._alPossibleValues);

				for( final ENUM_VAL eval : enums ){
					ArrayList<PVAR_INST_DEF> this_tmp = new ArrayList<PVAR_INST_DEF>(cur_action);
					this_tmp.add(new PVAR_INST_DEF(p._sPVarName, eval , instantiations));
					getLegalActions(s, this_tmp, start_index+1, legal_actions);
				}
				//noop choice
				getLegalActions(s, cur_action, start_index+1, legal_actions);
			}
		}
		//gets here when each choice is assigned a value
		try{
			s.checkStateActionConstraints(cur_action);
			legal_actions.add(cur_action);
		}catch(EvalException exc){
			//System.out.println("Legal Actions : "+ legal_actions.toString());

		}
	}
}
