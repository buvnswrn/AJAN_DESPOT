package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.Problem;

import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.State;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation.AJAN_Agent;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation.AJAN_Agent_State;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Floor;

import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

public class Helper {
    static Map<String,Object> agentVariables;
    static Map<String, Class<?>> agentTypes;
    public static void declareVariables(AJAN_Agent model){
        agentVariables = new HashMap<String, Object>();
        agentTypes = new HashMap<>();
        agentVariables.put("current_",model);
        agentTypes.put("current_",AJAN_Agent.class);

        agentVariables.put("TAG_REWARD", 10);
        agentTypes.put("TAG_REWARD",int.class);

        agentVariables.put("NBEAMS", 8);
        agentTypes.put("NBEAMS",int.class);

        agentVariables.put("ONE", 1);
        agentTypes.put("ONE",int.class);

        agentVariables.put("BITS_PER_READING",7);
        agentTypes.put("BITS_PER_READING",int.class);

        agentVariables.put("same_loc_obs_",0);
        agentTypes.put("same_loc_obs_",int.class);

        agentVariables.put("noise_sigma_",0.5);
        agentTypes.put("noise_sigma_",float.class);

        agentVariables.put("unit_size",1.0);
        agentTypes.put("unit_size",float.class);

        agentVariables.put("default_action_",new Vector<Integer>());
        agentTypes.put("default_action_",Vector.class);

        agentVariables.put("reading_distributions_",new Vector<Vector<Vector<Double>>>());
        agentTypes.put("reading_distributions_",Vector.class);

        agentVariables.put("transition_probabilities_",new Vector<Vector<Vector<State>>>());
        agentTypes.put("transition_probabilities_",Vector.class);

        agentVariables.put("rob_",new Vector<Integer>());
        agentTypes.put("rob_",Vector.class);

        agentVariables.put("opp_",new Vector<Integer>());
        agentTypes.put("opp_",Vector.class);

        agentVariables.put("states_",new Vector<AJAN_Agent_State>());
        agentTypes.put("states_",Vector.class);

        agentVariables.put("robot_pos_unknown_",false);
        agentTypes.put("robot_pos_unknown_",boolean.class);

//        agentVariables.put("floor_",new Floor());
//        agentTypes.put("floor_",Floor.class);

        agentVariables.put("discount", 0.95 );
        agentTypes.put("discount",float.class);
    }

    public static Object get(String name){
        return agentVariables.get(name);
    }

    public static Class<?> getType(String name){
        return agentTypes.get(name);
    }

    public static void set(String name, Object value){
        agentVariables.put(name, value);
    }




}
