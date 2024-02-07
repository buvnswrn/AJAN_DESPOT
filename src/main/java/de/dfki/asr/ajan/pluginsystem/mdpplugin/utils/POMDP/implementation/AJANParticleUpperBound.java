package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation;

import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.State;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Floor;

import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.Problem.Helper.get;
import static java.lang.Math.pow;

public class AJANParticleUpperBound {
    Map<String, Object> variables;
    AJANParticleUpperBound(AJAN_Agent model){
        variables = new HashMap<>();
        Vector<Double> value_ = new Vector<Double>(model.NumStates());
        variables.put("value_",value_);

        Floor floor = (Floor)get("floor_");
        for (int s = 0; s < model.NumStates(); s++) {
            int rob = ((Vector<Integer>)get("rob_")).get(s);
            int opp = ((Vector<Integer>)get("opp_")).get(s);
            int dist = (int) floor.Distance(rob, opp);
            value_.add(s, -(1-discount(dist)/(1-discount())) + (int) get("TAG_REWARD") * discount(dist));
        }
    }

    private double discount(int dist) {
        return pow((int)get("discount"), dist);
    }

    private double discount() {
        return (double) get("discount");
    }

    public double Value(int state_id){
        return ((Vector<Double>)variables.get("value_")).get(state_id);
    }
//    public double Value(AJAN_Agent_State s, int action){
//        return ((Vector<Double>)variables.get("value_")).get(s.state_id);
//    }

}
