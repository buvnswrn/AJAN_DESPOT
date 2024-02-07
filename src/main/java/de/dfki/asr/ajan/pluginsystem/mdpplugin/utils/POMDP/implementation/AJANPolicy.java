package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation;

import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.DefaultPolicy;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.History;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Compass;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Coord;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Floor;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.Vector;

import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.Problem.Helper.get;
import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Coord.Add;

public class AJANPolicy implements DefaultPolicy {
    Map<String, Object> variables;
    public String name;

    public AJANPolicy(AJAN_Agent model){
        variables = new HashMap<>();
        variables.put("agent_model_", model);
    }

    public int TestMethod() {
        System.out.println("TestMethod Invoked");
        return 0;
    }

    @Override
    public int Action(Vector<AJAN_Agent_State> particles, long historyReference) {
        Floor floor_ = (Floor)get("floor_");
        History history = new History();
        AJAN_Agent agent_model_ = (AJAN_Agent) variables.get("agent_model_");

        if(history.Size(historyReference) == 0){
            Random random = new Random();
            return random.nextInt((agent_model_.NumActions() -1));
        }

        Coord rob;

        if((int)get("same_loc_obs_") != (floor_.NumCells())){
            rob = agent_model_.MostLikelyRobPosition(particles);
        } else {
            rob = floor_.GetCell(history.LastObservation(historyReference));
        }

        Coord opp;
        opp = agent_model_.MostLikelyOppPosition(particles);
        double distance = Coord.ManhattanDistance(rob, opp);

        if(distance <=1){
            return agent_model_.TagAction();
        }

        Vector<Integer> actions = new Vector<>();
        for (int d = 0; d < 4; d++) {
            if(!Compass.Opposite(d, history.LastAction(historyReference)) && floor_.Inside(Add(rob,Compass.DIRECTIONS[d]))){
                actions.add(d);
            }
        }
        if (actions.size() == 0) {
            for (int d = 0; d < 4; d++) {
                if(floor_.Inside(Add(rob, Compass.DIRECTIONS[d]))){
                    actions.add(d);
                }
            }
        }

        if(actions.size() == 0){
            return 0;
        }
        Random random = new Random();
        int action = actions.get(random.nextInt(actions.size()));
        return action;
    }
}
