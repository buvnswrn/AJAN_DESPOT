package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation;

import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.POMCPPrior;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Compass;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Coord;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Floor;

import java.util.HashMap;
import java.util.Map;

import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.Problem.Helper.get;


public class AJANPOMCPPrior implements POMCPPrior {

    Map<String, Object> variables;
    AJANPOMCPPrior(AJAN_Agent model) {
        variables = new HashMap<>();
        variables.put("agent_model",model);
        variables.put("legal_actions_",model);
    }
    @Override
    public void ComputePreference(AJAN_Agent_State state, long historyReference) {
        AJAN_Agent agent_model_ = (AJAN_Agent) variables.get("agent_model");
        Coord rob = agent_model_.GetRobPos(state);
        legal_actions_.clear();
        preferred_actions_.clear();

        for (int a = 0; a < 5; a++) {
            legal_actions_.add(a);
        }

        if (history_.Size(historyReference) != 0){
            if(history_.LastObservation(historyReference) == (int)get("same_loc_obs_")){
                preferred_actions_.add(agent_model_.TagAction());
            } else {
                if((boolean)get("robot_pos_unknown")){
                    for (int a = 0; a < 4; a++) {
                        if(((Floor)get("floor_")).Inside(
                                (Coord.Add(rob , Compass.DIRECTIONS[a])))){
                            if(!Compass.Opposite(a,history_.LastAction(historyReference))){
                                preferred_actions_.add(a);
                            }
                        }
                    }
                }
            }
        }
    }
}
