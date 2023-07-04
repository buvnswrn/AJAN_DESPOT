package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation;


import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.State;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Coord;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Floor;

import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.Problem.Helper.get;
import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.Variables.current_;
import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.Variables.floor_;

public class AJAN_Agent_State extends State {

//    String description;
    int agent_position; // TODO: Can be replaced with ArrayList or Vector<Integer>/Vector<std::any>
                        // denoting different states (i.e) LEFT = 1, RIGHT = 2 etc ...
    public AJAN_Agent_State(int _state_id, double weight) {
        super(_state_id, weight);
    }

//    @Override
    public static String text(int state_id, double weight) {
        AJAN_Agent agent_model = ((AJAN_Agent)get(current_));
        if(agent_model != null){
            int rob = agent_model.StateIndexToRobIndex(state_id);
            Coord rob_pos = ((Floor)get(floor_)).GetCell(rob);
            int opp = agent_model.StateIndexToOppIndex(state_id);
            Coord opp_pos = ((Floor)get(floor_)).GetCell(opp);
            return "Rob at " + rob_pos.toString() + ", Opp at " + opp_pos.toString();
        } else {
            return String.valueOf(state_id);
        }
    }
}
