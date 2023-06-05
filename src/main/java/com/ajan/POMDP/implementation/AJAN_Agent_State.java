package com.ajan.POMDP.implementation;

import com.ajan.POMDP.State;

public class AJAN_Agent_State extends State {

//    String description;
    int agent_position; // TODO: Can be replaced with ArrayList or Vector<Integer>/Vector<std::any>
                        // denoting different states (i.e) LEFT = 1, RIGHT = 2 etc ...
    public AJAN_Agent_State(int _state_id, double weight) {
        super(_state_id, weight);
    }

    @Override

    public String text() {
        return agent_position==0?"LEFT":"RIGHT";
    }
}
