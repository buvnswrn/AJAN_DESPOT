package com.ajan.POMDP.implementation;

import com.ajan.POMDP.State;

public class AJAN_Agent_State extends State {

    int agent_position;
    public AJAN_Agent_State(int _state_id, double weight) {
        super(_state_id, weight);
    }

    @Override
    public String text() {
        return agent_position==0?"LEFT":"RIGHT";
    }
}
