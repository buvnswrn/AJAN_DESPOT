package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation;

import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.State;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.World;

import java.util.HashMap;

public class AJANWorld extends World {
    HashMap<String,Object> variables;
    @Override
    public boolean Connect(){
        return false;
    }

    @Override
    public State Initialize() {
        return null;
    }

    @Override
    public State GetCurrentState() {
        return null;
    }

    @Override
    public void PrintState(State state) {

    }

    @Override
    public boolean ExecuteAction(int action, int obs) {
        return false;
    }


}
