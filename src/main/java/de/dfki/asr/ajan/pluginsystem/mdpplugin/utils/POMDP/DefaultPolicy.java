package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP;

import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation.AJAN_Agent_State;

import java.util.Vector;

public interface DefaultPolicy {
//    int Action(Vector<AJAN_Agent_State> particles, History history, long historyReference);
    int Action(Vector<AJAN_Agent_State> particles, long historyReference);
}
