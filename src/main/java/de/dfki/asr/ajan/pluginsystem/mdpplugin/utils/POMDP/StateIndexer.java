package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP;

import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation.AJAN_Agent_State;

public interface StateIndexer {

int NumStates();
int GetIndex(State state);

State GetState(int index);
}
