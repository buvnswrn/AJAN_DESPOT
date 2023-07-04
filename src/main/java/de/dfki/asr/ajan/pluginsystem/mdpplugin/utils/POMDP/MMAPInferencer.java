package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP;

import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation.AJAN_Agent_State;

import java.util.Vector;

public interface MMAPInferencer {
    AJAN_Agent_State GetMMAP(Vector<AJAN_Agent_State> particles);
}
