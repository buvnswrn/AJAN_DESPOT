package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP;

import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation.AJAN_Agent_State;

import java.util.Vector;

public interface POMCPPrior {
    Vector<Integer> legal_actions_ = new Vector<>();
    Vector<Integer> preferred_actions_ = new Vector<>();
    double exploration_constant = 0;
    History history_ = new History();

    void ComputePreference(AJAN_Agent_State state, long historyReference);
}
