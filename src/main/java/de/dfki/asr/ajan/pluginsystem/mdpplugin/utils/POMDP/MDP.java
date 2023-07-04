package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP;

import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation.AJAN_Agent_State;

import java.util.Vector;

public interface MDP {
    Vector<ValuedAction> policy_ = new Vector<>();
    Vector<Vector<Double>> blind_alpha_ = new Vector<>();

    int NumStates();
    int NumActions();
    Vector<AJAN_Agent_State> TransitionProbability(int s, int action);

    double Reward(int s, int action);

//    void ComputeOptimalPolicyUsingVI(long reference);
//    Vector<ValuedAction> policy(long reference);
//    void ComputeBlindAlpha(long reference);
//    double ComputeActionValue(ParticleBelief belief,
//                              StateIndexer indexer, int action, long reference);
//    Vector<Vector<Double>> blind_alpha_(long reference);
}
