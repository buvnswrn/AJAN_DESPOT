package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP;

import java.util.Map;
import java.util.Vector;

public interface BeliefMDP {
    Vector<State> getTauParticles(Vector<State> particles, int action, int obs);

    void Observe(Belief belief, int action, Map<Integer, Double> obss);

    double StepRewardFromParticles(Vector<State> belief, int action);

}
