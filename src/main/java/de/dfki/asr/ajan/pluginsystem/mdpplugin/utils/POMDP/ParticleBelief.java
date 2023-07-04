package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP;

import java.util.Vector;

public class ParticleBelief extends Belief{

    public ParticleBelief(DSPOMDP model) {
        super(model);
    }

    @Override
    Vector<State> Sample(int num) {
        // TODO: write code to contact c++ class itself
        return null;
    }

    @Override
    void Update(int action, int obs) {
        // TODO: Implement this class. For now it is not needed, routing to Belief MDP.
    }

    @Override
    String text() {
        return null;
    }

    @Override
    Belief MakeCopy() {
        return null;
    }
}
