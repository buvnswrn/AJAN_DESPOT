package com.ajan.POMDP;

public abstract class DSPOMDP {
    public abstract boolean Step(State state, double random_num, int action, double reward, int obs);
    public abstract int NumActions();
    public abstract double Reward(State state, int action);
    public abstract double ObsProb(int obs,State state, int action);

    public abstract State CreateStartState(String type); // initialize the type to DEFAULT
    public abstract Belief InitialBelief(State start, String type); // initialize the type to DEFAULT
    public abstract double GetMaxReward();

    public abstract ValuedAction GetBestAction();

    public abstract void PrintState(State state);

    public abstract void PrintObs(State state, int obs);

    public abstract void PrintAction(int action);

    public abstract void PrintBelief(Belief belief);

    public abstract State Copy(State state);
    public abstract int NumActiveParticles();

}

