package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP;

public abstract class World {
    public abstract boolean Connect();
    public abstract State Initialize();
    public abstract State GetCurrentState();

    public abstract void PrintState(State state);
    public abstract boolean ExecuteAction(int action,int obs);
}
