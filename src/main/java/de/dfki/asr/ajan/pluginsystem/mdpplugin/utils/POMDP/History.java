package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP;

import java.util.Vector;

public class History {
    Vector<Integer> actions_;
    Vector<Integer> observations_;

    void AddAction(int action) {
        actions_.add(action);
    }

    void AddObservation(int obs){
        observations_.add(obs);
    }

//    public native void Add(int action, int obs, long reference);
//    public native void RemoveLast( long reference);
//    public native int Action(int t, long reference);
//    public native int Observation(int t, long reference);
    public native int Size(long reference);
//    public native void Truncate(int d, long reference);
    public native int LastAction(long reference);
    public native int LastObservation(long reference);
//    public native History Suffix(int s, long reference);

}
