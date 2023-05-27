package com.ajan.POMDP;


import com.ajan.POMDP.implementation.AJAN_Agent;

public class AJANPlanner {
    public AJAN_Agent ajanAgent;
    static {
        System.loadLibrary("AJANPlanner");
    }

    public boolean InitializeModel(){
        ajanAgent = new AJAN_Agent();
        return true;
    }

    public boolean InitializeWorld() {
        // TODO: See how to implement World class
        return true;
    }

    public String ChooseSolver() {
        return "DESPOT";
    }

    private native int RunPlanner();
    private native void InitializePlannerInDespot();

    public static void main(String[] args) {
        AJANPlanner planner = new AJANPlanner();
        planner.RunPlanner();
    }
}
