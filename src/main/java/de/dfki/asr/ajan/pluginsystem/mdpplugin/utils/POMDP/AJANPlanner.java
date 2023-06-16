package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP;


import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation.AJANWorld;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation.AJAN_Agent;

public class AJANPlanner {
    public AJAN_Agent ajanAgent;
    public AJANWorld ajanWorld;
    static {
        System.loadLibrary("ajanplanner");
    }

    public boolean InitializeModel(){
        ajanAgent = new AJAN_Agent();
        return true;
    }

    public boolean InitializeWorld(String worldType) {
        ajanWorld = new AJANWorld(ajanAgent);
        return true;
    }
    public void PrintMethod() {System.out.println("Received a call");}
    public String ChooseSolver() {
        return "DESPOT";
    }

    private native int RunPlanner(AJAN_Agent ajanAgent, AJANWorld ajanWorld);
    private native void InitializePlannerInDespot();

    public static void main(String[] args) {
        AJANPlanner planner = new AJANPlanner();
        planner.InitializeModel();
        planner.InitializePlannerInDespot();
        planner.RunPlanner(planner.ajanAgent,planner.ajanWorld);
    }
}
