package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP;


import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation.AJANWorld;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation.AJAN_Agent;

public class AJANPlanner {
    public AJAN_Agent ajanAgent;
    public AJANWorld AJANWorld;
    static {
        System.loadLibrary("ajanplanner");
    }

    public boolean InitializeModel(){
        ajanAgent = new AJAN_Agent();
        AJANWorld = new AJANWorld(ajanAgent);
        return true;
    }

    public boolean InitializeWorld(String worldType) {
        AJANWorld = new AJANWorld(ajanAgent);
        return true;
    }
    public void PrintMethod() {System.out.println("Received a call");}
    public String ChooseSolver() {
        return "DESPOT";
    }
    public String getWorldType() {
        return "simulator";
    }

    private native int RunPlanner(AJAN_Agent ajanAgent, AJANWorld AJANWorld);
    private native void InitializePlannerInDespot();

    public static void main(String[] args) {
        AJANPlanner planner = new AJANPlanner();
        planner.InitializeModel();
        planner.InitializePlannerInDespot();
        planner.RunPlanner(planner.ajanAgent,planner.AJANWorld);
    }
}
