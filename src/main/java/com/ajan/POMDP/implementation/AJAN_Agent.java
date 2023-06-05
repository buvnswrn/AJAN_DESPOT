package com.ajan.POMDP.implementation;

import com.ajan.POMDP.Belief;
import com.ajan.POMDP.DSPOMDP;
import com.ajan.POMDP.State;
import com.ajan.POMDP.ValuedAction;
//import org.slf4j.Logger;
//import org.slf4j.LoggerFactory;

import java.util.Random;
import java.util.Vector;


public class AJAN_Agent extends DSPOMDP {

    static AJAN_Agent_State currentState;
    static double currentReward;
    static int currentObservation;
    static int currentAction;
    final int LEFT = 0;
    final int RIGHT = 1;
    final int HOVER = 2;
    final double NOISE = 0.15;
//     private final Logger LOGGER = LoggerFactory.getLogger(AJAN_Agent.class);

    public AJAN_Agent() {
        currentState = new AJAN_Agent_State(-1, 0.5); // Check for memory management here
//        InitializeObject(this);
    }
    @Override
    public boolean Step(AJAN_Agent_State state, double random_num, int action, double reward, int obs) {
        // TODO: Implement this in Knowledge Graphs -  Execute the Transition Function here. Out: AgentInfo,reward,obs
//        System.out.println("r_c_agent_position:{"+state.agent_position+"},action:{"+action+"},obs:{"+obs+"},reward:{"+reward+"}");
        currentState.state_id = state.state_id;
        currentState.scenario_id = state.scenario_id;
        currentState.agent_position = state.agent_position;
        currentState.weight = state.weight;
        currentAction = action;
        boolean terminal = false;
//        System.out.println("Executing Step");
        // region LOGIC here
        //TODO: Implementation can be in Knowledge Graphs
        if (action == LEFT || action == RIGHT) {
//            if(currentState.agent_position != currentAction){
//                currentReward = 10;
//            } else {
//                currentReward = -100;
////                System.out.println("Equal agent position...............");
//            }
            currentReward = state.agent_position != currentAction ? 10 : -100;
            currentState.agent_position = random_num <= 0.5 ? LEFT : RIGHT;
//            if(random_num<=0.5){
//                currentState.agent_position = LEFT;
//            } else {
//                currentState.agent_position = RIGHT;
//            }
            currentObservation = 2;
//            System.out.println("Entered in LEFT || RIGHT");
        } else {
            currentReward = -1;
//            System.out.println("Hovering the lane.......");
            if (random_num <= (1 - NOISE))
                currentObservation = currentState.agent_position;
            else
                currentObservation = (LEFT + RIGHT - currentState.agent_position);
        }
//        System.out.println("j_agent_position:{"+currentState.agent_position+"},obs:{"+currentObservation+"},reward:{"+currentReward+"}");
        //endregion
        return terminal;
    }

    @Override
    public int NumActions() {
        // TODO: KB Implementation
        return 3;
    }
    @Override
    public int NumStates() {
        // TODO: KB Implementation
        return 2;}

    @Override
    public double Reward(State state, int action) {
        // TODO: KB Implementation
        return 0;
    }

    @Override
    public double ObsProb(int obs, AJAN_Agent_State state, int action) {
        // TODO: Implementation needed in Knowledge Graphs - Execute the Observation Probability query here. Out: AgentInfo,reward,obs
        currentState = state;
        currentAction = action;
        currentObservation = obs;
        // check for the observation made
        if(currentAction !=HOVER)
            // return the
            return currentObservation == 2 ? 1 : 0;
        return currentState.agent_position == currentObservation ? (1-NOISE): NOISE;
    }

    @Override
    public State CreateStartState(String type) {
        // TODO: This also can be implemented in KB
        AJAN_Agent_State agentState = new AJAN_Agent_State(-1,0.5);
        agentState.agent_position = new Random().nextInt(2);
        return agentState;
    }

    @Override
    public Vector<State> getInitialBeliefParticles(State start, String type) {
        Vector<State> particles = new Vector<>();

        AJAN_Agent_State left = new AJAN_Agent_State(-1, 0.5);
        left.agent_position = LEFT;
        particles.add(left);
        AJAN_Agent_State right = new AJAN_Agent_State(-1, 0.5);
        right.agent_position = RIGHT;
        particles.add(right);

        return particles;
    }

//    public Vector<State> GetInitialBeliefParticles(State start, String type){
//        Vector<State> particles = new Vector<>();
//        AJAN_Agent_State left = new AJAN_Agent_State(-1,0.5);
//        particles.add(left);
//        AJAN_Agent_State right = new AJAN_Agent_State(-1, 0.5);
//        particles.add(right);
//        return particles;
//    }

    @Override
    public double GetMaxReward() {
        return 10;
    }

    @Override
    public ValuedAction GetBestAction() {
        return new ValuedAction(HOVER, -1);
    }

    @Override
    public void PrintState(State state) {
        AJAN_Agent_State agentState = (AJAN_Agent_State) state;
//        LOGGER.info(state.text());
        System.out.println(state.text());
    }

    @Override
    public void PrintObs(State state, int obs) {
//        LOGGER.info(String.valueOf(obs)); // (obs==LEFT?"LEFT":"RIGHT");
        System.out.println(String.valueOf(obs));
    }

    @Override
    public void PrintAction(int action) {

        if(action == LEFT) {
            System.out.println("Fly Left");
        } else if (action == RIGHT) {
            System.out.println("Fly Right");
        } else {
            System.out.println("Listen");
        }

    }

    @Override
    public void PrintBelief(Belief belief) {

    }

//    @Override
//    public State Copy(State state) {
//        return null;
//    }
//
//    @Override
//    public int NumActiveParticles() {
//        return 0;
//    }

    public String CreateScenarioLowerBound(String name, String particle_bound_name){
        return "DEFAULT";
    }

//    public void InitializeParameters(AJAN_Agent_State state,double random_num,int action,double reward,int obs){
//        System.out.println("Setting Agent Parameters");
//        currentState = state;
//        currentReward = reward;
//        currentAction = action;
//        currentObservation = obs;
//    }

    public AJAN_Agent getParameters(){
        System.out.println("Returning Agent Parameters");
        return this;
    }

    public void PrintMethod() {System.out.println("Received a call");}

//    private native void InitializeObject(AJAN_Agent agent);
}
