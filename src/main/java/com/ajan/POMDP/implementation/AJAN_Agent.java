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
    final int LEFT = 0;
    final int RIGHT = 1;
    final int HOVER = 2;
    final double NOISE = 0.15;
//     private final Logger LOGGER = LoggerFactory.getLogger(AJAN_Agent.class);

    public AJAN_Agent() {
//        InitializeObject(this);
    }
    @Override
    public boolean Step(State state, double random_num, int action, double reward, int obs) {
        AJAN_Agent_State agentState = (AJAN_Agent_State) state;
        boolean terminal = false;
        if (action == LEFT || action == RIGHT) {
            reward = agentState.agent_position != action ? 10 : -100;
            agentState.agent_position = random_num <= 0.5 ? LEFT : RIGHT;
            obs = 2;
        } else {
            reward = -1;
            if (random_num <= 1 - NOISE)
                obs = agentState.agent_position;
            else
                obs = (LEFT + RIGHT - agentState.agent_position);
        }
        return terminal;
    }

    @Override
    public int NumActions() {
        return 3;
    }

    @Override
    public double Reward(State state, int action) {
        return 0;
    }

    @Override
    public double ObsProb(int obs, State state, int action) {
        AJAN_Agent_State agentState = (AJAN_Agent_State) state; // typecast the state to agent state
        // check for the observation made
        if(action !=HOVER)
            // return the
            return obs == 2 ? 1 : 0;
        return agentState.agent_position == obs ? (1-NOISE): NOISE;
    }

    @Override
    public State CreateStartState(String type) {
        AJAN_Agent_State agentState = new AJAN_Agent_State(-1,0.5);
        agentState.agent_position = new Random().nextInt(2);
        return agentState;
    }

    @Override
    public Belief InitialBelief(State start, String type) {
        return null;
    }

    public Vector<State> GetInitialBeliefParticles(State start, String type){
        Vector<State> particles = new Vector<>();
        AJAN_Agent_State left = new AJAN_Agent_State(-1,0.5);
        particles.add(left);
        AJAN_Agent_State right = new AJAN_Agent_State(-1, 0.5);
        particles.add(right);
        return particles;
    }

    @Override
    public double GetMaxReward() {
        return 0;
    }

    @Override
    public ValuedAction GetBestAction() {
        return null;
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

    }

    @Override
    public void PrintBelief(Belief belief) {

    }

    @Override
    public State Copy(State state) {
        return null;
    }

    @Override
    public int NumActiveParticles() {
        return 0;
    }

    public String CreateScenarioLowerBound(String name, String particle_bound_name){
        return "DEFAULT";
    }

//    private native void InitializeObject(AJAN_Agent agent);
}
