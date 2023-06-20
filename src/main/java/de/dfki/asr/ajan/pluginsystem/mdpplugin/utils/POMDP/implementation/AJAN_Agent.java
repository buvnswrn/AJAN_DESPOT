package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation;


import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.Belief;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.DSPOMDP;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.State;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.ValuedAction;

import java.util.*;



public class AJAN_Agent extends DSPOMDP {

    static AJAN_Agent_State currentState;
    public static double currentReward;
    public static int currentObservation;
    public static int currentAction;
    final int LEFT = 0;
    final int RIGHT = 1;
    final int HOVER = 2;
    final double NOISE = 0.15;
    HashMap<String,?> variables;
//     private final Logger LOGGER = LoggerFactory.getLogger(AJAN_Agent.class);

    public AJAN_Agent() {
        currentState = new AJAN_Agent_State(-1, 0.5); // Check for memory management here
        /*
         Vector<Vector<Vector<State>>> transition_probabilities = new Vector<>();
         transition_probabilities.setSize(NumStates());
        */
    }
    @Override
    public boolean Step(AJAN_Agent_State state, double random_num, int action, double reward, int obs) {
        // TODO: Implement this in Knowledge Graphs -  Execute the Transition Function here. Out: AgentInfo,reward,obs
        currentState.state_id = state.state_id;
        currentState.scenario_id = state.scenario_id;
        currentState.agent_position = state.agent_position;
        currentState.weight = state.weight;
        currentAction = action;
        boolean terminal = false;
        // region LOGIC here
        //TODO: Implementation can be in Knowledge Graphs
        Random random = new Random((long) random_num);
        random_num = random.nextDouble();
        //region base_tag
        if(action == (int)variables.get("TAG")){
            double distance = (int)variables.get("distance"); //TODO: change the distance function here
            if(distance <=1){
                reward = (int)variables.get("TAG_REWARD");
                terminal = true;
            } else {
                reward =  - (int) variables.get("TAG_REWARD");
            }
        } else {
            reward = -1;
        }
        final Vector<State> distribution = getTransitionProbabilities(state,action);

        double sum =0;
        for (int i = 0; i < Objects.requireNonNull(distribution).size(); i++) {
            final State next = distribution.get(i);
            sum+=next.weight;
            if(sum >=random_num){
                currentState.state_id = next.state_id; // FIXME: How to change the state id
                break;
            }
        }
        //endregion
        //region laser_tag
        if(terminal) {
            currentObservation = (int)variables.get("same_loc_obs");
        } else {
            if(((State)variables.get("rob_")).state_id == ((State)variables.get("rob_")).state_id ) {
                currentObservation = (int) variables.get("same_loc_obs");
            } else {
                Vector<Vector<Double>> laser_tag_distribution = getReadingDistribution(state.state_id);

                currentObservation = 0;
                for (int dir = 0; dir < (int) variables.get("NBEAMS"); dir++) {
                    double mass = random.nextDouble();
                    int reading = 0;
                    for(; reading< laser_tag_distribution.get(dir).size();reading++){
                        SetReading(reading, dir);
                    }
                }
            }
        }
        //endregion
        //endregion
        return terminal;
    }

    private Vector<Vector<Double>> getReadingDistribution(int stateId) {
    return null;
    }

    private Vector<State> getTransitionProbabilities(AJAN_Agent_State state, int action) {
        return null;
    }
    private void SetReading(int observation, int dir) {
        currentObservation &=  ~(((int)variables.get("ONE")<< (int)variables.get("BITS_PER_READING")-1)
                <<(dir *(int)variables.get("BITS_PER_READING")));
        currentObservation |= observation << (dir * (int)variables.get("BITS_PER_READING"));
    }

    @Override
    public int NumActions() {
        // TODO: KB Implementation
        return 3;
    }
    @Override
    public int NumStates() {
        // TODO: KB Implementation
        return 2; // return floor_.NumCells()* floor_.NumCells();
    }

    @Override
    public double Reward(State state, int action) {
        // TODO: KB Implementation
        return 0;
    }

    @Override
    public double ObsProb(int obs, AJAN_Agent_State state, int action) {
        // TODO: Implementation needed in Knowledge Graphs - Execute the Observation Probability query here. Out: AgentInfo,reward,obs
        if(action !=HOVER)
            return obs == 2 ? 1 : 0;
        return state.agent_position == obs ? (1-NOISE): NOISE;
    }

    @Override
    public AJAN_Agent_State CreateStartState(String type) {
        // TODO: This also can be implemented in KB
        AJAN_Agent_State agentState = new AJAN_Agent_State(-1,0.5);
        agentState.agent_position = new Random().nextInt(2);
        return agentState;
    }

    @Override
    public Vector<AJAN_Agent_State> getInitialBeliefParticles(State start, String type) {
        System.out.println("Got call to initial belief");
        Vector<AJAN_Agent_State> particles = new Vector<>();
        AJAN_Agent_State left = new AJAN_Agent_State(-1, 0.5);
        left.agent_position = LEFT;
        particles.add(left);
        AJAN_Agent_State right = new AJAN_Agent_State(-1, 0.5);
        right.agent_position = RIGHT;
        particles.add(right);

        return particles;
    }

    @Override
    public double GetMaxReward() {
        return 0;
    }

    @Override
    public ValuedAction GetBestAction() {
        return new ValuedAction(HOVER, -1);
    }

    @Override
    public void PrintState(State state) {
        AJAN_Agent_State agentState = (AJAN_Agent_State) state;
        int aindex = ((int[])variables.get("rob_"))[state.state_id];
        int oindex = ((int[])variables.get("opp_"))[state.state_id];
//        LOGGER.info(state.text());
        int floor_rows = getFloorRows();
        for (int y = floor_rows; y >= 0; y--) {
            int index = getFloorRows();
        }
        System.out.println(agentState.text());
    }

    private int getFloorRows() {
        return 0;
    }

    @Override
    public void PrintObs(State state, int obs) {
        System.out.println(obs);
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

    @Override
    public String CreateScenarioLowerBound(String name, String particle_bound_name){
        return "DEFAULT";
    }

    public AJAN_Agent getParameters(){
        System.out.println("Returning Agent Parameters");
        return this;
    }

    public void PrintMethod() {System.out.println("Received a call");}

//    private native void InitializeObject(AJAN_Agent agent);
}
