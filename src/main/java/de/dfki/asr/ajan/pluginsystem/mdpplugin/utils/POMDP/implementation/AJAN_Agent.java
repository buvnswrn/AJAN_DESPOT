package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation;


import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.*;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Coord;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Floor;

import java.util.*;

import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.Connector.ROSConnector.GetReading;
import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.Problem.Helper.declareVariables;
import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.Problem.Helper.set;
import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.Problem.Helper.get;
import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Compass.DIRECTIONS;
import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Coord.Add;
import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.AJAN_Util_Helper.*;
import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Coord.Muliply;
import static java.lang.Math.*;
import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.Variables.*;


public class AJAN_Agent extends DSPOMDP implements StatePolicy,MDP, StateIndexer, MMAPInferencer, BeliefMDP {

    static AJAN_Agent_State currentState;
    static AJANParticleUpperBound currentParticleUpperBound;
    static AJANPolicy currentAjanPolicy;
    public static double currentReward;
    public static int currentObservation;
    public static int currentAction;
    public long cppReference;

    //region Application Specific Variables
    //endregion
//     private final Logger LOGGER = LoggerFactory.getLogger(AJAN_Agent.class);

    public AJAN_Agent() {
        currentState = new AJAN_Agent_State(-1, 0.5); // Check for memory management here
        declareVariables(this);
        // region base_tag
        Init(BenchmarkMap());
        /*
         Vector<Vector<Vector<State>>> transition_probabilities_ = new Vector<>();
         transition_probabilities_.setSize(NumStates());
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
        if(action == TagAction()){
            double distance_ = Coord.ManhattanDistance(((Floor)get(floor_)).GetCell(
                    ((Vector<Integer>)get(rob_)).get(state.state_id)),((Floor)get(floor_)).GetCell(
                            ((Vector<Integer>)get(opp_)).get(state.state_id)));
            if(distance_ <=1){
                currentReward = (int)get(TAG_REWARD);
                terminal = true;
            } else {
                currentReward =  - (int) get(TAG_REWARD);
            }
        } else {
            currentReward = -1;
        }

        final Vector<AJAN_Agent_State> distribution = TransitionProbability(state.state_id,action);

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
            currentObservation = (int)get(same_loc_obs_);
        } else {
            if(((State)get(rob_)).state_id == ((State)get(rob_)).state_id ) {
                currentObservation = (int) get(same_loc_obs_);
            } else {
                Vector<Vector<Double>> laser_tag_distribution = ((Vector<Vector<Double>>)((Vector<?>)get(reading_distributions_)).get(state.state_id));
                currentObservation = 0;
                for (int dir = 0; dir < (int) get(NBEAMS); dir++) {
                    double mass = random.nextDouble();
                    int reading = 0;
                    for(; reading< laser_tag_distribution.get(dir).size();reading++){
                        mass -= laser_tag_distribution.get(dir).get(reading);
                        if(mass < 1e-8){ // Globals::TINY = 1e-8
                            break;
                        }
                    }
                    currentObservation = SetReading(currentObservation, reading, dir);
                }
            }
        }
        //endregion
        //endregion
        return terminal;
    }

    @Override
    public int NumActions() {
        // TODO: KB Implementation
        return 5;
    }

    @Override
    public int NumStates() {
        // TODO: KB Implementation
        return ((Floor)get(floor_)).NumCells()* ((Floor)get(floor_)).NumCells();
    }

    @Override
    public double ObsProb(int obs, AJAN_Agent_State state, int action) {
        // TODO: Implementation needed in Knowledge Graphs - Execute the Observation Probability query here. Out: AgentInfo,reward,obs
        if(Objects.equals(
                ((Vector<?>)get(rob_)).get(state.state_id),
                ((Vector<?>)get(opp_)).get(state.state_id)
        )
        )
            return (obs == (int)get(same_loc_obs_))?0:1;
        double prod = 1.0;
        for (int dir = 0; dir < (int) get(NBEAMS); dir++) {
            int reading = GetReading(obs, dir);
            if(reading>= LaserRange(state, dir)/(float) get(unit_size))
                return 0;
            double prob_mass = ((Vector<Vector<Vector<Double>>>)get(reading_distributions_))
                                    .get(state.state_id).get(dir).get(reading);
            prod *= prob_mass;
        }
        return prod;
    }

    @Override
    public AJAN_Agent_State CreateStartState(String type) {
        // TODO: This also can be implemented in KB
        int n = new Random().nextInt(((Vector<?>)get(states_)).size());
        AJAN_Agent_State agentState = (AJAN_Agent_State) ((Vector<?>)get(states_)).get(n);
//        agentState.agent_position = new Random().nextInt(2);
        return agentState;
    }
    @Override
    public Vector<AJAN_Agent_State> getInitialBeliefParticles(State start, String type) {
        System.out.println("Got call to initial belief");
        Vector<AJAN_Agent_State> particles = new Vector<>();
        int N = ((Floor)get(floor_)).NumCells();
        double wgt = (1.0/N)/N;
        for(int rob = 0; rob <N; rob++){
            for (int opp = 0; opp < N; opp++) {
                AJAN_Agent_State state = new AJAN_Agent_State(RobOppIndicesToStateIndex(rob, opp), wgt);
                particles.add(state);
            }
        }
        return particles;
    }
    @Override
    public double GetMaxReward() {
        return 0;
    }

    @Override
    public double Reward(State state, int action) {
        // TODO: KB Implementation
        System.out.println("Unimplemented Reward Function Hit!!");
        return 0;
    }

    @Override
    public double Reward (int s, int action) {
        AJAN_Agent_State state = (AJAN_Agent_State)((Vector<?>)get(states_)).get(s);
        double reward = 0;
        if(action == TagAction()){
            if((int)((Vector<?>)get(rob_)).get(state.state_id) ==
                    (int)((Vector<?>)get(opp_)).get(state.state_id)){
                reward = (int)(get(TAG_REWARD));
            } else {
                reward =  - (int)(get(TAG_REWARD));
            }
        } else {
            reward = -1;
        }
        return reward;
    }

    @Override
    public ValuedAction GetBestAction() {
        return new ValuedAction(2,-1);
    }

    @Override
    public int GetAction(State state) {
        return (int)((Vector<?>)get(default_action)).get(GetIndex(state));
    }

    @Override
    public AJANPolicy CreateScenarioLowerBound(String name, String particle_bound_name){
        System.out.println("CreateScenarioLowerBound in Java"+name);
        final Floor floor = (Floor)get(floor_);
        final int  same_loc_obs = (int) get(same_loc_obs_);
        final int numCells = floor.NumCells();
        System.out.println("CreateScenarioLowerBound: Checking conditions");
        if(Objects.equals(name, "DEFAULT") && same_loc_obs != numCells){
            System.out.println("CreateScenarioLowerBound: currentAjanPolicy enter");
            currentAjanPolicy = new AJANPolicy(this);
            currentAjanPolicy.name = "AJANPolicy";
            System.out.println("CreateScenarioLowerBound: currentAjanPolicy");
            return currentAjanPolicy;
//            return "AJANPolicy";
        } else if (Objects.equals(name, "DEFAULT") && same_loc_obs == numCells){
            System.out.println("CreateScenarioLowerBound:NULL-Enter");
            currentAjanPolicy = new AJANPolicy(this);
            currentAjanPolicy.name = "ModeStatePolicy";
//            ComputeDefaultActions("MDP"); // This type should be MDP but we are using SP for now
//            return "ModeStatePolicy";
            System.out.println("CreateScenarioLowerBound:NULL");
            return null;
        } else {
            if(!Objects.equals(name, "print"))
                System.out.println("Unsupported lower bound: " + name );
            System.out.println("Supported types: DEFAULT");
            System.out.println( "With base lower bound: except TRIVIAL " );
            System.out.println("CreateScenarioLowerBound:NULL2");
            return null;
        }
    }

    @Override
    public String CreateScenarioUpperBound(String name, String particle_bound_name){
        return name;
    }

    @Override
    public AJANParticleUpperBound CreateParticleUpperBound(String name) {
        if(Objects.equals(name, "SP") || Objects.equals(name, "DEFAULT")){
            currentParticleUpperBound = new AJANParticleUpperBound(this);
        }
        return currentParticleUpperBound;
    }

    @Override
    public String CreateParticleLowerBound(String name){
        return "AJANLowerBound";
    }

    @Override
    public String CreatePOMCPPrior(String name) {
        return "UniformPOMCPPrior";
    }

    @Override
    public Vector<AJAN_Agent_State> TransitionProbability(int s, int action) {
        return (Vector<AJAN_Agent_State>) ((Vector<?>)((Vector<?>)get(transition_probabilities_)).get(s)).get(action);
    }

    @Override
    public int GetIndex(State state) {
        return state.state_id;
    }

    @Override
    public State GetState(int index) {
        return ((Vector<AJAN_Agent_State>)get(states_)).get(index);
    }

    @Override
    public void PrintState(State state) {
        AJAN_Agent_State agentState = (AJAN_Agent_State) state;
        int aindex = (int)((Vector<?>)get(rob_)).get(state.state_id);
        int oindex = (int)((Vector<?>)get(opp_)).get(state.state_id);
//        LOGGER.info(state.text());
        int floor_rows = getFloorRows();
        for (int y = floor_rows; y >= 0; y--) {
            int index = getFloorRows();
        }
        System.out.println(agentState.text(state.state_id, state.weight));
    }

    @Override
    public void PrintObs(State state, int obs) {
        System.out.println(obs);
    }

    @Override
    public void PrintAction(int action) {

    }

    @Override
    public void PrintBelief(Belief belief) {

    }

    @Override
    public AJAN_Agent_State GetMMAP(Vector<AJAN_Agent_State> particles) {
        Coord rob = MostLikelyRobPosition(particles);
        Coord opp = MostLikelyOppPosition(particles);
        int state_id = RobOppIndicesToStateIndex(((Floor)get(floor_)).GetIndex(rob),
                ((Floor)get(floor_)).GetIndex(opp));
        return (AJAN_Agent_State) ((Vector<?>)get(states_)).get(state_id);
    }

    @Override
    public Vector<State> getTauParticles(Vector<State> particles, int action, int obs) {
        Vector<Double> probs = new Vector<>();
        double sum = 0;
        for (int i = 0; i < particles.size(); i++) {
            AJAN_Agent_State state = (AJAN_Agent_State) particles.get(i);
            Vector<State> distribution = (Vector<State>)((Vector<?>)((Vector<?>)get(transition_probabilities_)).get(GetIndex(state))).get(action);
            for (int j = 0; j < distribution.size(); j++) {
                State next = distribution.get(j);
                double p = state.weight * next.weight * ObsProb(obs,((AJAN_Agent_State)((Vector<?>)get(states_)).get(next.state_id)),action);
                double temp = probs.get(next.state_id) != null ? probs.get(next.state_id):0;
                probs.add(next.state_id, temp+p);
                sum+=p;
            }
        }
        Vector<State> new_particles = new Vector<>();
        for (int i = 0; i < NumStates(); i++) {
            if(probs.get(i)>0){
                State new_particle = (State) ((Vector<?>)get(states_)).get(i);
                new_particle.weight = probs.get(i)/sum;
                new_particles.add(new_particle);
                probs.add(i, 0.0);
            }
        }
        return new_particles;
    }

    @Override
    public void Observe(Belief belief, int action, Map<Integer, Double> obss) {
        System.out.println("Observer function Not implemented");
    }

    @Override
    public double StepRewardFromParticles(Vector<State> particles, int action) {
        double sum = 0;
        for (int i = 0; i < particles.size(); i++) {
            State particle = particles.get(i);
            AJAN_Agent_State state = (AJAN_Agent_State) particle;
            double reward = 0;
            if(action == TagAction()) {
                if(((Vector<?>)get(rob_)).get(state.state_id) == ((Vector<?>)get(opp_)).get(state.state_id)) {
                    reward = (double)get(TAG_REWARD);
                } else {
                    reward = - (double) get(TAG_REWARD);
                }
            } else {
                reward = -1;
            }
            sum+= state.weight * reward;
        }

        return sum;
    }


//    @Override
//    public native void ComputeOptimalPolicyUsingVI(long reference);

//    public native void CopyDefaultPolicy(Vector<ValuedAction> policy);

//    @Override
//    public native Vector<ValuedAction> policy(long reference);

//    @Override
//    public native void ComputeBlindAlpha(long reference);

//    @Override
//    public native double ComputeActionValue(ParticleBelief belief, StateIndexer indexer, int action, long reference);

//    @Override
//    public native Vector<Vector<Double>> blind_alpha_(long reference);




    //region Helper Methods
    public AJAN_Agent getParameters(){
        System.out.println("Returning Agent Parameters");
        return this;
    }
    private int getFloorRows() {
        return 0;
    }

    public void PrintMethod() {System.out.println("Received a call");}
    private double LaserRange(AJAN_Agent_State state, int dir) {
        Coord rob = ((Floor)get(floor_)).GetCell((int)((Vector<?>)get(rob_)).get(state.state_id));
        Coord opp = ((Floor)get(floor_)).GetCell((int)((Vector<?>)get(opp_)).get(state.state_id));
        int d = 1;
        while(true){
            Coord coord = Add(rob,(Muliply(DIRECTIONS[dir], d)));
            if(Objects.equals(((Floor)get(floor_)).GetIndex(coord),-1) || Objects.equals(coord,opp)){
                break;
            }
            d++;
        }
        int x = DIRECTIONS[dir].x;
        int y = DIRECTIONS[dir].y;
        return d* sqrt(x*x+y*y);
    }

    private Vector<State> getTransitionProbabilities(AJAN_Agent_State state, int action) {
        return null;
    }
    private int SetReading(int observation,int reading, int dir) {
        observation &=  ~(((int)get(ONE)<< (int)get(BITS_PER_READING)-1)
                            << (dir *(int)get(BITS_PER_READING)));
        observation |= reading << (dir * (int)get(BITS_PER_READING));
        return observation;
    }

    private int RobOppIndicesToStateIndex(int rob, int opp) {
        return rob * ((Floor)get(floor_)).NumCells() + opp;
    }

    int TagAction() {
        return 4;
    }

//    void ComputeDefaultActions(String type) {
//        if(Objects.equals(type, "MDP")){
//        this.ComputeOptimalPolicyUsingVI(cppReference); // TODO: Ensure this calls the native method
//        this.CopyDefaultPolicy(policy_);
//        int num_states = NumStates();
//        ((Vector<?>)get(default_action)).ensureCapacity(NumStates());
//        for (int s = 0; s < NumStates(); s++) {
//            ((Vector<Integer>)get(default_action)).add(s,
//                    policy_.get(s).action);
//        }
//        } if (Objects.equals(type, "SP")) {
//            ((Vector<?>)get(default_action)).ensureCapacity(NumStates());
//            for (int s = 0; s < NumStates(); s++) {
//                ((Vector<Integer>)get(default_action)).add(s,0);
//                if(((Vector<?>)get(rob_)).get(s) == ((Vector<?>)get(opp_)).get(s)){
//                    ((Vector<Integer>)get(default_action)).add(s, TagAction());
//                } else {
//                    double cur_dist = ((Floor)get(floor_)).Distance(
//                            (int)((Vector<?>)get(rob_)).get(s),
//                            (int)((Vector<?>)get(opp_)).get(s)
//                    );
//                    for (int a = 0; a < 4; a++) {
//                        int next = NextRobPosition(
//                                (int)((Vector<?>)get(rob_)).get(s),
//                                (int)((Vector<?>)get(opp_)).get(s),
//                                a
//                        );
//
//                        double dist = ((Floor)get(floor_)).Distance(
//                                next,
//                                (int)((Vector<?>)get(opp_)).get(s)
//                        );
//                        if(dist<cur_dist){
//                            ((Vector<Integer>)get(opp_)).add(s,a);
//                            break;
//                        }
//                    }
//                }
//            }
//        } else {
//            System.out.println("Unsupported Default action type"+type);
//        }
//    }

    private int NextRobPosition(int rob, int opp, int a) {
//         System.out.println("NextRobPosition");
        Coord pos = Add(((Floor)get(floor_)).GetCell(rob) , DIRECTIONS[a]);
        if(a != TagAction() && ((Floor)get(floor_)).Inside(pos) && pos != ((Floor)get(floor_)).GetCell(opp))
            return ((Floor)get(floor_)).GetIndex(pos);
//         System.out.println("Sending rob position"+rob);
        return rob;
    }

    private void Init(String iss) {
        // region Base tag Init
        System.out.println(iss);
        ReadConfig(new Scanner(iss));
        AJAN_Agent_State state;
        ((Vector<?>) get(states_)).ensureCapacity(NumStates());
        ((Vector<?>) get(rob_)).ensureCapacity(NumStates());
        ((Vector<?>) get(opp_)).ensureCapacity(NumStates());
        for(int rob=0; rob < ((Floor)get(floor_)).NumCells(); rob++){
            for(int opp=0; opp < ((Floor)get(floor_)).NumCells(); opp++){
                int s = RobOppIndicesToStateIndex(rob, opp);
                state = new AJAN_Agent_State(s , 0);
                ((Vector<AJAN_Agent_State>)get(states_)).add(s, state);
                ((Vector<Integer>)get(rob_)).add(s, rob);
                ((Vector<Integer>)get(opp_)).add(s, opp);

            }
        }
        ((Vector<Vector<Vector<State>>>)get(transition_probabilities_)).ensureCapacity(NumStates());
        ((Vector<Vector<Vector<State>>>)get(transition_probabilities_)).setSize(NumStates());
        for (int s = 0; s < NumStates(); s++) {
//             System.out.println("Here it is");
            Vector<Vector<State>> stateVector = new Vector<>();
            stateVector.setSize(NumStates());
            ((Vector<Vector<Vector<State>>>)get(transition_probabilities_)).add(s,stateVector);
//            ((Vector<?>)((Vector<?>)get(transition_probabilities_)).get(s)).setSize(NumActions());
            Map<Integer, Double> opp_distribution = OppTransitionDistribution(s);
            for (int a = 0; a < NumActions(); a++) {
                Vector<State> actionVector = new Vector<>();
                actionVector.setSize(NumActions());
                actionVector.clear();
                ((Vector<Vector<Vector<State>>>)get(transition_probabilities_)).get(s).add(a,actionVector);
//                ((Vector<?>)((Vector<?>)((Vector<?>)get(transition_probabilities_)).get(s)).get(a)).clear();
//                 System.out.println("Fetching rob position");
                int next_rob = NextRobPosition(((Vector<Integer>)get(rob_)).get(s),
                        ((Vector<Integer>)get(opp_)).get(s), a);
                if(!(a == TagAction() && Coord.ManhattanDistance(
                        ((Floor)get(floor_)).GetCell((int)((Vector<?>)get(rob_)).get(s)),
                        ((Floor)get(floor_)).GetCell((int)((Vector<?>)get(opp_)).get(s)))
                        <=1 )) {
                    for (Map.Entry<Integer,Double> it : opp_distribution.entrySet()){
                            State next = new AJAN_Agent_State(0,0);
                            next.state_id = RobOppIndicesToStateIndex(next_rob, it.getKey().intValue());
                            next.weight = it.getValue();
                            ((Vector<State>)((Vector<?>)((Vector<?>)get(transition_probabilities_))
                                    .get(s))
                                    .get(a))
                                    .add(next);
                    }

                }
            }

        }
        //endregion
        //region Laser tag Init
        for (int i = 0; i < (int)get(NBEAMS); i++) {
            set(same_loc_obs_,SetReading((int)get(same_loc_obs_), 101, i));
        }
        ((Vector<Vector<Vector<Double>>>)get(reading_distributions_)).setSize(NumStates()); // std::vector<std::vector<std::vector<double> > >

        for(int s= 0; s < NumStates(); s++){
            int N_BEAMS = (int)get(NBEAMS);
            double unitSize = (double) get(unit_size);
            double noise_sigma = (double) get(noise_sigma_);
            Vector<Vector<Double>> stateVector = new Vector<>();
            stateVector.setSize(N_BEAMS);
            ((Vector<Vector<Vector<Double>>>)get(reading_distributions_)).add(s,stateVector);
            for (int d = 0; d < N_BEAMS; d++) {
                double dist = LaserRange(((AJAN_Agent_State)((Vector<?>)get(states_)).get(s)), d);
                Vector<Double> readingVector = new Vector<>();
                ((Vector<Vector<Vector<Double>>>)get(reading_distributions_)).get(s).add(d,readingVector);
                for (int reading = 0; reading < dist / unitSize; reading++) {

                    double min_noise = reading * unitSize - dist;
                    double max_noise = min(dist, (reading +1) * unitSize) - dist;
                    double prob = 2 * (gausscdf(max_noise, 0, noise_sigma)
                            -(reading >0
                            ? gausscdf(min_noise, 0, noise_sigma)
                            :0));
                    ((Vector<Double>)((Vector<?>)((Vector<?>)get(reading_distributions_)).get(s)).get(d)).add(prob);
                }
            }
        }
        //endregion
        System.out.println("Initialization Successful");
    }

    private Map<Integer, Double> OppTransitionDistribution(int state) {
        double temp;
//         System.out.println("Opposite Distribution");
        final Floor floor = (Floor)get(floor_);
        Coord rob = (floor.GetCell(
                ((int)((Vector<?>)get(rob_)).get(state))
        ));
        Coord opp = (floor.GetCell(
                ((int)((Vector<?>)get(opp_)).get(state))
        ));
        Map<Integer, Double> distribution = new HashMap<>();
        if(opp.x == rob.x){
            int index = floor.Inside(Add(opp, new Coord(1, 0)))
                    ? floor.GetIndex(Add(opp , new Coord(1, 0)))
                    : floor.GetIndex(opp);
            temp = distribution.get(index)!=null?distribution.get(index):0;
            distribution.put(index,  temp+0.2);
            index = (floor.Inside(Add(opp , new Coord(-1, 0)))
                    ? floor.GetIndex(Add(opp , new Coord(-1, 0)))
                    : floor.GetIndex(opp));
            temp = distribution.get(index)!=null?distribution.get(index):0;
            distribution.put(index,temp +0.2);
        } else {
            int dx = opp.x > rob.x ? 1 : -1;
            int index = floor.Inside(Add(opp ,new Coord(dx, 0)))
                    ? floor.GetIndex(Add(opp ,new Coord(dx, 0)))
                    : floor.GetIndex(opp);
            temp = distribution.get(index)!=null?distribution.get(index):0;
            distribution.put(index, temp + 0.4);
        }

        if (opp.y == rob.y){
            int index = floor.Inside(Add(opp , new Coord(0, 1)))
                    ? floor.GetIndex(Add(opp , new Coord(0, 1)))
                    : floor.GetIndex(opp);
            temp = distribution.get(index)!=null?distribution.get(index):0;
            distribution.put(index,temp +0.2);
            index = floor.Inside(Add(opp , new Coord(0, -1)))
                    ? floor.GetIndex(Add(opp , new Coord(0, -1)))
                    : floor.GetIndex(opp);
            temp = distribution.get(index)!=null?distribution.get(index):0;
            distribution.put(index,temp +0.2);
        } else {
            int dy = opp.y > rob.y ? 1 : -1;
            int index = floor.Inside(Add(opp , new Coord(0, dy)))
                    ? floor.GetIndex(Add(opp , new Coord(0, dy)))
                    : floor.GetIndex(opp);
            temp = distribution.get(index)!=null?distribution.get(index):0;
            distribution.put(index, temp + 0.4);
        }
        temp = distribution.get(floor.GetIndex(opp))!=null?distribution.get(floor.GetIndex(opp)):0;
        distribution.put(floor.GetIndex(opp), temp + 0.4);
        return distribution;
    }

    private void ReadConfig(Scanner iss) {
//        String line, key, val;
        Floor floor = null;
        while(iss.hasNext()){
            String key = iss.next();
            String val = iss.next();

            if(key.equals("mapSize")) {
                int nrows = iss.nextInt();
                int ncols = iss.nextInt();
                floor = new Floor(nrows, ncols);
                set(floor_,floor);

                for (int y = 0; y < nrows; y++) {
                    String line = iss.next();
                    for (int x = 0; x < ncols; x++) {
                        if(line.charAt(x) !='#') {
                            floor.AddCell(x, y);
                        }
                    }
                }
                floor.ComputeDistances();
            } else if (key.equals("width-height-obstacles")){
                int h = iss.nextInt();
                int w = iss.nextInt();
                int o = iss.nextInt();
                Scanner randomMapScanner = new Scanner(RandomMap(h, w, o));
                ReadConfig(randomMapScanner);
            } else {
                System.out.println("Map not perfect");
            }
        }
        assert floor != null;
        set(floor_,floor);
        System.out.println("Floor NumCells"+floor.NumCells());
    }

    private String BenchmarkMap() {
        int height =7; int width = 11; int obstacles = 8;
//        StringBuilder map = new StringBuilder(height * (width + 1) - 1); // ERROR: String index out of bound error
//        StringBuilder map = new StringBuilder(); // ERROR: String index out of bound error
        StringBuilder map = new StringBuilder(new String(new char[height*(width +1) -1]).replace('\0','.'));
        map.setLength(height * (width + 1) -1);
//        map.append('.');
        for (int h = 1; h < height; h++) {
            map.setCharAt(h * (width +1) -1, '\n');
        }

        int[] obstaclesList  = {1+2*(width+1), 3+4*(width+1), 3+0*(width+1), 5+0*(width+1), 6+4*(width+1), 9+4*(width+1), 9+1*(width+1), 10+6*(width+1)};
        for (int i = 0; i < obstacles; i++) {
            int p = obstaclesList[i];
            assert (map.charAt(p) != '\n' && map.charAt(p) != '#');
            map.setCharAt(p,'#');
        }
        return "mapSize = " + height + " " + width + "\n" + map.toString();
    }

    private String RandomMap(int height, int width, int obstacles) {
        StringBuilder map = new StringBuilder(height * (width + 1) - 1);
        map.append('.');
        for (int h = 1; h < height; h++)
            map.setCharAt(h * (width +1) -1, '\n');

        for (int i = 0; i < obstacles;) {
            Random random = new Random();
            int p = random.nextInt(map.length());
            if (map.charAt(p) != '\n' && map.charAt(p) != '#') {
                map.setCharAt(p, '#');
                i++;
            }
        }
        return "mapSize = " + height + " " + width+ "\n" + map.toString();
    }

    public Coord MostLikelyRobPosition(Vector<AJAN_Agent_State> particles) {
        Vector<Double> probs = new Vector<Double>(((Floor)get(floor_)).NumCells());
        double maxWeight = 0;
        int rob = -1;
        for (AJAN_Agent_State agentState : particles) {
            int id = ((Vector<Integer>) get(rob_)).get(agentState.state_id);
            probs.add(id, probs.get(id) + agentState.weight);
            if (probs.get(id) > maxWeight) {
                maxWeight = probs.get(id);
                rob = id;
            }
        }
        for (int i = 0; i < probs.size(); i++) {
            probs.add(i, 0.0);
        }
        return ((Floor)get(floor_)).GetCell(rob);
    }

    public Coord MostLikelyOppPosition(Vector<AJAN_Agent_State> particles) {
        Vector<Double> probs = new Vector<Double>(((Floor)get(floor_)).NumCells());
        double maxWeight = 0;
        int opp = -1;
        for (AJAN_Agent_State agentState : particles) {
            int id = ((Vector<Integer>) get(opp_)).get(agentState.state_id);
            probs.add(id, probs.get(id) + agentState.weight);
        }
        for (int i = 0; i < probs.size(); i++) {
            if (probs.get(i) > maxWeight) {
                maxWeight = probs.get(i);
                opp = i;
            }
            probs.add(i, 0.0);
        }
        return ((Floor)get(floor_)).GetCell(opp);
    }

    public Coord GetRobPos(AJAN_Agent_State state) {
        return ((Floor)get(floor_)).GetCell((int)((Vector<?>)get(rob_)).get(state.state_id));
    }

    public int StateIndexToRobIndex(int index) {
        return index/((Floor)get(floor_)).NumCells();
    }

    public int StateIndexToOppIndex(int index) {
        return index % ((Floor)get(floor_)).NumCells();
    }


    //endregion

    //    private native void InitializeObject(AJAN_Agent agent);
}
