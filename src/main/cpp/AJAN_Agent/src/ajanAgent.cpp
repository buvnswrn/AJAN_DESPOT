//
// Created by bhuvanesh on 25.05.23.
//

#include "ajan_agent.h"
#include "AjanPolicy.h"
#include "AJANParticleUpperBound.h"
#include "AJANPOMCPPrior.h"

#include <despot/util/coord.h>
#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/particle_belief.h>
#include <any>
#include <jni.h>
#include <despot/solver/pomcp.h>

using namespace std;
namespace despot {

    std::map<std::string,std::any> variables;
    AJANAgent* AJANAgent::current_ = NULL;
    double AJANAgent::TAG_REWARD = 10;
    int AJANAgent::NBEAMS = 8;
    const uint64_t ONE = 1;
    int AJANAgent::BITS_PER_READING = 7;

    //region AJANAgentState Functions
    AJANAgentState::AJANAgentState() {
        // this method will fail
    }
    AJANAgentState::AJANAgentState(int _state_id){
        state_id = _state_id;
    }

    AJANAgentState::AJANAgentState(JNIEnv * env, jobject stateObject) {
        AJANAgentState::javaEnv = env;
        AJANAgentState::javaStateObject = stateObject;
    }

    string AJANAgentState::text() const {
//        return agent_position == AJANAgent::LEFT ? "LEFT" : "RIGHT";
    if(AJANAgent::current_ !=NULL){
        int rob = AJANAgent::current_->StateIndexToRobIndex(state_id);
        Coord rob_pos = AJANAgent::current_->floor_.GetCell(rob);
        int opp = AJANAgent::current_->StateIndexToOppIndex(state_id);
        Coord opp_pos = AJANAgent::current_->floor_.GetCell(opp);
        return "Rob at " + to_string(rob_pos) + ", Opp at "+ to_string(opp_pos);
    } else
        return to_string(state_id);
    }

    //endregion
    //region AJANAgent Functions
    AJANAgent::AJANAgent():
            noise_sigma_(0.5),
            unit_size(1.0),
            robot_pos_unknown_(false){
        //region base_tag
        current_ = this;
        istringstream iss(
                string("mapSize = 5 10\n") + string("#####...##\n")
                + string("#####...##\n") + string("#####...##\n")
                + string("...........\n") + string("..........."));
        Init(iss);
        istringstream iss1(BenchmarkMap());
        Init(iss1);
        //endregion
    }
    AJANAgent::AJANAgent(JNIEnv * env,jobject* agentObject):
        noise_sigma_(0.5),
        unit_size(1.0),
        robot_pos_unknown_(false) {
        current_=this;
        istringstream iss(BenchmarkMap());
        Init(iss);
        AJANAgent::javaEnv = env;
        AJANAgent::javaAgentObject = AJANAgent::javaEnv->NewGlobalRef(*agentObject);

    }
    bool AJANAgent::Step(despot::State &s, double random_num, despot::ACT_TYPE action, double &reward,
                         despot::OBS_TYPE &obs) const {
        AJANAgentState& ajanAgentState = static_cast<AJANAgentState&>(s);
        Random random(random_num);
        bool terminal = BaseStep(s, random.NextDouble(), action, reward);
        //region basetag call

        //endregion
        //region lasertag call
        if(terminal) {
            obs = same_loc_obs_;
        } else {
            if(rob_[ajanAgentState.state_id] == opp_[ajanAgentState.state_id])
                obs = same_loc_obs_;
            else {
                const vector<vector<double>>& laser_distribution = reading_distributions_[ajanAgentState.state_id];
                obs = 0;
                for (int dir = 0; dir < NBEAMS; dir++) {
                    double mass = random.NextDouble();
                    int reading = 0;
                    for (;  reading < laser_distribution[dir].size() ; reading++) {
                        mass -= laser_distribution[dir][reading];
                        if(mass < Globals::TINY)
                            break;
                    }
                    SetReading(obs, reading, dir);
                }
            }
        }
        //endregion
        return terminal;
        //region FIXME: Don't Ask Java for now
//        bool returnValue = AJANAgent::getAJANStep(ajanAgentState,random_num,action,reward,obs,
//                    "Step","(Lde/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/implementation/AJAN_Agent_State;DIDI)Z");
//        UpdateValues(ajanAgentState, reward, obs);
//        return returnValue;
        //endregion
    }

    int AJANAgent::NumActions() const{
        return 5;
        //FIXME: Don't make Java calls for now
//        return getAJANNum("NumActions", "()I");;
    }

    int AJANAgent::NumStates() const {
        return floor_.NumCells() * floor_.NumCells();
        // FIXME: Don't make Java calls for now
//        return getAJANNum("NumStates","()I");
    }

    double AJANAgent::ObsProb(OBS_TYPE obs, const State &s, ACT_TYPE a) const {
        const AJANAgentState& state = static_cast<const AJANAgentState&>(s);
        if (rob_[state.state_id] == opp_[state.state_id])
            return obs == same_loc_obs_;
        double prod = 1.0;
        for (int dir = 0; dir < NBEAMS; dir++) {
            int reading= GetReading(obs, dir);
            if(reading >= LaserRange(state, dir) / unit_size)
                return 0;
            double prob_mass = reading_distributions_[state.state_id][dir][reading];
            prod *= prob_mass;
        }
        return prod;
        //FIXME: Don't call java for now
//        return getAJANObsProb(obs,state,a);
    }

    State *AJANAgent::CreateStartState(std::string type) const {
        cout<<"CreateStartState"<<endl;
        int n = Random::RANDOM.NextInt(states_.size());
        cout<<"CreateStartStateEnd"<<endl;
        return new AJANAgentState(*states_[n]);
        //FIXME: Don't make Java calls for now
//        return getAJANStartState(type);
    }

    Belief *AJANAgent::InitialBelief(const State *start, std::string type) const {
        cout<<"Initializing Belief"<<endl;
        vector<State*> particles;
        int N = floor_.NumCells();
        double wgt = 1.0/ N /N;
        cout<<"N:"<<N<<",wgt:"<<wgt<<endl; //N is 0; Not sure why this is not initialized properly
        for (int rob = 0; rob < N; rob++) {
            for (int opp = 0; opp < N; opp++) {
                AJANAgentState* state = static_cast<AJANAgentState*>(Allocate(
                        RobOppIndicesToStateIndex(rob, opp), wgt));
                particles.push_back(state);
            }
        }
        ParticleBelief* belief = new ParticleBelief(particles, this);
        belief->state_indexer(this);
        cout<<"Initialized Belief"<<endl;
        return belief;
//        return new ParticleBelief(getAJANParticles(start,type),this);
    }

    double AJANAgent::GetMaxReward() const {
        return getAJANMaxReward();
    }

    double AJANAgent::Reward(int s, ACT_TYPE action) const {
        const AJANAgentState* state = states_[s];
        double reward = 0;
        if(action == TagAction()) {
            if(rob_[state->state_id] == opp_[state->state_id]) {
                reward = TAG_REWARD;
            } else {
                reward = -TAG_REWARD;
            }
        } else {
            reward = -1;
        }
        return reward;
    }

    ValuedAction AJANAgent::GetBestAction() const {
        return getAJANBestAction();
    }

    int AJANAgent::GetAction(const despot::State &state) const {
        return default_action_[GetIndex(&state)];
    }

    void AJANAgent::ComputeDefaultActions(string type) const {
//        if(type == "MDP") {
//// Consider this for MDP Implementation
////            const_cast<AJANAgent*>(this)->ComputeOptimalPolicyUsingVI();
//        int num_states = NumStates();
//        default_action_.resize(num_states);
//
//            for (int s = 0; s < num_states; s++) {
//                default_action_[s] = policy_[s].action;
//            }
//        }
        if(type == "SP") {
            default_action_.resize(NumStates());
            for (int s = 0; s < NumStates(); ++s) {
                default_action_[s] = 0;
                if(rob_[s] == opp_[s]){
                    default_action_[s] = TagAction();
                } else {
                    double cur_dist = floor_.Distance(rob_[s],opp_[s]);
                    for(int a = 0; a< 4; a++){
                        int next = NextRobPosition(rob_[s], opp_[s], a);
                        double dist = floor_.Distance(next, opp_[s]);
                        if(dist<cur_dist){
                            default_action_[s] = a;
                            break;
                        }
                    }
                }
            }
        } else {
            cerr<<" Unsupported default action type "<< type << endl;
            exit(1);
        }
    }

    ScenarioLowerBound *AJANAgent::CreateScenarioLowerBound(std::string name, std::string particle_bound_name) const {
        ScenarioLowerBound* bound = NULL;
//        if (name == "TRIVIAL" || name == "DEFAULT") {
//            bound = new TrivialParticleLowerBound(this);
//        } else if (name == "RANDOM") {
//            bound = new RandomPolicy(this,
//                                     CreateParticleLowerBound(particle_bound_name));
////        } else if (name == "LEFT") {
////            bound = new BlindPolicy(this, AJANAgent::LEFT,
////                                    CreateParticleLowerBound(particle_bound_name));
////        } else if (name == "RIGHT") {
////            bound = new BlindPolicy(this, AJANAgent::RIGHT,
////                                    CreateParticleLowerBound(particle_bound_name));
////        } else if (name == "LISTEN") {
////            bound = new BlindPolicy(this, AJANAgent::HOVER,
////                                    CreateParticleLowerBound(particle_bound_name));
//        } else {
//            cerr << "Unsupported scenario lower bound: " << name << endl;
//            exit(1);
//        }
        const DSPOMDP* model = this;
        const StateIndexer* indexer = this;
        const StatePolicy *policy = this;
        if(name == "DEFAULT" && same_loc_obs_ != floor_.NumCells()){
            return new AjanPolicy(model,
                                     CreateParticleLowerBound(particle_bound_name));
        } else if (name == "DEFAULT" && same_loc_obs_ == floor_.NumCells()){
            cout << "same_loc_obs_ == floor_.NumCells() need to implement MDP methods"<<endl;
            ComputeDefaultActions("SP"); // This type should be MDP but we are using SP for now
            return new ModeStatePolicy(model, *indexer, *policy,
                                       CreateParticleLowerBound(particle_bound_name));
        } else {
            if(name != "print")
                cerr << "Unsupported lower bound: " << name <<endl;
            cerr << "Supported types: DEFAULT"<< endl;
            cerr << "With base lower bound: except TRIVIAL " <<endl;
            exit(1);
            return NULL;
        }
        return bound;
    }
    ScenarioUpperBound *AJANAgent::CreateScenarioUpperBound(std::string name, std::string particle_bound_name) const {
        if(name =="TRIVIAL" || name == "DEFAULT"){
            return CreateParticleUpperBound(name);
        } else {
            if (name != "print")
                cerr << "Unsupported upper bound: " << name << endl;
            cerr << "Supported types: TRIVIAL, MDP, SP, MANHATTAN, LOOKAHEAD (default to SP)" << endl;
            cerr << "With base upper bound: LOOKAHEAD" << endl;
            exit(1);
            return nullptr;
        }
    }

    ParticleUpperBound *AJANAgent::CreateParticleUpperBound(std::string name) const {
        if(name =="SP" || name == "DEFAULT"){
            return new AJANParticleUpperBound(this);
        } else {
            if(name != "print")
                cerr << "Unsupported particle lower bound: " << name << endl;
            cerr << "Supported types: TRIVIAL, MDP, SP, MANHATTAN (default to SP)" << endl;
            exit(1);
            return nullptr;
        }
        return DSPOMDP::CreateParticleUpperBound(name);
    }



    //region Print Functions
    void AJANAgent::PrintState(const State &state, ostream &out) const {
        const AJANAgentState& droneState = static_cast<const AJANAgentState&>(state);
        out << droneState.text() << endl;
    }

    void AJANAgent::PrintBelief(const Belief &belief, ostream &out) const {
    }

    void AJANAgent::PrintObs(const State &state, OBS_TYPE obs, ostream &out) const {
//        out << (obs == LEFT ? "LEFT" : "RIGHT")<< endl;
//        const AJANAgentState& droneState = static_cast<const AJANAgentState&>(state);
//        jobject agentState = convertToAJANAgentState(droneState);
////        printInJava("PrintState","(Lde/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/implementation/AJAN_Agent_State;I)V",
//                    agentState,obs);
    }

    void AJANAgent::PrintAction(ACT_TYPE action, ostream &out) const {
//        if(action == LEFT){
//            out << "Fly Left" << endl;
//        } else if (action == RIGHT) {
//            out << "Fly right" << endl;
//        } else {
//            out << "Hover" << endl;
//        }
    }
    //endregion
    //region Memory Functions
    State *AJANAgent::Allocate(int state_id, double weight) const {
        AJANAgentState* particle = memory_pool_.Allocate();
        particle->state_id = state_id;
        particle->weight = weight;
        return particle;
    }

    State *AJANAgent::Copy(const State *particle) const {
        AJANAgentState* new_particle = memory_pool_.Allocate();
        *new_particle = *static_cast<const AJANAgentState*>(particle);
        new_particle->SetAllocated();
        return new_particle;
    }

    void AJANAgent::Free(State *particle) const {
        memory_pool_.Free(static_cast<AJANAgentState*>(particle));
    }

    int AJANAgent::NumActiveParticles() const {
        return memory_pool_.num_allocated();
    }
    //endregion
    //endregion
    //region Helper Functions
    // region AJAN-DESPOT Functions
    bool AJANAgent::getAJANStep(despot::AJANAgentState &s, double random_num, despot::ACT_TYPE action, double &reward,
                     despot::OBS_TYPE &obs,const char *methodName, const char *returnType) const{
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass, methodName, returnType);
        jobject state = convertToAJANAgentState(s);
        jboolean stepExecuted = javaEnv->CallBooleanMethod(javaAgentObject, javaMethod,
                                                           state, random_num,action,reward, obs);
        return stepExecuted == JNI_TRUE;
    }
    int AJANAgent::getAJANNum(const char *methodName, const char *returnType) const {
        jclass javaClass = javaEnv ->GetObjectClass(javaAgentObject);
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass,methodName,returnType);
        return javaEnv->CallIntMethod(javaAgentObject,javaMethod);
    }
    double AJANAgent::getAJANObsProb(OBS_TYPE obs, const AJANAgentState &state, ACT_TYPE a) const {
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass,"ObsProb","(ILde/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/implementation/AJAN_Agent_State;I)D");
        jobject s = convertToAJANAgentState(state);
        jdouble obsProb = javaEnv->CallDoubleMethod(javaAgentObject,javaMethod,obs,s,a);
        return obsProb;
    }
    double AJANAgent::getAJANMaxReward() const {
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass,"GetMaxReward","()D");
        jdouble obsProb = javaEnv->CallDoubleMethod(javaAgentObject,javaMethod);
        return obsProb;
    }
    ValuedAction AJANAgent::getAJANBestAction() const {
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass,"GetBestAction","()Lde/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/ValuedAction;");
        jobject valuedAction = javaEnv->CallObjectMethod(javaAgentObject,javaMethod,10);
        jclass valuedActionClass = javaEnv->GetObjectClass(valuedAction);
        jfieldID actionField = javaEnv->GetFieldID(valuedActionClass, "action", "I");
        jfieldID valueField = javaEnv->GetFieldID(valuedActionClass, "value", "D");
        jint action = javaEnv->GetIntField(valuedAction, actionField);
        jdouble value = javaEnv->GetDoubleField(valuedAction, valueField);
        return ValuedAction(action,value);
    }
    AJANAgentState *AJANAgent::getAJANStartState(std::string type) const {
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
        std::string ty = "testing";
        jstring typeString = javaEnv->NewStringUTF(ty.c_str());
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass,"CreateStartState",
                                        "(Ljava/lang/String;)Lde/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/implementation/AJAN_Agent_State;");
        jobject valuedAction = javaEnv->CallObjectMethod(javaAgentObject,javaMethod,typeString);
        AJANAgentState *ajanAgentState = getAgentStateFromAJANState(valuedAction, false);
        return ajanAgentState;
    }

    std::vector<State *> AJANAgent::getAJANParticles(const State *pState, std::string type) const {
        vector<State*> particles;
        std::string ty = "testing";
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
        jobject state = getAJANStateFromState(pState);
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass,"getInitialBeliefParticles","(Lde/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/State;Ljava/lang/String;)Ljava/util/Vector;");
        jobject particleVector = javaEnv->CallObjectMethod(javaAgentObject, javaMethod,state,state,javaEnv->NewStringUTF(ty.c_str()));
        cout<<"Fetched vector"<<endl;
        jclass javaVectorClass = javaEnv->GetObjectClass(particleVector);
        jmethodID javaVectorSizeMethod = javaEnv->GetMethodID (javaVectorClass, "size", "()I");
        jmethodID javaVectorGetMethod  = javaEnv->GetMethodID(javaVectorClass, "get",
                                                              "(I)Ljava/lang/Object;");
        jint size = javaEnv->CallIntMethod(particleVector,javaVectorSizeMethod);
        cout<<"Got "<<size<<" particles"<<endl;
        for (int i = 0; i < size; ++i) {
            jobject agentState = javaEnv->CallObjectMethod(particleVector,javaVectorGetMethod,i);
            cout<<"Fetching particle "<<i<<endl;
            particles.push_back(getAgentStateFromAJANState(agentState, true));
        }
        return particles;
    }
    //endregion
    //region Helpers
    jobject AJANAgent::convertToAJANAgentState(const AJANAgentState &state) const {
        jclass ajanStateClass = javaEnv ->FindClass("de/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/implementation/AJAN_Agent_State");
        jobject ajanState = javaEnv->AllocObject(ajanStateClass);
            jfieldID state_id = javaEnv->GetFieldID(ajanStateClass, "state_id","I");
            jfieldID scenario_id = javaEnv->GetFieldID(ajanStateClass, "scenario_id","I");
            jfieldID weight = javaEnv->GetFieldID(ajanStateClass, "weight","D");
            jfieldID agent_position = javaEnv->GetFieldID(ajanStateClass, "agent_position", "I");
            javaEnv->SetIntField(ajanState, state_id, state.state_id);
            javaEnv->SetIntField(ajanState, scenario_id, state.scenario_id);
            javaEnv->SetDoubleField(ajanState, weight, state.weight);
//            javaEnv->SetIntField(ajanState, agent_position, state.agent_position);
            return ajanState;
    }

    void AJANAgent::printInJava(const char *methodName,const char *returnType, jobject pJobject) const {
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass, methodName, returnType);
        javaEnv->CallVoidMethod(javaAgentObject, javaMethod, pJobject);
    }
    void AJANAgent::printInJava(const char *methodName,const char *returnType, jobject pJobject, const OBS_TYPE obs) const {
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass, methodName, returnType);
        javaEnv->CallVoidMethod(javaAgentObject, javaMethod, pJobject, obs);
    }
    void AJANAgent::printInJava(const char *methodName,const char *returnType, ACT_TYPE pJobject) const {
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass, methodName, returnType);
        javaEnv->CallVoidMethod(javaAgentObject, javaMethod, pJobject);
    }

    void AJANAgent::UpdateValues(AJANAgentState &state, double &reward, OBS_TYPE &obs) const {
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
        jfieldID currentStateID = javaEnv->GetStaticFieldID(javaClass, "currentState",
                                                            "Lde/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/implementation/AJAN_Agent_State;");
        jobject currentState = javaEnv->GetStaticObjectField(javaClass,currentStateID);
        jfieldID currentRewardID = javaEnv->GetStaticFieldID(javaClass, "currentReward", "D");
        jdouble currentReward = javaEnv->GetStaticDoubleField(javaClass, currentRewardID);
        jfieldID currentObservationID = javaEnv->GetStaticFieldID(javaClass, "currentObservation", "I");
        jint currentObservation = javaEnv->GetStaticIntField(javaClass,currentObservationID);
        reward = currentReward;
        obs = currentObservation;
        UpdateStateValues(state,currentState);
    }

    void AJANAgent::UpdateStateValues(AJANAgentState &ajanAgentState, jobject pJobject) const {
        jclass javaClass = javaEnv->GetObjectClass(pJobject);
        jfieldID ajanAgentStateID = javaEnv->GetFieldID(javaClass,"agent_position","I");
        jint ajanAgentPosition = javaEnv->GetIntField(pJobject,ajanAgentStateID);
//        ajanAgentState.agent_position = ajanAgentPosition;
    }
    jobject AJANAgent::getAJANStateFromState(const State *state) const{
        jclass ajanStateClass = javaEnv ->FindClass("de/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/implementation/AJAN_Agent_State");
        jobject ajanState = javaEnv->AllocObject(ajanStateClass);
        jfieldID state_id = javaEnv->GetFieldID(ajanStateClass, "state_id","I");
        jfieldID scenario_id = javaEnv->GetFieldID(ajanStateClass, "scenario_id","I");
        jfieldID weight = javaEnv->GetFieldID(ajanStateClass, "weight","D");
        javaEnv->SetIntField(ajanState, state_id, state->state_id);
        javaEnv->SetIntField(ajanState, scenario_id, state->scenario_id);
        javaEnv->SetDoubleField(ajanState, weight, state->weight);
        return ajanState;
    }
    AJANAgentState *AJANAgent::getAgentStateFromAJANState(jobject valuedAction, bool needAllocation) const {
        jclass valuedActionClass = javaEnv->GetObjectClass(valuedAction);
        jfieldID weightField = javaEnv->GetFieldID(valuedActionClass, "weight", "D");
        jfieldID stateField = javaEnv->GetFieldID(valuedActionClass, "state_id", "I");
        jfieldID scenarioField = javaEnv->GetFieldID(valuedActionClass, "scenario_id", "I");
        jfieldID agentPositionField = javaEnv->GetFieldID(valuedActionClass, "agent_position", "I");
        AJANAgentState *ajanAgentState;
        if (needAllocation){
            ajanAgentState = static_cast<AJANAgentState *>(Allocate(javaEnv->GetIntField(valuedAction, stateField),
                                                                     javaEnv->GetDoubleField(valuedAction,
                                                                                             weightField)));
        } else {
            ajanAgentState = new AJANAgentState();
            ajanAgentState->weight = javaEnv->GetDoubleField(valuedAction, weightField);
            ajanAgentState->state_id = javaEnv->GetIntField(valuedAction, stateField);
        }
//        ajanAgentState->agent_position = javaEnv->GetIntField(valuedAction, agentPositionField);
        ajanAgentState->scenario_id = javaEnv->GetIntField(valuedAction, scenarioField);
        return ajanAgentState;
    }

    void AJANAgent::Init(std::istream& is) {
        // region Base tag Init
        ReadConfig(is);
        AJANAgentState* state;
        states_.resize(NumStates()); // in Java, we can use vector.setSize()
        rob_.resize(NumStates());
        opp_.resize(NumStates());
        for(int rob=0; rob < floor_.NumCells(); rob++) {
            for (int opp = 0; opp < floor_.NumCells(); opp++) {
                int s = RobOppIndicesToStateIndex(rob, opp);
                state = new AJANAgentState(s);
                states_[s] = state;
                rob_[s] = rob;
                opp_[s] = opp;
            }
        }

        transition_probabilities_.resize(NumStates());
        for (int s = 0; s < NumStates(); ++s) {
            transition_probabilities_[s].resize(NumActions());
            const map<int, double>& opp_distribution = OppTransitionDistribution(s);
            for (int a = 0; a < NumActions(); ++a) {
                transition_probabilities_[s][a].clear();
                int next_rob = NextRobPosition(rob_[s], opp_[s], a);
                if(!(a == TagAction() && Coord::ManhattanDistance(floor_.GetCell(rob_[s]), floor_.GetCell(opp_[s]))<=1)){
                    for (map<int,double>::const_iterator it = opp_distribution.begin(); it != opp_distribution.end(); it++) {
                        State next;
                        next.state_id = RobOppIndicesToStateIndex(next_rob, it->first);
                        next.weight = it->second;
                        transition_probabilities_[s][a].push_back(next);
                    }
                }
            }
        }
        //endregion
        // region laser_tag Init
        for (int i = 0; i < NBEAMS; i++)
            SetReading(same_loc_obs_, 101, i);
        reading_distributions_.resize(NumStates());

        for (int s = 0; s < NumStates(); s++) {
            reading_distributions_[s].resize(NBEAMS);
            for (int d = 0; d < NBEAMS; d++) {
                double dist = LaserRange(*states_[s], d);
                for (int reading = 0; reading < dist / unit_size; reading++) {
                        double min_noise = reading * unit_size - dist;
                        double max_noise = min(dist, (reading +1) * unit_size) - dist;
                        double prob = 2 * (gausscdf(max_noise, 0, noise_sigma_)
                                -(reading >0
                                ? gausscdf(min_noise, 0, noise_sigma_)
                                :0));
                        reading_distributions_[s][d].push_back(prob);
                }
            }
        }

        //endregion
    }

    //region Init Methods
    double AJANAgent::LaserRange(const State& state, int dir) const {
        Coord rob = floor_.GetCell(rob_[state.state_id]), opp = floor_.GetCell(opp_[state.state_id]);
        int d =1;
        while(true){
            Coord coord = rob + Compass::DIRECTIONS[dir] * d;
            if(floor_.GetIndex(coord) == -1 || coord == opp)
                break;
            d++;
        }
        int x= Compass::DIRECTIONS[dir].x, y =Compass::DIRECTIONS[dir].y;

        return d* sqrt(x*x+y*y);
    }

    int AJANAgent::RobOppIndicesToStateIndex(int rob, int opp) const {
        return rob * floor_.NumCells() + opp;
    }

    map<int, double> AJANAgent::OppTransitionDistribution(int state) {
        Coord rob = floor_.GetCell(rob_[state]) , opp = floor_.GetCell(opp_[state]);
        map<int, double> distribution;
        if(opp.x == rob.x) {
            int index = floor_.Inside(opp+ Coord(1, 0))
                        ? floor_.GetIndex(opp + Coord(1, 0))
                        : floor_.GetIndex(opp);
            distribution[index] += 0.2;
            index = floor_.Inside(opp + Coord(-1, 0))
                    ? floor_.GetIndex(opp + Coord(-1, 0))
                    : floor_.GetIndex(opp);
            distribution[index] += 0.2;
        } else {
            int dx = opp.x > rob.x ? 1 : -1;
            int index = floor_.Inside(opp + Coord(dx, 0))
                        ? floor_.GetIndex(opp + Coord(dx, 0))
                        : floor_.GetIndex(opp);
            distribution[index] += 0.4;
        }
        if (opp.y == rob.y){
            int index = floor_.Inside(opp + Coord(0, 1))
                        ? floor_.GetIndex(opp + Coord(0, 1))
                        : floor_.GetIndex(opp);
            distribution[index] += 0.2;
            index = floor_.Inside(opp + Coord(0, -1))
                    ? floor_.GetIndex(opp + Coord(0, -1))
                    : floor_.GetIndex(opp);
            distribution[index] += 0.2;
        } else {
            int dy = opp.y > rob.y ? 1 : -1;
            int index = floor_.Inside(opp + Coord(0, dy))
                        ? floor_.GetIndex(opp + Coord(0, dy))
                        : floor_.GetIndex(opp);
            distribution[index] += 0.4;
        }
        distribution[floor_.GetIndex(opp)]+=0.2;
        ;        return distribution;
    }

    void AJANAgent::ReadConfig(std::istream& is) {
        string line, key, val;
        while(is >> key >> val) {
            if(key == "mapSize") {
                int nrows, ncols;
                is >> nrows >> ncols;
                floor_ = Floor(nrows, ncols);
                for (int y = 0; y < nrows; y++) {
                    is >> line;
                    for (int x = 0; x < ncols; x++) {
                        if(line[x] != '#') {
                            floor_.AddCell(Coord(x,y));
                        }
                    }
                }
                floor_.ComputeDistances();
            } else if (key == "width-height-obstacles") {
                int h, w, o;
                is >> h >> w >> o;
                istringstream iss(AJANAgent::RandomMap(h, w, o));
                ReadConfig(iss);
            } else {
                cout << "Map not perfect" << endl;
            }
        }
        cout<<"Floor Numcells"<<floor_.NumCells()<<endl;
    }
    string AJANAgent::RandomMap(int height, int width, int obstacles) {
        string map(height * (width + 1) - 1, '.');
        for (int h = 1; h < height; h++)
            map[h * (width + 1) - 1] = '\n';

        for (int i = 0; i < obstacles;) {
            int p = Random::RANDOM.NextInt(map.length());
            if (map[p] != '\n' && map[p] != '#') {
                map[p] = '#';
                i++;
            }
        }

        return "mapSize = " + to_string(height) + " " + to_string(width) + "\n" + to_string(map);
    }

    int AJANAgent::NextRobPosition(int rob, int opp, int a) const{
        Coord pos = floor_.GetCell(rob) + Compass::DIRECTIONS[a];
        if(a != TagAction() && floor_.Inside(pos) && pos != floor_.GetCell(opp))
            return floor_.GetIndex(pos);
        return rob;
    }

    int AJANAgent::TagAction() const {
        return 4;
    }
    string AJANAgent::BenchmarkMap() {
        int height=7; int width=11; int obstacles=8;
        string map(height * (width + 1) - 1, '.');
        for (int h = 1; h < height; h++)
            map[h * (width + 1) - 1] = '\n';

        int obstacles_list [] = {1+2*(width+1), 3+4*(width+1), 3+0*(width+1), 5+0*(width+1), 6+4*(width+1), 9+4*(width+1), 9+1*(width+1), 10+6*(width+1)};
        for (int i = 0; i < obstacles;) {
            int p = obstacles_list[i];
            assert (map[p] != '\n' && map[p] != '#');
            map[p] = '#';
            i++;
        }

        return "mapSize = " + to_string(height) + " " + to_string(width) + "\n" + to_string(map);
    }

    //endregion
    //region text helper methods
    int AJANAgent::StateIndexToRobIndex(int index) const {
        return index/ floor_.NumCells();
    }

    int AJANAgent::StateIndexToOppIndex(int index) const {
        return index % floor_.NumCells();
    }

    //endregion

    void AJANAgent::SetReading(uint64_t obs, uint64_t reading, uint64_t dir) const {
        // Clear bits
        obs &= ~(((ONE << BITS_PER_READING) - 1) << (dir * BITS_PER_READING));
        // Set bits
        obs |= reading << (dir * BITS_PER_READING);
    }

    int AJANAgent::GetReading(uint64_t obs, int dir) const {
        return (obs >>(dir * BITS_PER_READING)) & ((ONE << BITS_PER_READING) -1);
    }

    const Floor AJANAgent::floor() const {
        return floor_;
    }

    Coord AJANAgent::MostLikelyRobPosition(const vector<State *> &particles) const{
        static std::vector<double> probs = std::vector<double>(floor_.NumCells());
//        cout<<"MostLikelyRobPosition"<<endl;
        double maxWeight = 0;
        int rob = -1;
        for (int i = 0; i < particles.size(); i++) {
            AJANAgentState* agentState = static_cast<AJANAgentState*>(particles[i]);
            int id = rob_[agentState->state_id];
            probs[id] += agentState->weight;

            if(probs[id] > maxWeight) {
                maxWeight = probs[id];
                rob = id;
            }
        }

        for (int i = 0; i < probs.size(); i++) {
            probs[i] = 0.0;
        }
//        cout<<"MostLikelyRobPositionEnd"<<endl;
        return floor_.GetCell(rob);

    }

    Coord AJANAgent::MostLikelyOpponentPosition(const vector<State *> &particles) const {
//        cout<<"MostLikelyOppPosition"<<endl;
        vector<double> probs = vector<double>(floor_.NumCells());

        for (int i = 0; i < particles.size(); i++) {
            AJANAgentState* ajanAgentState = static_cast<AJANAgentState*>(particles[i]);
            probs[opp_[ajanAgentState->state_id]] += ajanAgentState->weight;
        }

        double maxWeight =0;
        int opp = -1;
        for (int i = 0; i < probs.size(); i++) {
            if(probs[i]> maxWeight) {
                maxWeight = probs[i];
                opp = i;
            }
            probs[i] = 0.0;
        }
//        cout<<"MostLikelyOppPositionEnd"<<endl;
        return floor_.GetCell(opp);
    }

    POMCPPrior *AJANAgent::CreatePOMCPPrior(std::string name) const {
        if(name=="UNIFORM") {
            return new UniformPOMCPPrior(this);
        } else if(name =="DEFAULT" || name=="SHR"){
            return new AJANPOMCPPrior(this);
        } else {
            cerr << "Unsupported POMCP prior: " << name << endl;
            exit(1);
            return nullptr;
        }
    }

    Coord AJANAgent::GetRobPos(const State *state) const {
        return floor_.GetCell(rob_[state->state_id]);
    }

    void AJANAgent::NoiseSigma(double noise_sigma) {
        noise_sigma_ =noise_sigma;
    }

    bool AJANAgent::BaseStep(State &s, double random_num, ACT_TYPE action, double &reward) const {
        AJANAgentState state = static_cast<AJANAgentState&>(s);
        bool terminal = false;
        if(action == TagAction()) {
            double distance = Coord::ManhattanDistance(floor_.GetCell(rob_[state.state_id]),floor_.GetCell(opp_[state.state_id]));
            if(distance <=1){
                reward = TAG_REWARD;
                terminal = true;
            } else {
                reward = - TAG_REWARD;
            }
        } else {
            reward = -1;
        }

        const vector<State>& distribution = transition_probabilities_[state.state_id][action];
        double sum = 0;
//        cout<<"distribution calculation"<<endl;
        for (int i = 0; i < distribution.size(); i++) {
            const State& next = distribution[i];
            sum+= next.weight;
            if(sum>=random_num){
                state.state_id = next.state_id;
                break;
            }
        }
        return terminal;
    }





    //endregion
    //endregion
};
