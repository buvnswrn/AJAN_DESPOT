//
// Created by bhuvanesh on 25.05.23.
//

#include "ajan_agent.h"

#include <despot/util/coord.h>
#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

#include <jni.h>
using namespace std;
namespace despot {


    const ACT_TYPE AJANAgent::LEFT = 0;
    const ACT_TYPE AJANAgent::RIGHT = 1;
    const ACT_TYPE AJANAgent::HOVER = 2;
    const double AJANAgent::NOISE = 0.15;

    //region AJANAgentState Functions
    AJANAgentState::AJANAgentState() {
        // this method will fail
    }
    AJANAgentState::AJANAgentState(int _state_id):
//        state_id = _state_id;
        agent_position(_state_id){}
        // this method will fail

    AJANAgentState::AJANAgentState(JNIEnv * env, jobject stateObject) {
        AJANAgentState::javaEnv = env;
        AJANAgentState::javaStateObject = stateObject;
    }

    string AJANAgentState::text() const {
        // Do Java call here
//        return "s" + to_string(state_id);
        return agent_position == AJANAgent::LEFT ? "LEFT" : "RIGHT";
    }

    //endregion
    //region AJANAgent Functions
    AJANAgent::AJANAgent(){}
    AJANAgent::AJANAgent(JNIEnv * env,jobject* agentObject) {
        AJANAgent::javaEnv = env;
        AJANAgent::javaAgentObject = AJANAgent::javaEnv->NewGlobalRef(*agentObject);
        cout<<AJANAgent::javaEnv->GetVersion()<<endl;
        jclass javaClass = AJANAgent::javaEnv->GetObjectClass(javaAgentObject);
        jmethodID javaMethod = AJANAgent::javaEnv->GetMethodID(javaClass,"PrintMethod","()V");
        AJANAgent::javaEnv->CallVoidMethod(javaAgentObject,javaMethod);
    }
    bool AJANAgent::Step(despot::State &s, double random_num, despot::ACT_TYPE action, double &reward,
                         despot::OBS_TYPE &obs) const {
        AJANAgentState& ajanAgentState = static_cast<AJANAgentState&>(s);
        bool returnValue = AJANAgent::getAJANStep(ajanAgentState,random_num,action,reward,obs,
                    "Step","(Lcom/ajan/POMDP/implementation/AJAN_Agent_State;DIDI)Z");
        UpdateValues(ajanAgentState, reward, obs);
        return returnValue;
    }

    int AJANAgent::NumActions() const{
        return getAJANNum("NumActions", "()I");;
    }
    int AJANAgent::NumStates() const {
        return getAJANNum("NumStates","()I");
    }

    double AJANAgent::ObsProb(OBS_TYPE obs, const State &s, ACT_TYPE a) const {
        const AJANAgentState& state = static_cast<const AJANAgentState&>(s);
        return getAJANObsProb(obs,state,a);
    }

    State *AJANAgent::CreateStartState(std::string type) const {
        //TODO:Communicate with Java
        return new AJANAgentState(Random::RANDOM.NextInt(2));
    }

    Belief *AJANAgent::InitialBelief(const State *start, std::string type) const {
        // TODO: Replace this with Java Call
//        cout<<"Initial Belief"<<endl;
        vector<State*> particles;
        AJANAgentState* left = static_cast<AJANAgentState*>(Allocate(-1, 0.5));
        left->agent_position = LEFT;
        particles.push_back(left);
        AJANAgentState* right = static_cast<AJANAgentState*>(Allocate(-1, 0.5));
        right->agent_position = RIGHT;
        particles.push_back(right);
        return new ParticleBelief(particles, this);
    }

    double AJANAgent::GetMaxReward() const {
        return 0;
    }

    ValuedAction AJANAgent::GetBestAction() const {
        return ValuedAction(HOVER,-1);
    }

    ScenarioLowerBound *AJANAgent::CreateScenarioLowerBound(std::string name, std::string particle_bound_name) const {
        ScenarioLowerBound* bound = NULL;
        if (name == "TRIVIAL" || name == "DEFAULT") {
            bound = new TrivialParticleLowerBound(this);
        } else if (name == "RANDOM") {
            bound = new RandomPolicy(this,
                                     CreateParticleLowerBound(particle_bound_name));
        } else if (name == "LEFT") {
            bound = new BlindPolicy(this, AJANAgent::LEFT,
                                    CreateParticleLowerBound(particle_bound_name));
        } else if (name == "RIGHT") {
            bound = new BlindPolicy(this, AJANAgent::RIGHT,
                                    CreateParticleLowerBound(particle_bound_name));
        } else if (name == "LISTEN") {
            bound = new BlindPolicy(this, AJANAgent::HOVER,
                                    CreateParticleLowerBound(particle_bound_name));
        } else {
            cerr << "Unsupported scenario lower bound: " << name << endl;
            exit(1);
        }
        return bound;
    }

    //region Print Functions
    void AJANAgent::PrintState(const State &state, ostream &out) const {
//        jobject agentState = convertToAJANAgentState(state);
//        printInJava("PrintState","(Lcom/ajan/POMDP/implementation/AJAN_Agent_State;)V",
//                    agentState);
//        cout<<"Printing State"<<endl;
        const AJANAgentState& droneState = static_cast<const AJANAgentState&>(state);
        out << droneState.text() << endl;
    }

    void AJANAgent::PrintBelief(const Belief &belief, ostream &out) const {
//        cout<<"Printing Belief"<<endl;
    }

    void AJANAgent::PrintObs(const State &state, OBS_TYPE obs, ostream &out) const {
//        cout<<"Printing Obs"<<endl;
        out << (obs == LEFT ? "LEFT" : "RIGHT")<< endl;
        const AJANAgentState& droneState = static_cast<const AJANAgentState&>(state);
        jobject agentState = convertToAJANAgentState(droneState);
//        printInJava("PrintState","(Lcom/ajan/POMDP/implementation/AJAN_Agent_State;I)V",
//                    agentState,obs);
    }

    void AJANAgent::PrintAction(ACT_TYPE action, ostream &out) const {
//        cout<<"Printing Action"<<endl;
        if(action == LEFT){
            out << "Fly Left" << endl;
        } else if (action == RIGHT) {
            out << "Fly right" << endl;
        } else {
            out << "Hover" << endl;
        }
//        printInJava("PrintAction","(I)V",
//                    action);
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
//        cout << "Getting AJAN Step"<<endl;
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
//        cout<<"fetched the class type"<<endl;
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass, methodName, returnType);
        jobject state = convertToAJANAgentState(s);
        jboolean stepExecuted = javaEnv->CallBooleanMethod(javaAgentObject, javaMethod,
                                                           state, random_num,action,reward, obs);
//        cout << "Method Execute Complete"<<endl;
        return stepExecuted == JNI_TRUE;
    }
    int AJANAgent::getAJANNum(const char *methodName, const char *returnType) const {
        jclass javaClass = javaEnv ->GetObjectClass(javaAgentObject);
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass,methodName,returnType);
        return javaEnv->CallIntMethod(javaAgentObject,javaMethod);
    }
    double AJANAgent::getAJANObsProb(OBS_TYPE obs, const AJANAgentState &state, ACT_TYPE a) const {
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass,"ObsProb","(ILcom/ajan/POMDP/implementation/AJAN_Agent_State;I)D");
        jobject s = convertToAJANAgentState(state);
        jdouble obsProb = javaEnv->CallDoubleMethod(javaAgentObject,javaMethod,obs,s,a);
        return obsProb;
    }
    //endregion
    //region Helpers
    jobject AJANAgent::convertToAJANAgentState(const AJANAgentState &state) const {
//        std::cout<<"Converting AJAN State to State" <<std::endl;
//    int state = getAJANStateFromState();
        jclass ajanStateClass = javaEnv ->FindClass("com/ajan/POMDP/implementation/AJAN_Agent_State");
//        cout<<"      "<<endl;
        jobject ajanState = javaEnv->AllocObject(ajanStateClass);
//            cout<<"Getting Field ID:state_id"<<endl;
            jfieldID state_id = javaEnv->GetFieldID(ajanStateClass, "state_id","I");
//            cout<<"Getting Field ID:scenario_id"<<endl;
            jfieldID scenario_id = javaEnv->GetFieldID(ajanStateClass, "scenario_id","I");
//            cout<<"Getting Field ID:weight"<<endl;
            jfieldID weight = javaEnv->GetFieldID(ajanStateClass, "weight","D");
            jfieldID agent_position = javaEnv->GetFieldID(ajanStateClass, "agent_position", "I");
//            cout<<"Setting Field IDs"<<endl;
            javaEnv->SetIntField(ajanState, state_id, state.state_id);
            javaEnv->SetIntField(ajanState, scenario_id, state.scenario_id);
            javaEnv->SetDoubleField(ajanState, weight, state.weight);
            javaEnv->SetIntField(ajanState, agent_position, state.agent_position);
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
//        cout<<"Getting the class"<<endl;
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
//        cout<<"Getting currentState"<<endl;
        jfieldID currentStateID = javaEnv->GetStaticFieldID(javaClass, "currentState",
                                                            "Lcom/ajan/POMDP/implementation/AJAN_Agent_State;");
        jobject currentState = javaEnv->GetStaticObjectField(javaClass,currentStateID);
//        cout<<"Getting the currentReward"<<endl;
        jfieldID currentRewardID = javaEnv->GetStaticFieldID(javaClass, "currentReward", "D");
        jdouble currentReward = javaEnv->GetStaticDoubleField(javaClass, currentRewardID);
//        cout<<"Getting the currentObservation"<<endl;
        jfieldID currentObservationID = javaEnv->GetStaticFieldID(javaClass, "currentObservation", "I");
        jint currentObservation = javaEnv->GetStaticIntField(javaClass,currentObservationID);
//        cout<<"Typecasting them"<<endl;
//        jfieldID currentAction = javaEnv->GetStaticFieldID(javaClass, "currentAction", "I");
        reward = currentReward;
//        cout<<"Typecasting them"<<endl;
        obs = currentObservation;
        UpdateStateValues(state,currentState);
    }

    void AJANAgent::UpdateStateValues(AJANAgentState &ajanAgentState, jobject pJobject) const {
//        cout<<"Updating the state values";
        jclass javaClass = javaEnv->GetObjectClass(pJobject);
//        cout<<"getting agent_position";
        jfieldID ajanAgentStateID = javaEnv->GetFieldID(javaClass,"agent_position","I");
        jint ajanAgentPosition = javaEnv->GetIntField(pJobject,ajanAgentStateID);
//        cout<<"typecasting agent_position";
        ajanAgentState.agent_position = ajanAgentPosition;
//        cout<<"c_agent_position:"<<ajanAgentState.agent_position<<" ";
    }
    //endregion
    //endregion
};
