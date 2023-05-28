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

    //region JNI Bridge
    JNIEnv* javaEnv;
    jobject javaAgentObject;
    jobject javaStateObject;

    JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
        JNIEnv* env;
        if(vm->GetEnv(reinterpret_cast<void**>(&env),JNI_VERSION_1_6)!=JNI_OK){
            std::cout<<"Initializing global"<<std::endl;
            return JNI_ERR;
        }
        std::cout<<"Initializing global Env variable"<<std::endl;
        javaEnv = env;
        return JNI_VERSION_1_6;
    }
    //endregion
    //region AJANAgentState Functions
    AJANAgentState::AJANAgentState() {
        // this method will fail
    }
    AJANAgentState::AJANAgentState(int _state_id) {
        state_id = _state_id;
        // this method will fail
    }

    AJANAgentState::AJANAgentState(jobject stateObject) {
        javaStateObject = stateObject;
    }

    string AJANAgentState::text() {
        // Do Java call here
        return "s" + to_string(state_id);
    }
    //endregion
    //region AJANAgent Functions
    AJANAgent::AJANAgent(){}
    AJANAgent::AJANAgent(JNIEnv* env,jobject agentObject) {
        javaEnv = env;
        javaAgentObject = agentObject;
        cout<<"Initialized AJAN Agent in DESPOT" <<endl;
    }
    bool AJANAgent::Step(despot::State &s, double random_num, despot::ACT_TYPE action, double &reward,
                         despot::OBS_TYPE &obs) const {
        bool terminal = false;
        //region Java Step call here
            bool returnValue = AJANAgent::getAJANStep(s,random_num,action,reward,obs,
                    "Step","(Lcom/ajan/POMDP/implementation/AJAN_Agent_State;DIDI)Z");
        // TODO: Get values of State, action, reward, obs from AJAN
            UpdateValues(s, reward, obs);
        //endregion
        return terminal;
    }

    int AJANAgent::NumActions() const{
        // TODO: Change the function to Communicate with Java
        return 10;
    }
    int AJANAgent::NumStates() const {
        return 2;
    }

    double AJANAgent::ObsProb(OBS_TYPE obs, const State &s, ACT_TYPE a) const {
        //TODO: Communicate With Java
        const AJANAgentState& state = static_cast<const AJANAgentState&>(s);

        if (a != HOVER)
            return obs == 2;

        return state.agent_position == obs ? (1 - NOISE) : NOISE;
    }

    State *AJANAgent::CreateStartState(std::string type) const {
        return DSPOMDP::CreateStartState(type);
    }

    Belief *AJANAgent::InitialBelief(const State *start, std::string type) const {
        // TODO: Replace this with Java Call
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
        return ValuedAction();
    }

    ScenarioLowerBound *AJANAgent::CreateScenarioLowerBound(std::string name, std::string particle_bound_name) const {
        ScenarioLowerBound* bound = NULL;
        if (name == "TRIVIAL" || name == "DEFAULT") {
            bound = new TrivialParticleLowerBound(this);
        } else if (name == "RANDOM") {
            bound = new RandomPolicy(this,
                                     CreateParticleLowerBound(particle_bound_name));
        } else if (name == "LEFT") {
            bound = new BlindPolicy(this, LEFT,
                                    CreateParticleLowerBound(particle_bound_name));
        } else if (name == "RIGHT") {
            bound = new BlindPolicy(this, RIGHT,
                                    CreateParticleLowerBound(particle_bound_name));
        } else if (name == "LISTEN") {
            bound = new BlindPolicy(this, HOVER,
                                    CreateParticleLowerBound(particle_bound_name));
        } else {
            cerr << "Unsupported scenario lower bound: " << name << endl;
            exit(1);
        }
        return bound;
    }

    //region Print Functions
    void AJANAgent::PrintState(const State &state, ostream &out) const {
        jobject agentState = convertToAJANAgentState(state);
        printInJava("PrintState","(Lcom/ajan/POMDP/implementation/AJAN_Agent_State;)V",
                    agentState);
    }

    void AJANAgent::PrintBelief(const Belief &belief, ostream &out) const {

    }

    void AJANAgent::PrintObs(const State &state, OBS_TYPE obs, ostream &out) const {
        jobject agentState = convertToAJANAgentState(state);
        printInJava("PrintState","(Lcom/ajan/POMDP/implementation/AJAN_Agent_State;I)V",
                    agentState,obs);
    }

    void AJANAgent::PrintAction(ACT_TYPE action, ostream &out) const {
        printInJava("PrintState","(I)V",
                    action);
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
    bool AJANAgent::getAJANStep(despot::State &s, double random_num, despot::ACT_TYPE action, double &reward,
                     despot::OBS_TYPE &obs,const char *methodName, const char *returnType) const{
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass, methodName, returnType);
        jobject state = convertToAJANAgentState(s);
        jboolean stepExecuted = javaEnv->CallBooleanMethod(javaAgentObject, javaMethod,
                                                           state, random_num,action,reward, obs);
        return stepExecuted == JNI_TRUE;
    }

    jobject AJANAgent::convertToAJANAgentState(const State &state) const {
//    int state = getAJANStateFromState();
            jclass ajanStateClass = javaEnv ->FindClass("com/ajan/POMDP/implementation/AJAN_Agent_State");
            jobject ajanState = javaEnv->AllocObject(ajanStateClass);
            jfieldID state_id = javaEnv->GetFieldID(ajanStateClass, "state_id","I");
            jfieldID scenario_id = javaEnv->GetFieldID(ajanStateClass, "scenario_id","I");
            jfieldID weight = javaEnv->GetFieldID(ajanStateClass, "weight","D");
            javaEnv->SetIntField(ajanState, state_id, state.state_id);
            javaEnv->SetIntField(ajanState, scenario_id, state.scenario_id);
            javaEnv->SetDoubleField(ajanState, weight, state.weight);
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

    void AJANAgent::UpdateValues(State &state, double &reward, OBS_TYPE &obs) const {
        jclass javaClass = javaEnv->GetObjectClass(javaAgentObject);
        jfieldID currentStateID = javaEnv->GetStaticFieldID(javaClass, "currentState",
                                                            "Lcom/ajan/POMDP/implementation/AJAN_Agent_State;");
        jobject currentState = javaEnv->GetStaticObjectField(javaClass,currentStateID);
        jfieldID currentRewardID = javaEnv->GetStaticFieldID(javaClass, "currentReward", "D");
        jdouble currentReward = javaEnv->GetStaticDoubleField(javaClass, currentRewardID);
        jfieldID currentObservationID = javaEnv->GetStaticFieldID(javaClass, "currentObservation", "I");
        jint currentObservation = javaEnv->GetIntField(javaClass,currentObservationID);
//        jfieldID currentAction = javaEnv->GetStaticFieldID(javaClass, "currentAction", "I");
        reward = static_cast<double>(currentReward);
        obs = static_cast<OBS_TYPE>(currentObservation);
        UpdateStateValues(state,currentState);
    }

    void AJANAgent::UpdateStateValues(State &s, jobject pJobject) const {
        jclass javaClass = javaEnv->GetObjectClass(pJobject);
        jfieldID ajanAgentStateID = javaEnv->GetFieldID(javaClass,"agent_position","I");
        jint ajanAgentPosition = javaEnv->GetIntField(pJobject,ajanAgentStateID);
        AJANAgentState& ajanAgentState = static_cast<AJANAgentState&>(s);
        ajanAgentState.agent_position = ajanAgentPosition;
    }


};
