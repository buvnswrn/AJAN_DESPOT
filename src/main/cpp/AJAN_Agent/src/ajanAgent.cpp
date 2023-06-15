//
// Created by bhuvanesh on 25.05.23.
//

#include "ajan_agent.h"

#include <despot/util/coord.h>
#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/particle_belief.h>
#include <any>
#include <jni.h>
using namespace std;
namespace despot {

    std::map<std::string,std::any> variables;
    const ACT_TYPE AJANAgent::LEFT = 0;
    const ACT_TYPE AJANAgent::RIGHT = 1;
    const ACT_TYPE AJANAgent::HOVER = 2;
    const double AJANAgent::NOISE = 0.15;

    //region AJANAgentState Functions
    AJANAgentState::AJANAgentState() {
        // this method will fail
    }
    AJANAgentState::AJANAgentState(int _state_id):
        agent_position(_state_id){}

    AJANAgentState::AJANAgentState(JNIEnv * env, jobject stateObject) {
        AJANAgentState::javaEnv = env;
        AJANAgentState::javaStateObject = stateObject;
    }

    string AJANAgentState::text() const {
        return agent_position == AJANAgent::LEFT ? "LEFT" : "RIGHT";
    }

    //endregion
    //region AJANAgent Functions
    AJANAgent::AJANAgent(){}
    AJANAgent::AJANAgent(JNIEnv * env,jobject* agentObject) {
        AJANAgent::javaEnv = env;
        AJANAgent::javaAgentObject = AJANAgent::javaEnv->NewGlobalRef(*agentObject);
    }
    bool AJANAgent::Step(despot::State &s, double random_num, despot::ACT_TYPE action, double &reward,
                         despot::OBS_TYPE &obs) const {
        AJANAgentState& ajanAgentState = static_cast<AJANAgentState&>(s);
        bool returnValue = AJANAgent::getAJANStep(ajanAgentState,random_num,action,reward,obs,
                    "Step","(Lde/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/implementation/AJAN_Agent_State;DIDI)Z");
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
        return getAJANStartState(type);
    }

    Belief *AJANAgent::InitialBelief(const State *start, std::string type) const {
        return new ParticleBelief(getAJANParticles(start,type),this);
    }

    double AJANAgent::GetMaxReward() const {
        return getAJANMaxReward();
    }

    ValuedAction AJANAgent::GetBestAction() const {
        return getAJANBestAction();
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
        const AJANAgentState& droneState = static_cast<const AJANAgentState&>(state);
        out << droneState.text() << endl;
    }

    void AJANAgent::PrintBelief(const Belief &belief, ostream &out) const {
    }

    void AJANAgent::PrintObs(const State &state, OBS_TYPE obs, ostream &out) const {
        out << (obs == LEFT ? "LEFT" : "RIGHT")<< endl;
        const AJANAgentState& droneState = static_cast<const AJANAgentState&>(state);
        jobject agentState = convertToAJANAgentState(droneState);
//        printInJava("PrintState","(Lde/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/implementation/AJAN_Agent_State;I)V",
//                    agentState,obs);
    }

    void AJANAgent::PrintAction(ACT_TYPE action, ostream &out) const {
        if(action == LEFT){
            out << "Fly Left" << endl;
        } else if (action == RIGHT) {
            out << "Fly right" << endl;
        } else {
            out << "Hover" << endl;
        }
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
        ajanAgentState.agent_position = ajanAgentPosition;
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
        ajanAgentState->agent_position = javaEnv->GetIntField(valuedAction, agentPositionField);
        ajanAgentState->scenario_id = javaEnv->GetIntField(valuedAction, scenarioField);
        return ajanAgentState;
    }

    //endregion
    //endregion
};
