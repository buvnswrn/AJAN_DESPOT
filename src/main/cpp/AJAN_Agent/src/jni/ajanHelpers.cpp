//
// Created by bhuvanesh on 24.06.23.
//

#include "ajanHelpers.h"
#include "jniGlobals.h"
#include <jni.h>
#include <any>
#include <despot/interface/pomdp.h>

 void getJavaClassReferences();

 void GetAllParticleUpperBoundMethodID();

 void GetAllStateMethodID();

using namespace std;


//void Init(JNIEnv *& env) {
//    ajanJavaEnv = env;
//    getJavaClassReferences();
//}
    JNIEnv *ajanJavaEnv;

    JNIEnv *getEnv() { return ajanJavaEnv; }

//region Map to store all the methodIDS
    map<string, jmethodID> plannerMethods;
    map<string, jmethodID> agentMethods;
    map<string, jmethodID> stateMethods;
    map<string, jmethodID> particleUpperBoundMethods;
    map<string, jmethodID> ajanPolicyMethods;
    map<string, jmethodID> worldMethods;
    map<string, jmethodID> vectorMethods;
//endregion
//region AJAN Reference Objects
    jobject *ajanJavaPlannerObject;
    jobject *ajanJavaAgentObject;
    jobject *ajanJavaWorldObject;
    jobject ajanParticleUpperBound;
    jobject ajanPolicyObject;
//endregion

//region AJAN Reference Classes
    static jclass plannerClass;
    static jclass agentClass;
    static jclass worldClass;
    static jclass vectorClass;
    static jclass stateClass;
    static jclass particleUpperBoundClass;
    static jclass ajanPolicyClass;
    static jclass coordClass;
    static jclass floorClass;
//endregion

//region getters and setters
    jclass getAjanStateClass() { return stateClass; }

    jclass getAjanPlannerClass() { return plannerClass; }

    jclass getAjanAgentClass() { return agentClass; }

    jclass getAjanWorldClass() { return worldClass; }

    jclass getVectorClass() { return vectorClass; }

    jclass getAjanFloorClass() { return floorClass; }

    jclass getAjanCoordClass() { return coordClass; }

    jclass getAjanParticleUpperBoundClass() { return particleUpperBoundClass; }

    jclass getAjanPolicyClass() { return ajanPolicyClass; }

    jobject *getAjanPlannerObject() { return ajanJavaPlannerObject; }

    jobject *getAjanAgentObject() { return ajanJavaAgentObject; }

    jobject *getAjanWorldObject() { return ajanJavaWorldObject; }

    jobject getAjanParticleUpperBoundObject() { return ajanParticleUpperBound; }

    jobject getAjanPolicyObject() { return ajanPolicyObject; }

    void setAjanParticleUpperBoundObject(jobject particleUpperBound) {
        ajanParticleUpperBound = getEnv()->NewGlobalRef(particleUpperBound);
    }

    void setAjanPolicyObject(jobject policyObject) {
        ajanPolicyObject = getEnv()->NewGlobalRef(policyObject);
    }

    string getSig(const string &method) {
        return ("L" + method + ";");
    }

//endregion
    void Init(JNIEnv *&env, jobject *plannerObject, jobject *agentObject, jobject *worldObject) {
        cout << "Initializing the Java References" << std::endl;
        ajanJavaEnv = env;
        ajanJavaPlannerObject = plannerObject;
        ajanJavaAgentObject = agentObject;
        ajanJavaWorldObject = worldObject;

        cout << "Initializing the Java classes" << std::endl;
        plannerClass = ajanJavaEnv->GetObjectClass(*plannerObject);
        agentClass = ajanJavaEnv->GetObjectClass(*agentObject);
        worldClass = ajanJavaEnv->GetObjectClass(*worldObject);

        getJavaClassReferences();
    }

void getJavaClassReferences() {
    vectorClass = ajanJavaEnv->FindClass(getSig(VECTOR).c_str());
    stateClass = ajanJavaEnv->FindClass(getSig(AJAN_AGENT_STATE).c_str());
    coordClass = ajanJavaEnv->FindClass(getSig(COORD).c_str());
    floorClass = ajanJavaEnv->FindClass(getSig(FLOOR).c_str());
    ajanPolicyClass = ajanJavaEnv->FindClass(getSig(AJAN_POLICY).c_str());
    particleUpperBoundClass = ajanJavaEnv->FindClass(getSig(AJAN_PARTICLE_UPPER_BOUND).c_str());
    cout << "Initializing the Java methods" << endl;
    GetAllMethodID();
}

    void GetAllMethodID() {
        GetAllPlannerMethodID();
        GetAllAgentMethodID();
        GetAllStateMethodID();
        GetAllWorldMethodID();
        GetAllVectorMethodID();
        GetAllParticleUpperBoundMethodID();
        GetAllAjanPolicyMethodID();
    }

    void GetAllStateMethodID() {
        cout << "Initializing the State methods" << std::endl;
        const int totalMethod = 1;
        string methodNames[totalMethod][2] = {
                {"text", "(ID)" + getSig(STRING)}
        };
        for (auto &methodName: methodNames) {
            stateMethods[methodName[0]] = (methodName[0], ajanJavaEnv->GetMethodID(vectorClass,
                                                                                   methodName[0].c_str(),
                                                                                   methodName[1].c_str()));
        }
        cout << "Initialization of State methods Complete" << std::endl;
    }

    void GetAllParticleUpperBoundMethodID() {
        cout << "Initializing the ParticleUpperBound methods" << std::endl;
        const int totalMethod = 1;
        string methodNames[totalMethod][2] = {
                {"Value", "(I)D"}
        };
        for (auto &methodName: methodNames) {
            particleUpperBoundMethods[methodName[0]] = (methodName[0], ajanJavaEnv->GetMethodID(particleUpperBoundClass,
                                                                                                methodName[0].c_str(),
                                                                                                methodName[1].c_str()));
        }
        cout << "Initialization of ParticleUpperBound methods Complete" << std::endl;
    }

    void GetAllAjanPolicyMethodID() {
        cout << "Initializing the AJAN Policy methods" << std::endl;
        const int totalMethod = 2;
        string methodNames1[totalMethod][2] = {
                {"Action", "(" + getSig(VECTOR) + "J)I",},
                {"TestMethod", "()I"}
        };

        for (auto &methodName: methodNames1) {
            ajanPolicyMethods[methodName[0]] = (methodName[0], ajanJavaEnv->GetMethodID(ajanPolicyClass,
                                                                                        methodName[0].c_str(),
                                                                                        methodName[1].c_str()));
        }
        cout << "Initialization of AJAN Policy methods Complete" << std::endl;
    }

    void GetAllVectorMethodID() {
        cout << "Initializing the Vector methods" << std::endl;

        const int totalMethod = 4;
        string methodNames[totalMethod][2] = {
                {"size",   "()I",},
                {"<init>", "()V",},
                {"get",    "(I)" + getSig(OBJECT)},
                {"add",    "(" + getSig(OBJECT) + ")Z"}
        };
        for (auto &methodName: methodNames) {
            vectorMethods[methodName[0]] = (methodName[0], ajanJavaEnv->GetMethodID(vectorClass,
                                                                                    methodName[0].c_str(),
                                                                                    methodName[1].c_str()));
        }
        cout << "Initialization of Vector methods Complete" << std::endl;
    }


    void GetAllWorldMethodID() {
        cout << "Initializing the World methods" << std::endl;
        const int totalMethod = 5;
        string methodNames[totalMethod][2] = {
                {"Connect",               "()Z"},
                {"ExecuteAction",         "(II)Z",},
                {"getCurrentObservation", "()I"},
                {"Initialize",            "()" + getSig(STATE)},
                {"GetCurrentState",       "()" + getSig(STATE)}
        };

        for (auto &methodName: methodNames) {
            worldMethods[methodName[0]] = (methodName[0], ajanJavaEnv->GetMethodID(worldClass,
                                                                                   methodName[0].c_str(),
                                                                                   methodName[1].c_str()));
        }
        cout << "Initialization of World methods Complete" << std::endl;
    }

    void GetAllAgentMethodID() {
        cout << "Initializing of Agent methods" << std::endl;
        const int totalMethod = 30;
        string methodNames[totalMethod][2] = {
                {"Step",                      "(" + getSig(AJAN_AGENT_STATE) + "DIDI)Z"},
                {"NumActions",                "()I"},
                {"NumStates",                 "()I"},
                {"ObsProb",                   "(I" + getSig(AJAN_AGENT_STATE) + "I)D"},
                {"CreateStartState",          "(" + getSig(STRING) + ")" + getSig(AJAN_AGENT_STATE)},
                {"getInitialBeliefParticles", "(" + getSig(STATE) + getSig(STRING) + ")" + getSig(VECTOR)},
                {"GetMaxReward",              "()D"},
//                {"Reward","("+ getSig(STATE)+"I)D"},
                {"Reward",                    "(II)D"},
                {"GetBestAction",             "()" + getSig(VALUED_ACTION)},
                {"GetAction",                 "(" + getSig(STATE) + ")I"},
                {"CreateScenarioLowerBound",  "(" + getSig(STRING) + getSig(STRING) + ")" + getSig(AJAN_POLICY)},
                {"CreateScenarioUpperBound",  "("+ getSig(STRING)+ getSig(STRING)+")"+ getSig(STRING)},
                {"CreateParticleUpperBound",  "("+ getSig(STRING)+")"+ getSig(AJAN_PARTICLE_UPPER_BOUND)},
                {"CreateParticleLowerBound",  "("+ getSig(STRING)+")"+ getSig(STRING)},
                {"TransitionProbability",     "(II)" + getSig(VECTOR)},
                {"GetIndex",                  "("+ getSig(STATE)+")I"},
                {"GetState",                  "(I)"+ getSig(STATE)},
                {"PrintState",                "(" + getSig(STATE) + ")V"},
                {"PrintObs",                  "(" + getSig(STATE) + "I)V"},
                {"PrintAction",               "(I)V"},
                {"PrintBelief",               "(" + getSig(BELIEF) + ")V"},
                {"GetMMAP",                   "(" + getSig(VECTOR) + ")" + getSig(AJAN_AGENT_STATE)},
                {"getTauParticles",           "(" + getSig(VECTOR) + "II)" + getSig(VECTOR)},
                {"Observe",             "()" + getSig(AJAN_AGENT)}, // see what is it for map
                {"StepRewardFromParticles",             "("+ getSig(VECTOR)+"I)D"},
//            {"Observe","("+ getSig()+"II)"+ getSig()},
                {"StepRewardFromParticles",   "(" + getSig(VECTOR) + "I)D"},
                {"PrintMethod",               "()V"},

        };

        for (auto &methodName: methodNames) {
            agentMethods[methodName[0]] = (methodName[0], ajanJavaEnv->GetMethodID(agentClass,
                                                                                   methodName[0].c_str(),
                                                                                   methodName[1].c_str()));
        }
        cout << "Initialization of Agent methods Complete" << std::endl;
    }

    void GetAllPlannerMethodID() {
        cout << "Initializing of Planner methods" << std::endl;
        const int totalMethod = 5;
        string methodNames[totalMethod][2] = {
                {"InitializeModel", "()Z"},
                {"InitializeWorld", "(" + getSig(STRING) + ")Z",},
                {"PrintMethod",     "()V"},
                {"ChooseSolver",    "()" + getSig(STRING)},
                {"getWorldType",    "()" + getSig(STRING)}
        };
        for (auto &methodName: methodNames) {
            plannerMethods[methodName[0]] = (methodName[0], ajanJavaEnv->GetMethodID(plannerClass,
                                                                                     methodName[0].c_str(),
                                                                                     methodName[1].c_str()));
        }
        cout << "Initialization of Planner methods Complete" << std::endl;
    }

    jmethodID getMethodID(string clazz, string methodName) {
//    cout<<"Calling "<<methodName<<" for "<<clazz<<endl;
        if (clazz == "Agent") {
            return agentMethods[methodName];
        } else if (clazz == "Planner") {
            return plannerMethods[methodName];
        } else if (clazz == "World") {
            return worldMethods[methodName];
        } else if (clazz == "Vector") {
            return vectorMethods[methodName];
        } else if (clazz == "ParticleUpperBound") {
            return particleUpperBoundMethods[methodName];
        } else if (clazz == "Policy") {
            return ajanPolicyMethods[methodName];
        } else if (clazz == "State") {
            return stateMethods[methodName];
        } else {
            cout << "Cannot find the method" << endl;
            return nullptr;
        }
    }

    jobject getJavaObject(JNIEnv * env, jclass clazz, jobject obj, const string &fieldID, const string &returnType, any value) {
        jfieldID javaFieldID = env->GetFieldID(clazz, fieldID.c_str(), returnType.c_str());
        if (returnType == "I") {
            env->SetIntField(obj, javaFieldID, any_cast<int>(value));
        } else if (returnType == "D") {
            env->SetDoubleField(obj, javaFieldID, any_cast<double>(value));
        } else if (returnType == "F") {
            env->SetFloatField(obj, javaFieldID, any_cast<float>(value));
        } else if (returnType == "J") {
            env->SetLongField(obj, javaFieldID, any_cast<long>(value));
        } else if (returnType == "Z") {
            env->SetBooleanField(obj, javaFieldID, any_cast<bool>(value));
        } else {
            cout << "Cannot set variable value" << endl;
        }
        return obj;
    }

    any getJavaValue(JNIEnv * env, jclass clazz, jobject obj, const string &fieldID, const string &returnType) {
        jfieldID javaFieldID = env->GetFieldID(clazz, fieldID.c_str(), returnType.c_str());
        if (returnType == "I") {
            return env->GetIntField(obj, javaFieldID);
        } else if (returnType == "D") {
            return env->GetDoubleField(obj, javaFieldID);
        } else if (returnType == "F") {
            return env->GetFloatField(obj, javaFieldID);
        } else if (returnType == "J") {
            return env->GetLongField(obj, javaFieldID);
        } else if (returnType == "Z") {
            return env->GetBooleanField(obj, javaFieldID);
        } else {
            cout << "Cannot set variable value" << endl;
        }
        return obj;
    }

    jobject getAJANStateFromState(const despot::State *state) {
        jclass ajanStateClass = getAjanStateClass();
        jobject ajanState = getEnv()->AllocObject(ajanStateClass);
        jfieldID state_id = getEnv()->GetFieldID(ajanStateClass, "state_id", "I");
        jfieldID scenario_id = getEnv()->GetFieldID(ajanStateClass, "scenario_id", "I");
        jfieldID weight = getEnv()->GetFieldID(ajanStateClass, "weight", "D");
        getEnv()->SetIntField(ajanState, state_id, state->state_id);
        getEnv()->SetIntField(ajanState, scenario_id, state->scenario_id);
        getEnv()->SetDoubleField(ajanState, weight, state->weight);
//        cout<<"getAJANStateFromState:Returning"<<endl;
        return ajanState;
    }

    jobject getAjanAgentStateVectorFromStateVector(const vector<despot::State *> &particles) {
//        jobject vectorObject = getEnv()->NewObject(getVectorClass(), getMethodID("Vector", "init"));
        jobject vectorObject = getEnv()->AllocObject(getVectorClass());
        for (int i = 0; i < particles.size(); i++) {
            jobject ajanStateObject = getAJANStateFromState(particles[i]);
            getEnv()->CallBooleanMethod(vectorObject, getMethodID("Vector", "add"), ajanStateObject);
        }
        return vectorObject;
    }

    string getString(jobject result) {
        auto solverName = (jstring) result;
        return getEnv()->GetStringUTFChars(solverName, nullptr);
    }

