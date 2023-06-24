//
// Created by bhuvanesh on 24.06.23.
//

#include "ajanHelpers.h"
#include "jniGlobals.h"
#include <jni.h>

using namespace std;

void Init(JNIEnv *& env, jobject* plannerObject, jobject * agentObject, jobject * worldObject){
    cout<<"Initializing the Java References"<<std::endl;
    ajanJavaEnv = env;
    ajanJavaPlannerObject = plannerObject;
    ajanJavaAgentObject = agentObject;
    ajanJavaWorldObject = worldObject;

    cout<<"Initializing the Java classes"<<std::endl;
    plannerClass = ajanJavaEnv->GetObjectClass(*plannerObject);
    agentClass = ajanJavaEnv->GetObjectClass(*agentObject);
    worldClass = ajanJavaEnv->GetObjectClass(*worldObject);

    vectorClass = ajanJavaEnv->FindClass(getSig(VECTOR).c_str());
    cout<<"Initializing the Java methods"<<std::endl;
    GetAllMethodID();
}

void GetAllMethodID() {
    GetAllPlannerMethodID();
    GetAllAgentMethodID();
    GetAllWorldMethodID();
    GetAllVectorMethodID();
}

void GetAllVectorMethodID() {
    cout<<"Initializing the Vector methods"<<std::endl;

    const int totalMethod = 2;
    string methodNames[totalMethod][2] = {
    {"size","()I",},
    {"get","(I)"+ getSig(OBJECT)}
    };
    for (auto & methodName : methodNames) {
        vectorMethods[methodName[0]] = (methodName[0],ajanJavaEnv->GetMethodID(vectorClass,
                                                                               methodName[0].c_str(),
                                                                               methodName[1].c_str()));
    }
    cout<<"Initialization of Vector methods Complete"<<std::endl;
}

void GetAllWorldMethodID() {
    cout<<"Initializing the World methods"<<std::endl;
    const int totalMethod = 5;
    string methodNames[totalMethod][2] = {
            {"Connect","()Z"},
            {"ExecuteAction","(II)Z",},
            {"getCurrentObservation","()I"},
            {"Initialize","()"+ getSig(STATE)},
            {"GetCurrentState","()" + getSig(STATE)}
    };

    for (auto & methodName : methodNames) {
        worldMethods[methodName[0]] = (methodName[0],ajanJavaEnv->GetMethodID(worldClass,
                                                                              methodName[0].c_str(),
                                                                              methodName[1].c_str()));
    }
    cout<<"Initialization of World methods Complete"<<std::endl;
}

void GetAllAgentMethodID() {
    cout<<"Initializing of Agent methods"<<std::endl;
    const int totalMethod = 20;
    string methodNames[totalMethod][2] = {
            {"Step","("+ getSig(AJAN_AGENT_STATE)+"DIDI)Z"},
            {"NumActions","()I"},
            {"NumStates","()I"},
            {"Reward","("+ getSig(STATE)+"I)D"},
            {"ObsProb","(I"+ getSig(AJAN_AGENT_STATE)+"I)D"},
            {"CreateStartState","("+ getSig(STRING)+")"+ getSig(AJAN_AGENT_STATE)},
            {"getInitialBeliefParticles","("+ getSig(STATE)+ getSig(STRING)+")"+ getSig(VECTOR)},
            {"GetMaxReward","()D"},
            {"GetBestAction","("+ getSig(ValuedAction)+")"},
            {"PrintState","("+ getSig(STATE)+")V"},
            {"PrintObs","("+ getSig(STATE)+"I)V"},
            {"PrintAction","(I)V"},
            {"PrintBelief","("+ getSig(BELIEF)+")V"},
            {"CreateScenarioLowerBound","("+ getSig(STRING)+ getSig(STRING)+")"+ getSig(STRING)},
            {"getParameters","()"+ getSig(AJAN_AGENT)},
            {"PrintMethod","()v"}
   };

    for (auto & methodName : methodNames) {
        agentMethods[methodName[0]] = (methodName[0],ajanJavaEnv->GetMethodID(agentClass,
                                                                              methodName[0].c_str(),
                                                                              methodName[1].c_str()));
    }
    cout<<"Initialization of Agent methods Complete"<<std::endl;
}

void GetAllPlannerMethodID() {
    cout<<"Initializing of Planner methods"<<std::endl;
    const int totalMethod = 5;
    string methodNames[totalMethod][2] = {
            {"InitializeModel","()Z"},
            {"InitializeWorld","("+ getSig(STRING)+")Z",},
            {"PrintMethod","()V"},
            {"ChooseSolver","()"+ getSig(STRING)},
            {"getWorldType","()" + getSig(STRING)}
    };
    for (auto & methodName : methodNames) {
        plannerMethods[methodName[0]] = (methodName[0],ajanJavaEnv->GetMethodID(plannerClass,
                                                                                methodName[0].c_str(),
                                                                                methodName[1].c_str()));
    }
    cout<<"Initialization of Planner methods Complete"<<std::endl;
}

JNIEnv * getEnv(){
    return ajanJavaEnv;
}

jobject* getAjanPlannerObject(){
    return ajanJavaPlannerObject;
}

jobject* getAjanAgentObject(){
    return ajanJavaAgentObject;
}

jobject* getAjanWorldObject(){
    return ajanJavaWorldObject;
}

jmethodID getMethodID(string clazz,string methodName){
    if(clazz == "Agent"){
        return agentMethods[methodName];
    } else if(clazz=="Planner"){
        return plannerMethods[methodName];
    } else if(clazz=="World"){
        return worldMethods[methodName];
    } else if(clazz == "Vector") {
        return vectorMethods[methodName];
    } else {
        cout<<"Cannot find the method"<<endl;
        return nullptr;
    }
}