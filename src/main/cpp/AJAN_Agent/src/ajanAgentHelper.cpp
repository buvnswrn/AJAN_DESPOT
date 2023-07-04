//
// Created by bhuvanesh on 30.06.23.
//
#include "de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_implementation_AJAN_Agent.h"
#include "ajan_agent.h"
#include "jni/ajanHelpers.h"
#include "jni/jniGlobals.h"

using namespace despot;
JNIEXPORT void JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_implementation_AJAN_1Agent_ComputeOptimalPolicyUsingVI
(JNIEnv * env, jobject thisObject, jlong reference) {
    AJANAgent agent_model_ = *reinterpret_cast<AJANAgent*>(reference);
    agent_model_.ComputeOptimalPolicyUsingVI();
}

JNIEXPORT jobject JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_implementation_AJAN_1Agent_policy
        (JNIEnv * env, jobject thisObject, jlong reference) {
    AJANAgent agent_model_ = *reinterpret_cast<AJANAgent*>(reference);
    vector<ValuedAction> temp = agent_model_.policy();
    // Convert the policies to Java Vectors
    jobject jvector = env->NewObject(getVectorClass(),
                                     env->GetMethodID(getVectorClass(), "<init>", "()V"));
    for (ValuedAction i:temp) {
        jmethodID addMethod = env->GetMethodID(getVectorClass(),"add",("("+ getSig(OBJECT)+")Z").c_str());
        env->CallBooleanMethod(jvector,addMethod,i);
    }
    return thisObject;
}

JNIEXPORT void JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_implementation_AJAN_1Agent_ComputeBlindAlpha
        (JNIEnv * env, jobject thisObject, jlong reference) {
    AJANAgent agent_model_ = *reinterpret_cast<AJANAgent*>(reference);
    agent_model_.ComputeBlindAlpha();
}

JNIEXPORT jdouble JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_implementation_AJAN_1Agent_ComputeActionValue
        (JNIEnv * env, jobject thisObject, jobject belief, jobject indexer, jint action, jlong reference) {
    AJANAgent agent_model_ = *reinterpret_cast<AJANAgent*>(reference);
    auto* belief1 = reinterpret_cast<ParticleBelief*>(reference);
    return agent_model_.ComputeActionValue(belief1,agent_model_,action);
}

JNIEXPORT jobject JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_implementation_AJAN_1Agent_blind_1alpha_1
        (JNIEnv * env, jobject thisObject, jlong reference) {
    cout<<"WARNING: Method not working properly"<<endl;
    AJANAgent agent_model_ = *reinterpret_cast<AJANAgent*>(reference);
    vector<vector<int>> temp = (const vector<std::vector<int>> &) agent_model_.blind_alpha();
    jobject jvector = env->NewObject(getVectorClass(),
                                     env->GetMethodID(getVectorClass(), "<init>", "()V"));

    for(const vector<int>& vi:temp){
        jobject jvector1 = env->NewObject(getVectorClass(),
                                          env->GetMethodID(getVectorClass(), "<init>", "()V"));
        jmethodID addMethod = env->GetMethodID(getVectorClass(),"add",("("+ getSig(OBJECT)+")Z").c_str());
        for(int i:vi){
            env->CallBooleanMethod(jvector,addMethod,i);
        }
        env->CallBooleanMethod(jvector,addMethod,jvector1);
    }
    return jvector;
}