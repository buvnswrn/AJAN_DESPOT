//
// Created by bhuvanesh on 15.06.23.
//

#include "ajanWorld.h"

bool ajanWorld::Connect() {
    // TODO:Communicate with Java
    jclass javaClass = javaEnv->GetObjectClass(javaWorldObject);
    jmethodID javaMethod = javaEnv->GetMethodID(javaClass,"Connect","()Z");
    jboolean connected = javaEnv->CallBooleanMethod(javaWorldObject,javaMethod);
    return connected == JNI_TRUE;
}

State* ajanWorld::Initialize() {
    return nullptr;
}

State* ajanWorld::GetCurrentState() {
    return nullptr;
}

bool ajanWorld::ExecuteAction(despot::ACT_TYPE action, despot::OBS_TYPE &obs) {
    //TODO:Communicate with java
    jclass javaClass = javaEnv->GetObjectClass(javaWorldObject);
    jmethodID  javaMethod = javaEnv->GetMethodID(javaClass, "ExecuteAction","(II)Z");
    jboolean  connected = javaEnv->CallBooleanMethod(javaWorldObject,javaMethod, action, obs);
    javaMethod = javaEnv->GetMethodID(javaClass,"getCurrentObservation","()I");
    jint currentObservation = javaEnv->CallIntMethod(javaWorldObject,javaMethod);
    obs = currentObservation;
    return connected == JNI_TRUE; // true for exit, false for continue
}

ajanWorld::ajanWorld(JNIEnv *env, jobject* javaWorldObject) {
    ajanWorld::javaEnv = env;
    ajanWorld::javaWorldObject = ajanWorld::javaEnv->NewGlobalRef(*javaWorldObject);
}
