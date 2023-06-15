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
    return true; // for exit
    return false; // for continue
}

ajanWorld::ajanWorld(JNIEnv *env, jobject* javaWorldObject) {
    ajanWorld::javaEnv = env;
    ajanWorld::javaWorldObject = ajanWorld::javaEnv->NewGlobalRef(*javaWorldObject);
}
