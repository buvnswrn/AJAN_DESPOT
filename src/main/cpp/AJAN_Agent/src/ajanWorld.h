//
// Created by bhuvanesh on 15.06.23.
//
#include <despot/interface/world.h>
#include <jni.h>

#ifndef POMDP_AJANWORLD_H
#define POMDP_AJANWORLD_H

using namespace despot;

class ajanWorld: public World {
public:
    ajanWorld(JNIEnv * env, jobject* javaWorldObject);

    virtual State* Initialize();

    virtual State* GetCurrentState();

    virtual bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);

    virtual bool Connect();

    JNIEnv * javaEnv;
    jobject javaWorldObject;
};


#endif //POMDP_AJANWORLD_H
