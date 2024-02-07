//
// Created by bhuvanesh on 15.06.23.
//
#include <despot/interface/world.h>
#include <jni.h>
#ifndef POMDP_AJANWORLD_H
#define POMDP_AJANWORLD_H



using namespace despot;
namespace despot {
    class AJANWorld : public World {
    public:
        JNIEnv* javaEnv;
        jobject javaWorldObject;
        double noise_sigma_ = 0.5;

        AJANWorld();

        AJANWorld(JNIEnv * javaEnv, jobject * worldObject);

        virtual State *Initialize();

        virtual State *GetCurrentState();

        virtual bool ExecuteAction(ACT_TYPE action, OBS_TYPE &obs);

        virtual bool Connect();


    };
}

#endif //POMDP_AJANWORLD_H
