//
// Created by bhuvanesh on 15.06.23.
//

#include "AJANWorld.h"
#include <jni.h>
#include "jni/ajanHelpers.h"
namespace despot {
    AJANWorld::AJANWorld() {
        cout << "Initialising default world" << endl;
        state_ = nullptr;
    }

    AJANWorld::AJANWorld(JNIEnv * env,jobject * worldObject) {
        state_ = nullptr;
        AJANWorld::javaEnv = env;
        AJANWorld::javaWorldObject = AJANWorld::javaEnv->NewGlobalRef(*worldObject);
    }


    bool AJANWorld::Connect() {
        cout <<"Connecting to ROS ..." <<endl;
        jboolean connected = getEnv()->CallBooleanMethod(*getAjanWorldObject(),
                                                     getMethodID("World","Connect"));
        cout <<"Connected:"<< (connected == JNI_TRUE) <<endl;
        return connected == JNI_TRUE;
    }

    State *AJANWorld::Initialize() {
        return nullptr;
    }

    State *AJANWorld::GetCurrentState() {
        return nullptr;
    }

    bool AJANWorld::ExecuteAction(despot::ACT_TYPE action, despot::OBS_TYPE &obs) {
        cout<<"Executing action:"<<action<<endl;
        jboolean connected = getEnv()->CallBooleanMethod(*getAjanWorldObject(),
                                                     getMethodID("World","ExecuteAction"),
                                                                                                    action, obs);
        jint currentObservation = getEnv()->CallIntMethod(*getAjanWorldObject(),
                                                    getMethodID("World","getCurrentObservation"));
        obs = currentObservation;
        return connected == JNI_TRUE; // true for exit, false for continue
    }


}