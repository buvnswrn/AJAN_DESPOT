//
// Created by bhuvanesh on 15.06.23.
//

#include "AJANWorld.h"
#include <jni.h>
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
        jclass javaClass = AJANWorld::javaEnv->GetObjectClass(AJANWorld::javaWorldObject);
        cout <<"Connecting to ROS ..." <<endl;
        jmethodID javaMethod = AJANWorld::javaEnv->GetMethodID(javaClass, "Connect", "()Z");
        jboolean connected = AJANWorld::javaEnv->CallBooleanMethod(javaWorldObject, javaMethod);
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
        jclass javaClass = javaEnv->GetObjectClass(javaWorldObject);
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass, "ExecuteAction", "(II)Z");
        jboolean connected = javaEnv->CallBooleanMethod(javaWorldObject, javaMethod, action, obs);
        javaMethod = javaEnv->GetMethodID(javaClass, "getCurrentObservation", "()I");
        jint currentObservation = javaEnv->CallIntMethod(javaWorldObject, javaMethod);
        obs = currentObservation;
        return connected == JNI_TRUE; // true for exit, false for continue
    }


}