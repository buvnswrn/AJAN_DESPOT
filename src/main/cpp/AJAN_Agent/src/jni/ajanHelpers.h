//
// Created by bhuvanesh on 24.06.23.
//
#include <jni.h>
#include <string>
#include <map>
#include <iostream>
#include <any>
#include <despot/interface/pomdp.h>

using namespace std;
#ifndef POMDP_AJANHELPERS_H
#define POMDP_AJANHELPERS_H

class ajanHelper;
//void Init(JNIEnv *&env);
     void Init(JNIEnv *&env, jobject *plannerObject, jobject *agentObject, jobject *worldObject);

     void GetAllMethodID();

     void GetAllPlannerMethodID();

     void GetAllAgentMethodID();

     void GetAllWorldMethodID();

     void GetAllVectorMethodID();

     void GetAllAjanPolicyMethodID();

     void GetAllParticleUpperBoundMethodID();

     jmethodID getMethodID(string clazz, string methodName);

     JNIEnv *getEnv();

     jclass getAjanStateClass();

     jclass getAjanPlannerClass();

     jclass getAjanAgentClass();

     jclass getAjanWorldClass();

     jclass getVectorClass();

     jclass getAjanFloorClass();

     jclass getAjanCoordClass();

     jclass getAjanParticleUpperBoundClass();

     jclass getAjanPolicyClass();

     jobject *getAjanPlannerObject();

     jobject *getAjanAgentObject();

     jobject *getAjanWorldObject();

     jobject getAjanParticleUpperBoundObject();

     jobject getAjanPolicyObject();

     void setAjanPolicyObject(jobject policyObject);

     void setAjanParticleUpperBoundObject(jobject particleUpperBound);

     void populateMethodIds(string methodNames[][2], int totalMethod, jclass classReference);

     jobject getJavaObject(JNIEnv * env, jclass clazz, jobject obj, const string &fieldID, const string &returnType, any value);

     any getJavaValue(JNIEnv * env, jclass clazz, jobject obj, const string &fieldID, const string &returnType);

     string getString(jobject result);

     jobject getAjanAgentStateVectorFromStateVector(const vector<despot::State *> &particles);

     string getSig(const string &method);
//};
#endif //POMDP_AJANHELPERS_H
