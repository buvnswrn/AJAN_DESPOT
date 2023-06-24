//
// Created by bhuvanesh on 24.06.23.
//
#include <jni.h>
#include <string>
#include <map>
#include <iostream>

using namespace std;
#ifndef POMDP_AJANHELPERS_H
#define POMDP_AJANHELPERS_H
static JNIEnv* ajanJavaEnv;

//region AJAN Reference Objects
static jobject* ajanJavaPlannerObject;
static jobject* ajanJavaAgentObject;
static jobject* ajanJavaWorldObject;
//endregion

//region AJAN Reference Classes
static jclass plannerClass;
static jclass agentClass;
static jclass worldClass;
static jclass vectorClass;
//endregion

//region Map to store all the methodIDS
static map<string, jmethodID> plannerMethods;
static map<string, jmethodID> agentMethods;
static map<string, jmethodID> worldMethods;
static map<string, jmethodID> vectorMethods;
//endregion

void Init(JNIEnv *& env, jobject* plannerObject, jobject * agentObject, jobject * worldObject);
void GetAllMethodID();
void GetAllPlannerMethodID();
void GetAllAgentMethodID();
void GetAllWorldMethodID();
void GetAllVectorMethodID();

jmethodID getMethodID(string clazz,string methodName);

JNIEnv * getEnv();
jobject* getAjanPlannerObject();
jobject* getAjanAgentObject();
jobject* getAjanWorldObject();
jobject* getJavaVectorObject();
void populateMethodIds(string methodNames[][2],int totalMethod, jclass classReference);
#endif //POMDP_AJANHELPERS_H
