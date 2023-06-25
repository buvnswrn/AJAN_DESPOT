//
// Created by bhuvanesh on 24.06.23.
//
#include <jni.h>
#include <string>
#include <map>
#include <iostream>
#include <any>

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
static jclass stateClass;
static jclass coordClass;
static jclass floorClass;
//endregion

//region Map to store all the methodIDS
static map<string, jmethodID> plannerMethods;
static map<string, jmethodID> agentMethods;
static map<string, jmethodID> worldMethods;
static map<string, jmethodID> vectorMethods;
//endregion

void Init(JNIEnv *& env);
void Init(JNIEnv *& env, jobject* plannerObject, jobject * agentObject, jobject * worldObject);
void GetAllMethodID();
void GetAllPlannerMethodID();
void GetAllAgentMethodID();
void GetAllWorldMethodID();
void GetAllVectorMethodID();

jmethodID getMethodID(string clazz,string methodName);

JNIEnv * getEnv() { return ajanJavaEnv; }

jclass getAjanStateClass() { return stateClass; }
jclass getAjanPlannerClass(){ return plannerClass; }
jclass getAjanAgentClass() { return agentClass; }
jclass getAjanWorldClass() { return worldClass; }
jclass getVectorClass() {return vectorClass; }
jclass getAjanFloorClass() { return floorClass; }
jclass getAjanCoordClass() { return coordClass; }

jobject* getAjanPlannerObject() { return ajanJavaPlannerObject; }
jobject* getAjanAgentObject() { return ajanJavaAgentObject; }
jobject* getAjanWorldObject() { return ajanJavaWorldObject; }

void populateMethodIds(string methodNames[][2],int totalMethod, jclass classReference);
jobject getJavaObject (jclass clazz, jobject obj, const string& fieldID, const string& returnType, any value);
any getJavaValue (jclass clazz, jobject obj, const string& fieldID, const string& returnType);

#endif //POMDP_AJANHELPERS_H
