//
// Created by bhuvanesh on 30.06.23.
//
#include "de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_History.h"
#include "despot/core/history.h"


//JNIEXPORT void JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_History_Add
//        (JNIEnv * env, jobject thisObject, jint action, jint obs, jlong historyReference) {
//    despot::History history = *reinterpret_cast<despot::History*>(historyReference);
//    history.Add(action, obs);
//}
//
//JNIEXPORT void JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_History_RemoveLast
//        (JNIEnv * env, jobject thisObject, jlong historyReference) {
//    despot::History history = *reinterpret_cast<despot::History*>(historyReference);
//    history.RemoveLast();
//}
//JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_History_Action
//        (JNIEnv * env, jobject thisObject, jint t, jlong historyReference) {
//    despot::History history = *reinterpret_cast<despot::History*>(historyReference);
//    return history.Action(t);
//}
//
//JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_History_Observation
//        (JNIEnv * env, jobject thisObject, jint t, jlong historyReference) {
//    despot::History history = *reinterpret_cast<despot::History*>(historyReference);
//    return history.Observation(t);
//}

JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_History_Size
        (JNIEnv * env, jobject thisObject, jlong historyReference) {
    despot::History history = *reinterpret_cast<despot::History*>(historyReference);
    return history.Size();
}

//JNIEXPORT void JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_History_Truncate
//        (JNIEnv * env, jobject thisObject, jint t, jlong historyReference) {
//    despot::History history = *reinterpret_cast<despot::History*>(historyReference);
//    history.Truncate(t);
//}

JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_History_LastAction
        (JNIEnv * env, jobject thisObject, jlong historyReference) {
    despot::History history = *reinterpret_cast<despot::History*>(historyReference);
    return history.LastAction();
}

JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_History_LastObservation
        (JNIEnv * env, jobject thisObject, jlong historyReference) {
    despot::History history = *reinterpret_cast<despot::History*>(historyReference);
    return history.LastObservation();
}

//JNIEXPORT jobject JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_History_Suffix
//        (JNIEnv * env, jobject thisObject, jint t, jlong historyReference) {
//    despot::History history = *reinterpret_cast<despot::History*>(historyReference);
//    history = history.Suffix(t); //TODO: See how to return the created History
//}