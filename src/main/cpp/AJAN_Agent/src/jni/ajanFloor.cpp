//
// Created by bhuvanesh on 03.07.23.
//
#include "de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor.h"
#include "despot/util/floor.h"
using namespace despot;
Floor floor;

JNIEXPORT jlong JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_Init__
        (JNIEnv * env, jobject thisObject) {
        floor = Floor();
    return reinterpret_cast<jlong>(&floor);
}

JNIEXPORT jlong JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_Init__II
        (JNIEnv * env, jobject thisObject, jint rows, jint cols) {
    floor = Floor();
    return reinterpret_cast<jlong>(&floor);
}

JNIEXPORT void JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_AddCell
        (JNIEnv * env, jobject thisObject, jint x, jint y, jlong) {
    floor.AddCell(x,y);
}