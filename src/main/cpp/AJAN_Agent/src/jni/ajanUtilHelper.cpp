//
// Created by bhuvanesh on 25.06.23.
//

#include "de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor.h"
#include "despot/util/floor.h"
#include "jniGlobals.h"
#include "ajanHelpers.h"

using namespace despot;

Floor floor_;

JNIEXPORT jlong JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_Init__
        (JNIEnv * env, jobject thisObject) {
    Init(env);
    floor_ = Floor();
    return (jlong) &floor_;
}

JNIEXPORT jlong JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_Init__II
        (JNIEnv * env, jobject thisObject, jint x, jint y) {
    floor_ = Floor(x, y);
    return (jlong) &floor_;
}

void JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_AddCell
        (JNIEnv * env, jobject thisObject, jint x, jint y, jlong reference) {
        auto* floor1_ = reinterpret_cast<Floor*>(reference);
        floor1_->AddCell(Coord(x,y));
}

JNIEXPORT jobject JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_GetCell
        (JNIEnv * env, jobject thisObject, jint i, jlong ptr){
    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    Coord coord = floor1_->GetCell(i);
    jobject jcoord = getEnv()->AllocObject(getAjanCoordClass());
    jcoord = getJavaObject(getAjanCoordClass(),jcoord,"x","I",coord.x);
    jcoord = getJavaObject(getAjanCoordClass(),jcoord,"y","I",coord.y);
    return jcoord;
}

JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_GetIndex
        (JNIEnv * env, jobject thisObject, jobject coordObject, jlong ptr) {
    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    int x = any_cast<int>(getJavaValue(getAjanCoordClass(), coordObject,"x" ,"I"));
    int y = any_cast<int>(getJavaValue(getAjanCoordClass(), coordObject,"y" ,"I"));
    return floor1_->GetIndex(x, y);
}

JNIEXPORT jboolean JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_Inside__Lde_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Coord_2J
        (JNIEnv * env, jobject thisObject, jobject coordObject, jlong ptr) {
    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    int x = any_cast<int>(getJavaValue(getAjanCoordClass(), coordObject,"x" ,"I"));
    int y = any_cast<int>(getJavaValue(getAjanCoordClass(), coordObject,"y" ,"I"));
    return floor1_->Inside(x, y);
}

JNIEXPORT jboolean JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_Inside__IIJ
        (JNIEnv *, jobject coordObject, jint x, jint y, jlong ptr) {
    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    return floor1_->Inside(x, y);
}

JNIEXPORT void JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_ComputeDistances
        (JNIEnv * env, jobject thisObject, jlong ptr) {
    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    floor1_->ComputeDistances();
}

JNIEXPORT jdouble JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_Distance
        (JNIEnv * env, jobject thisObject, jint c1, jint c2, jlong ptr) {
    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    return floor1_->Distance(c1, c2);
}

JNIEXPORT jobject JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_ComputeShortestPath
        (JNIEnv * env, jobject thisObject, jint c1, jint c2, jlong ptr) {
    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    // Convert to vector and send it back
    jobject jvector = env->NewObject(getVectorClass(),
                                            env->GetMethodID(vectorClass, "<init>", "()V"));
    vector<int> temp = floor1_->ComputeShortestPath(c1, c2);
    for (int i:temp) {
        jmethodID addMethod = env->GetMethodID(getVectorClass(),"add",("("+ getSig(OBJECT)+")Z").c_str());
        env->CallBooleanMethod(jvector,addMethod,i);
    }
    return jvector;
}

JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_num_1rows
        (JNIEnv * env, jobject thisObject, jlong ptr) {
    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    return floor1_->num_rows();
}

JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_num_1cols
        (JNIEnv * env, jobject thisObject, jlong ptr) {
    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    return floor1_->num_cols();
}

JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_NumCells
        (JNIEnv * env, jobject thisObject, jlong ptr) {
    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    return floor1_->NumCells();
}

JNIEXPORT void JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_destroyNativeClass
        (JNIEnv * env, jobject thisObject, jlong ptr) {
    delete reinterpret_cast<Floor*>(ptr);
}
