//
// Created by bhuvanesh on 25.06.23.
//

#include "de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor.h"
#include "de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_AJAN_Util_Helper.h"
#include "despot/util/floor.h"
#include "ajanHelpers.h"

using namespace despot;

static Floor floor_;

JNIEXPORT jlong JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_Init__
        (JNIEnv * env, jobject thisObject) {
//    Init(env);
    floor_ = Floor();
    return (jlong) &floor_;
}

JNIEXPORT jlong JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_Init__II
        (JNIEnv * env, jobject thisObject, jint x, jint y) {
    floor_ = Floor(x, y);
//    cout<<"FloorCells:"<<floor_.NumCells()<<endl;
    return (jlong) &floor_;
}

void JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_AddCell
        (JNIEnv * env, jobject thisObject, jint x, jint y, jlong reference) {
//        Floor floor1_ = *reinterpret_cast<Floor*>(reference);
//        cout<<"FloorCells:"<<floor_.NumCells()<<endl;
    floor_.AddCell(Coord(x,y));
//    cout<<"FloorCellsAfter:"<<floor_.NumCells()<<endl;
}

JNIEXPORT jobject JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_GetCell
        (JNIEnv * env, jobject thisObject, jint i, jlong ptr){
//    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    Coord coord = floor_.GetCell(i);
    jclass s = env->FindClass("de/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/util/Coord");
    jobject jcoord = env->AllocObject(s);
//    cout<<"Object created"<<endl;
//    jobject jcoord = getEnv()->AllocObject(getAjanCoordClass());
    jcoord = getJavaObject(env, s,jcoord,"x","I",coord.x);
    jcoord = getJavaObject(env, s,jcoord,"y","I",coord.y);
    return jcoord;
}

JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_GetIndex
        (JNIEnv * env, jobject thisObject, jobject coordObject, jlong ptr) {
//    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    jclass coordObjectClass = env->GetObjectClass(coordObject);
    int x = any_cast<int>(getJavaValue(env, coordObjectClass, coordObject,"x" ,"I"));
    int y = any_cast<int>(getJavaValue(env, coordObjectClass, coordObject,"y" ,"I"));
    return floor_.GetIndex(x, y);
}

JNIEXPORT jboolean JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_Inside__Lde_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Coord_2J
        (JNIEnv * env, jobject thisObject, jobject coordObject, jlong ptr) {
//    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    jclass coordObjectClass = env->GetObjectClass(coordObject);
    int x = any_cast<int>(getJavaValue(env, coordObjectClass, coordObject,"x" ,"I"));
    int y = any_cast<int>(getJavaValue(env, coordObjectClass, coordObject,"y" ,"I"));
    return floor_.Inside(x, y);
}

JNIEXPORT jboolean JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_Inside__IIJ
        (JNIEnv *, jobject coordObject, jint x, jint y, jlong ptr) {
//    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    return floor_.Inside(x, y);
}

JNIEXPORT void JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_ComputeDistances
        (JNIEnv * env, jobject thisObject, jlong ptr) {
//    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    floor_.ComputeDistances();
}

JNIEXPORT jdouble JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_Distance
        (JNIEnv * env, jobject thisObject, jint c1, jint c2, jlong ptr) {
//    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    return floor_.Distance(c1, c2);
}

JNIEXPORT jobject JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_ComputeShortestPath
        (JNIEnv * env, jobject thisObject, jint c1, jint c2, jlong ptr) {
//    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    // Convert to vector and send it back
    jobject jvector = env->NewObject(getVectorClass(),
                                            env->GetMethodID(getVectorClass(), "<init>", "()V"));
    vector<int> temp = floor_.ComputeShortestPath(c1, c2);
    for (int i:temp) {
        jmethodID addMethod = env->GetMethodID(getVectorClass(),"add",("("+ getSig("java/lang/Object")+")Z").c_str());
        env->CallBooleanMethod(jvector,addMethod,i);
    }
    return jvector;
}

JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_num_1rows
        (JNIEnv * env, jobject thisObject, jlong ptr) {
//    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    return floor_.num_rows();
}

JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_num_1cols
        (JNIEnv * env, jobject thisObject, jlong ptr) {
//    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    return floor_.num_cols();
}

JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_NumCells
        (JNIEnv * env, jobject thisObject, jlong ptr) {
//    auto* floor1_ = reinterpret_cast<Floor*>(ptr);
    return floor_.NumCells();
}

JNIEXPORT void JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_Floor_destroyNativeClass
        (JNIEnv * env, jobject thisObject, jlong ptr) {
    delete reinterpret_cast<Floor*>(ptr);
}

JNIEXPORT jdouble JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_util_AJAN_1Util_1Helper_gausscdf
        (JNIEnv * env, jclass ajanUtilHelperClass, jdouble x, jdouble mean, jdouble sigma) {
    return gausscdf(x,mean,sigma);
}
