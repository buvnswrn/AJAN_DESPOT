#include <jni.h>
#include <iostream>
#include "com_jni_example_TemperatureSampler.h"
#include "TigerState.h"

JNIEnv* globalEnv;
jobject thisClassObject;
typedef int ACT_TYPE;
typedef uint64_t OBS_TYPE;
JNIEXPORT jfloat JNICALL Java_com_jni_example_TemperatureSampler_getTemperature (JNIEnv * env, jobject thisObject) {
    std::cout<<"Returning Simple Temperature :p" <<std::endl;
    return 27.8;
}


jobject getAJANStateFromState(despot::State& state){

//    int state = getAJANStateFromState();
    jclass ajanStateClass = globalEnv ->FindClass("de/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/implementation/AJAN_Agent_State");
    jobject ajanState = globalEnv->AllocObject(ajanStateClass);
    jfieldID state_id = globalEnv->GetFieldID(ajanStateClass, "state_id","I");
    jfieldID scenario_id = globalEnv->GetFieldID(ajanStateClass, "scenario_id","I");
    jfieldID weight = globalEnv->GetFieldID(ajanStateClass, "weight","D");
    globalEnv->SetIntField(ajanState, state_id, state.state_id);
    globalEnv->SetIntField(ajanState, scenario_id, state.scenario_id);
    globalEnv->SetDoubleField(ajanState, weight, state.weight);
    return ajanState;
}
JNIEXPORT jobject JNICALL Java_com_jni_example_TemperatureSampler_getDetailedTemperature (JNIEnv * env, jobject thisObject) {
    jclass temperatureDataClass = env ->FindClass("com/jni/example/TemperatureData");
    jobject temperatureData = env->AllocObject(temperatureDataClass);

    jclass temperatureScaleClass = env->FindClass("com/jni/example/TemperatureScale");

    jfieldID timestamp = env->GetFieldID(temperatureDataClass, "timestamp", "Ljava/lang/String;");
    jfieldID temperature = env->GetFieldID(temperatureDataClass, "temperature", "F");
    jfieldID scale = env->GetFieldID(temperatureDataClass, "scale", "Lcom/jni/example/TemperatureScale;");
    jfieldID scaleEnumID = env->GetStaticFieldID(temperatureScaleClass,"CELCIUS","Lcom/jni/example/TemperatureScale;");
    jobject celciusScale = env->GetStaticObjectField(temperatureScaleClass,scaleEnumID);
    //region hide
    jclass callerClass = env->GetObjectClass(thisObject);
    jmethodID preferredScaleMethodID = env->GetMethodID(callerClass,"getPreferredScale","()Lcom/jni/example/TemperatureScale;");
    jobject preferredScale = env->CallObjectMethod(thisObject,preferredScaleMethodID);

    if(!env->IsSameObject(preferredScale,celciusScale)){
        std::cout<<"Preferred scale is not supported, using CELCIUS instead!"<<std::endl;
    }

    env->SetObjectField(temperatureData,timestamp,env->NewStringUTF("02-03-2020 17:30:48"));
    env->SetFloatField(temperatureData,temperature,27.8);
    env->SetObjectField(temperatureData,scale,celciusScale);
    std::cout<<"Returning Detailed Temperature..." <<std::endl;
    TigerState state1;
//    state1.id = 100;
    state1.scenario_id = 100;
    state1.state_id = 1;
    state1.weight = 10;
    jclass javaClass = globalEnv->GetObjectClass(thisClassObject);
    jmethodID javaMethod = globalEnv->GetMethodID(javaClass,"getMessageString","()Ljava/lang/String;");
    jstring preferredScale1 =(jstring)globalEnv->CallObjectMethod(thisClassObject,javaMethod);
    const char* stringValues = globalEnv->GetStringUTFChars(preferredScale1,NULL);
    std::cout<<"Gote Preferred Scale:"<<stringValues<<std::endl;

    ACT_TYPE temp = 10;
    OBS_TYPE obs = 100;
    double reward = 100;
    jobject state = getAJANStateFromState(state1);
    jclass javaClass1 = globalEnv->GetObjectClass(thisClassObject);
    jmethodID javaMethod1 = globalEnv->GetMethodID(javaClass1,"setParams","(IDILde/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/implementation/AJAN_Agent_State;)I");
    const char* str = "Message from c++";
    jstring javaString = globalEnv->NewStringUTF(str);
    ACT_TYPE preferredScale11 = (ACT_TYPE) globalEnv->CallIntMethod(thisClassObject,javaMethod1,temp,reward,obs, state);

    std::cout<<"Got Action Type Type:"<<preferredScale11<<std::endl;
    return temperatureData;
    //endregion
}

JNIEXPORT void JNICALL Java_com_jni_example_TemperatureSampler_InitializeObject
        (JNIEnv * env, jobject thisObject, jobject stateObject){
    thisClassObject = thisObject;
}
//JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
//    JNIEnv* env;
//    if(vm->GetEnv(reinterpret_cast<void**>(&env),JNI_VERSION_1_6)!=JNI_OK){
//        std::cout<<"Initializing global"<<std::endl;
//        return JNI_ERR;
//    }
//    std::cout<<"Initializing global Env variable"<<std::endl;
//    globalEnv = env;
////    jclass javaClass = globalEnv->FindClass("com/jni/example/TemperatureSampler");
////    jmethodID javaMethod = globalEnv->GetMethodID(javaClass,"getMessageString","()Ljava/lang/String;");
////    jstring preferredScale = static_cast<jstring>(env->CallObjectMethod(javaClass,javaMethod));
////    std::cout<<"Got Preferred Scale:"<<preferredScale<<std::endl;
//    return JNI_VERSION_1_6;
//}



