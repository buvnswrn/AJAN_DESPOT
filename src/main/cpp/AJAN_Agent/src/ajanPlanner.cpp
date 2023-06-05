//
// Created by bhuvanesh on 25.05.23.
//
#include "com_ajan_POMDP_AJANPlanner.h"
#include <despot/planner.h>
#include "ajan_agent.h"
#include <jni.h>


using namespace despot;
// region JNI Bridge
JNIEnv* javaGlobalEnv;
jobject javaPlannerObject;
jobject javaAgentObject;
jobject javaStateObject;

class AJANPlanner: public Planner {
public:
    JNIEnv* javaEnv;
    jobject javaPlannerObject;
    jobject* javaAgentObject;
    jobject javaStateObject;
    //endregion
    AJANPlanner(JNIEnv *& env, jobject thisObject,jobject * agentObject){
        AJANPlanner::javaEnv = env;
        AJANPlanner::javaPlannerObject = thisObject;
        AJANPlanner::javaAgentObject = agentObject;
        jclass javaClass = AJANPlanner::javaEnv->GetObjectClass(thisObject);
        jmethodID javaMethod = AJANPlanner::javaEnv->GetMethodID(javaClass,"PrintMethod","()V");
        AJANPlanner::javaEnv->CallVoidMethod(thisObject,javaMethod);
        return;
    }

    DSPOMDP* InitializeModel(option::Option* options){
        // Initialize POMDP Model here;
        DSPOMDP* model = new AJANAgent(AJANPlanner::javaEnv,AJANPlanner::javaAgentObject); //Error from here
        return model;
    }

    World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options){
        return InitializePOMDPWorld(world_type, model, options);
    }

    std::string ChooseSolver(){
        std::string solverName = getJstring("ChooseSolver", "()Ljava/lang/String;");
        std::cout<<"Received:"<<solverName<<std::endl;
        return solverName;
    }



    void InitializeDefaultParameters() {
    }

    //endregion
    // region Helper Functions
    std::string getJstring(const char *methodName, const char *returnType) {
        jclass javaClass = javaEnv->GetObjectClass(javaPlannerObject);
        jmethodID javaMethod = javaEnv->GetMethodID(javaClass , methodName, returnType);
        jstring solverName = (jstring) javaEnv->CallObjectMethod(javaPlannerObject, javaMethod);
        return std::string(javaEnv ->GetStringUTFChars(solverName, NULL));
    }

    //endregion
};

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
    JNIEnv* env;
    if(vm->GetEnv(reinterpret_cast<void**>(&env),JNI_VERSION_1_6)!=JNI_OK){
        std::cout<<"Initializing global"<<std::endl;
        return JNI_ERR;
    }
    std::cout<<"Initializing global Env variable"<<std::endl;
    javaGlobalEnv = env;
//    jclass javaClass = globalEnv->FindClass("com/jni/example/TemperatureSampler");
//    jmethodID javaMethod = globalEnv->GetMethodID(javaClass,"getMessageString","()Ljava/lang/String;");
//    jstring preferredScale = static_cast<jstring>(env->CallObjectMethod(javaClass,javaMethod));
//    std::cout<<"Got Preferred Scale:"<<preferredScale<<std::endl;
    return JNI_VERSION_1_6;
}
// region JNI Methods
JNIEXPORT jint JNICALL Java_com_ajan_POMDP_AJANPlanner_RunPlanner(JNIEnv * Env, jobject thisObject, jobject agentObject) {
    char* argv[] = {strdup("AJAN_Planner") };
    std::cout<<"Starting the DESPOT planner"<<std::endl;
    jclass javaClass = Env->GetObjectClass(thisObject);
    jmethodID javaMethod = Env->GetMethodID(javaClass,"PrintMethod","()V");
    Env->CallVoidMethod(thisObject,javaMethod);
    javaGlobalEnv = Env;
    return AJANPlanner(Env, thisObject,&agentObject).RunEvaluation(1,argv);
}

JNIEXPORT void JNICALL Java_com_ajan_POMDP_AJANPlanner_InitializePlannerInDespot
        (JNIEnv * env, jobject thisObject){
    javaGlobalEnv = env;
    javaPlannerObject = thisObject;
}


