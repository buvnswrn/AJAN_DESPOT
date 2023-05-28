//
// Created by bhuvanesh on 25.05.23.
//

#include <despot/planner.h>
#include "ajan_agent.h"
#include <jni.h>
#include "com_ajan_POMDP_AJANPlanner.h"

using namespace despot;
class AJANPlanner: public Planner {
public:
    // region JNI Bridge
    JNIEnv* javaEnv;
    jobject javaPlannerObject;
    jobject javaAgentObject;
    jobject javaStateObject;

    //endregion
    AJANPlanner(){

    }

    DSPOMDP* InitializeModel(option::Option* options){
        // Initialize POMDP Model here;
        DSPOMDP* model = new AJANAgent(javaEnv,javaAgentObject);
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

    // region JNI Methods
    JNIEXPORT jint JNICALL Java_com_ajan_POMDP_AJANPlanner_RunPlanner(JNIEnv * Env, jobject thisObject) {
        char* argv[] = { "AJAN_Planner" };
        std::cout<<"Starting the DESPOT planner"<<std::endl;
        return AJANPlanner().RunEvaluation(1,argv);
    }

    JNIEXPORT void JNICALL Java_com_ajan_POMDP_AJANPlanner_InitializePlannerInDespot
            (JNIEnv * env, jobject thisObject){
        javaEnv = env;
        javaPlannerObject = thisObject;
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

