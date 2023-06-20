//
// Created by bhuvanesh on 25.05.23.
//
#include "de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_AJANPlanner.h"
#include <despot/planner.h>
#include "ajan_agent.h"
#include "AJANWorld.h"
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
    jobject*  javaWorldObject;
    //endregion
    AJANPlanner(JNIEnv *& env, jobject thisObject,jobject * agentObject, jobject * worldObject){
        AJANPlanner::javaEnv = env;
        AJANPlanner::javaPlannerObject = thisObject;
        AJANPlanner::javaAgentObject = agentObject;
        AJANPlanner::javaWorldObject = worldObject;
        jclass javaClass = AJANPlanner::javaEnv->GetObjectClass(thisObject);
        cout<<"atemmpting to connect to world"<<endl;
        cout<<"got world"<<endl;
        jmethodID javaMethod = AJANPlanner::javaEnv->GetMethodID(javaClass,"PrintMethod","()V");
        AJANPlanner::javaEnv->CallVoidMethod(thisObject,javaMethod);
        return;
    }

    DSPOMDP* InitializeModel(option::Option* options){
        // Initialize POMDP Model here;
        DSPOMDP* model = new AJANAgent(AJANPlanner::javaEnv,AJANPlanner::javaAgentObject);
        //Error from here
        return model;
    }

        World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options){
//        world_type = "simulator";
        cout <<"Trying to get worldtype"<<endl;
        world_type = getJstring("getWorldType","()Ljava/lang/String;");
        if(world_type == "pomdp"){
            return InitializePOMDPWorld(world_type, model, options);
        } else {
            cout<<"Create a world as defined and implemented by the user"<<endl;
            AJANWorld* world = new AJANWorld(AJANPlanner::javaEnv, AJANPlanner::javaWorldObject);
            cout<<"Establish connection with external system"<<endl;
            world->Connect();
            cout<<"Initialize the state of the external system"<<endl;
            world->Initialize();
            cout<<"Initialized"<<endl;
            static_cast<AJANAgent*>(model)->NoiseSigma(world->noise_sigma_);
            cout<<"Assigned Noise sigma"<<endl;
            return world;
        }
//        return InitializePOMDPWorld(world_type, model, options);
    }

    std::string ChooseSolver(){
        std::string solverName = getJstring("ChooseSolver", "()Ljava/lang/String;");
        std::cout<<"Received:"<<solverName<<std::endl;
        return solverName;
    }



    void InitializeDefaultParameters() {
        Globals::config.pruning_constant = 0.01; // for laser_tag
    }

//    void PlanningLoop(Solver*& solver, World* world, Logger* logger){
//        for (int i = 0; i < Globals::config.sim_len; i++) {
//            bool terminal = RunStep(solver, world, logger);
//            if(terminal)
//                break;
//        }
//    }

//    FIXME: See whether this can be executed in Java side

//    bool RunStep(Solver* solver, World* world, Logger* logger){
//
//    }
    void PlanningLoop(Solver*& solver, World* world, Logger* logger) {
        for (int i = 0; i < Globals::config.sim_len; i++) {
            bool terminal = RunStep(solver, world, logger);
            if (terminal)
                break;
        }
    }

    bool RunStep(Solver* solver, World* world, Logger* logger) {
        cout<<"Running step"<<endl;
        logger->CheckTargetTime();

        double step_start_t = get_time_second();

        double start_t = get_time_second();
        cout<<"Solver search started"<<endl;
        ACT_TYPE action = solver->Search().action;
        cout<<"Solver search finished"<<endl;
        double end_t = get_time_second();
        double search_time = (end_t - start_t);
        logi << "[Custom RunStep] Time spent in " << typeid(*solver).name()
             << "::Search(): " << search_time << endl;

        OBS_TYPE obs;
        start_t = get_time_second();
        cout<<"Executing action"<<endl;
        bool terminal = world->ExecuteAction(action, obs);
        end_t = get_time_second();
        double execute_time = (end_t - start_t);
        logi << "[Custom RunStep] Time spent in ExecuteAction(): " << execute_time << endl;

        start_t = get_time_second();
        solver->BeliefUpdate(action, obs);
        end_t = get_time_second();
        double update_time = (end_t - start_t);
        logi << "[Custom RunStep] Time spent in Update(): " << update_time << endl;

        return logger->SummarizeStep(step_++, round_, terminal, action, obs,
                                     step_start_t);
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

// region JNI Methods
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
JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_AJANPlanner_RunPlanner
(JNIEnv * Env, jobject thisObject, jobject agentObject, jobject worldObject) {
    char* argv[] = {strdup("AJAN_Planner") };
    std::cout<<"Starting the DESPOT planner"<<std::endl;
    jclass javaClass = Env->GetObjectClass(thisObject);
    jmethodID javaMethod = Env->GetMethodID(javaClass,"PrintMethod","()V");
    Env->CallVoidMethod(thisObject,javaMethod);
    javaGlobalEnv = Env;
    return AJANPlanner(Env, thisObject,&agentObject, &worldObject).RunEvaluation(1,argv);
}

JNIEXPORT void JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_AJANPlanner_InitializePlannerInDespot
        (JNIEnv * env, jobject thisObject){
    javaGlobalEnv = env;
    javaPlannerObject = thisObject;
}

//endregion


