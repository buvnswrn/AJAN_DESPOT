//
// Created by bhuvanesh on 25.05.23.
//
#include "de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_AJANPlanner.h"
#include <despot/planner.h>
#include "ajan_agent.h"
#include "AJANWorld.h"
#include <jni.h>
#include "jni/ajanHelpers.h"


using namespace despot;

class AJANPlanner: public Planner {
public:
    //endregion
    AJANPlanner(){
        cout<<"atemmpting to connect to world"<<endl;
        cout<<"got world"<<endl;
        getEnv()->CallVoidMethod(*getAjanPlannerObject(), getMethodID("Planner","PrintMethod"));
        return;
    }

    DSPOMDP* InitializeModel(option::Option* options) override {
        // Initialize POMDP Model here;
        DSPOMDP* model = new AJANAgent();
        //Error from here
        return model;
    }

        World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options) override {
//        world_type = "simulator";
        cout <<"Trying to get worldtype"<<endl;
        world_type = getJstring("getWorldType","()Ljava/lang/String;");
        if(world_type == "pomdp"){
            return InitializePOMDPWorld(world_type, model, options);
        } else {
            cout<<"Create a world as defined and implemented by the user"<<endl;
            AJANWorld* world = new AJANWorld();
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

    std::string ChooseSolver() override {
        std::string solverName = getJstring("ChooseSolver", "()Ljava/lang/String;");
        std::cout<<"Received:"<<solverName<<std::endl;
        return solverName;
    }

    void InitializeDefaultParameters() override {
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
        jstring solverName = (jstring) getEnv()->CallObjectMethod(*getAjanPlannerObject(), getMethodID("Planner",methodName));
        return getEnv() ->GetStringUTFChars(solverName, nullptr);
    }

    //endregion
};

// region JNI Methods
JNIEXPORT jint JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_AJANPlanner_RunPlanner
(JNIEnv * Env, jobject thisObject, jobject agentObject, jobject worldObject) {
    char* argv[] = {strdup("AJAN_Planner") };
    std::cout<<"Starting the DESPOT planner"<<std::endl;
    Init(Env, &thisObject, &agentObject, &worldObject);
    getEnv()->CallVoidMethod(*getAjanPlannerObject(), getMethodID("Planner","PrintMethod"));
    return AJANPlanner().RunEvaluation(1,argv);
}

JNIEXPORT void JNICALL Java_de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_AJANPlanner_InitializePlannerInDespot
        (JNIEnv * env, jobject thisObject){
//    javaGlobalEnv = env;
//    javaPlannerObject = thisObject;
}

//endregion


