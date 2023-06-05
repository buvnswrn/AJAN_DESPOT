//
// Created by bhuvanesh on 25.05.23.
//
#include <jni.h>
#ifndef DESPOT_AJAN_AGENT_H
#define DESPOT_AJAN_AGENT_H
#include <despot/interface/pomdp.h>
namespace despot {

    class AJANAgentState:public State {
        public:
            JNIEnv* javaEnv;
            jobject javaStateObject;
            int agent_position;
            // we can use a map/dict here to store multiple variables; Map<*char,ACT_TYPE>
            AJANAgentState();
            AJANAgentState(int _state_id);
            AJANAgentState(JNIEnv * env, jobject stateObject);

        std::string text() const;
    };

    class AJANAgent:public DSPOMDP {
    private:
        mutable MemoryPool<AJANAgentState> memory_pool_;
    public:
        JNIEnv* javaEnv;
        jobject javaAgentObject;
        static const ACT_TYPE LEFT, RIGHT, HOVER;
        static const double NOISE;
        AJANAgent();
        AJANAgent(JNIEnv* javaEnv, jobject* javaAgentObject);
        //region POMDP Functions
        bool Step(State& s, double random_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const;
        int NumStates() const;
        int NumActions() const;
        double ObsProb(OBS_TYPE obs, const State& s, ACT_TYPE a) const;
//        State* CreateStartState(std::string type) const;
        State* CreateStartState(std::string type = "DEFAULT") const;
        Belief* InitialBelief(const State* start, std::string type="DEFAULT") const;
        double GetMaxReward() const;
        ValuedAction GetBestAction() const;
        ScenarioLowerBound* CreateScenarioLowerBound(std::string name="DEFAULT",
                                                     std::string particle_bound_name="DEFAULT") const;
        //endregion
        //region Print Functions
        void PrintState(const State& state, std::ostream& out = std::cout) const;
        void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
        void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const;
        void PrintAction(ACT_TYPE action,std::ostream& out = std::cout) const;
        //endregion
        //region Memory Functions
        State* Allocate(int state_id, double weight) const;
        State* Copy(const State* particle) const;
        void Free(State* particle) const;
        int NumActiveParticles() const;
        //endregion

        bool
        getAJANStep(AJANAgentState &s, double random_num, ACT_TYPE action, double &reward, OBS_TYPE &obs, const char *methodName,
                    const char *returnType) const;

        jobject convertToAJANAgentState(const AJANAgentState &state) const;

        void printInJava(const char *methodName, const char *returnType, jobject pJobject) const;

        void printInJava(const char *methodName, const char *returnType, jobject pJobject, const OBS_TYPE obs) const;

        void printInJava(const char *methodName, const char *returnType, ACT_TYPE pJobject) const;

        void UpdateValues(AJANAgentState &state, double &reward, OBS_TYPE &obs) const;

        void UpdateStateValues(AJANAgentState &state, jobject pJobject) const;
    };
}

#endif //DESPOT_AJAN_AGENT_H
