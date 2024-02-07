//
// Created by bhuvanesh on 25.05.23.
//
#include <jni.h>
#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>
#include <despot/util/coord.h>
#include <despot/util/floor.h>
#include "de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_implementation_AJAN_Agent.h"
#ifndef DESPOT_AJAN_AGENT_H
#define DESPOT_AJAN_AGENT_H

namespace despot {
    const std::string AGENT = "Agent";
    class AJANAgentState:public State {
        public:
            JNIEnv* javaEnv;
            jobject javaStateObject;

//            int agent_position;
            // we can use a map/dict here to store multiple variables; Map<*char,ACT_TYPE>
            AJANAgentState();
            AJANAgentState(int _state_id);
            AJANAgentState(JNIEnv * env, jobject stateObject);

        std::string text() const;
    };

    class AJANAgent:public MDP, public StateIndexer, public StatePolicy, public BeliefMDP, public MMAPInferencer {
    private:
        mutable MemoryPool<AJANAgentState> memory_pool_;
    public:

//        static AJANAgent* current_;
//        static double TAG_REWARD;
//        Floor floor_;
//        bool robot_pos_unknown_;
//        std::vector<AJANAgentState*> states_;
//        std::vector<int> rob_;
//        std::vector<int> opp_;
//        std::vector<std::vector<std::vector<State>>> transition_probabilities_;
//        std::vector<State> tempStateVector;
//        static int NBEAMS;

//        static int BITS_PER_READING;
        uint64_t same_loc_obs_;
//        std::vector<std::vector<std::vector<double>>> reading_distributions_;

//        double unit_size;
//        double noise_sigma_;

        AJANAgent();
        AJANAgent(JNIEnv* javaEnv, jobject* javaAgentObject);
        //region POMDP Functions
        bool Step(State& state, double random_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const;
        int NumStates() const;
        int NumActions() const;
        double ObsProb(OBS_TYPE obs, const State& s, ACT_TYPE a) const;
//        State* CreateStartState(std::string type) const;
        State* CreateStartState(std::string type = "DEFAULT") const;
        Belief* InitialBelief(const State* start, std::string type="DEFAULT") const;
        double GetMaxReward() const;
        double Reward(int s, ACT_TYPE action) const;
        ValuedAction GetBestAction() const;
        int GetAction(const State& state) const;
        ScenarioLowerBound* CreateScenarioLowerBound(std::string name="DEFAULT",
                                                     std::string particle_bound_name="DEFAULT") const;
        ScenarioUpperBound* CreateScenarioUpperBound(std::string name="DEFAULT",
                                                     std::string particle_bound_name="DEFAULT") const;
        ParticleUpperBound* CreateParticleUpperBound(std::string name) const;

//        POMCPPrior* CreatePOMCPPrior(std::string name = "DEFAULT") const;
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
        inline int GetIndex(const State* state) const {
            return state->state_id;
        };

        const State* GetState(int index) const;
//        {

//            return states_[index];
//        };

        Belief *Tau(const Belief *belief, ACT_TYPE action, OBS_TYPE obs) const ;

        void Observe(const Belief* belief, ACT_TYPE action,
                     std::map<OBS_TYPE, double>& obss) const ;

        double StepReward(const Belief* belief, ACT_TYPE action) const ;

        const std::vector<State> &TransitionProbability(int s, ACT_TYPE a) const;
        const State* GetMMAP(const std::vector<State*>& particles) const;
        //region AJAN Methods
        bool
        getAJANStep(AJANAgentState &s, double random_num, ACT_TYPE action, double &reward, OBS_TYPE &obs, const char *methodName,
                    const char *returnType) const;

        jobject convertToAJANAgentState(const AJANAgentState &state) const;

        void printInJava(const char *methodName, const char *returnType, jobject pJobject) const;

        void printInJava(const char *methodName, const char *returnType, jobject pJobject, const OBS_TYPE obs) const;

        void printInJava(const char *methodName, const char *returnType, ACT_TYPE pJobject) const;

        void UpdateValues(AJANAgentState &state, double &reward, OBS_TYPE &obs) const;

        void UpdateStateValues(AJANAgentState &state, jobject pJobject) const;

        int getAJANNum(const char* methodName, const char* returnType) const;

        double getAJANObsProb(OBS_TYPE obs, const AJANAgentState &state, ACT_TYPE a) const;

        double getAJANMaxReward() const;

//        ValuedAction getAJANBestAction() const;

        AJANAgentState *getAJANStartState(std::string type) const;

        std::vector<State *> getAJANParticles(const State *pState, std::string type) const;

        jobject getAJANStateFromState(const State *state) const ;

        AJANAgentState *getAgentStateFromAJANState(jobject valuedAction, bool needAllocation) const;
        //endregion
        // region helper methods
//        int StateIndexToRobIndex(int i) const;
//
//        int StateIndexToOppIndex(int index) const;
//
//        Coord GetRobPos(const State *state) const;
//
//        void Init(std::istream& iss);
//
//        void ReadConfig(std::istream& is);
//
//        int RobOppIndicesToStateIndex(int rob, int opp) const;
//
//        std::map<int, double> OppTransitionDistribution(int s);
//
//        int NextRobPosition(int i, int i1, int a) const;
//        std::string RandomMap(int height, int width, int obstacles);
//
//        int TagAction() const;
//
//
//
//        void SetReading(uint64_t obs, uint64_t reading, uint64_t dir) const;
//
//
//
//        std::string BenchmarkMap();
//
//        double LaserRange(const State &state, int dir) const;
//
//        int GetReading(uint64_t obs, int dir) const;

        mutable std::vector<int> default_action_;

        void ComputeDefaultActions(std::string type) const;

//        const Floor floor() const;
//
//        Coord MostLikelyRobPosition(const std::vector<State *> &vector) const;
//
//        Coord MostLikelyOpponentPosition(const std::vector<State *> &particles) const;
//        //endregion
//
//        void NoiseSigma(double noise_sigma);
//
//        bool BaseStep(State &state, double aDouble, ACT_TYPE action, double &reward) const;


        const std::vector<State> &getStateVectorFromAJANStateVector(jobject transProb) const;

        jobject getUpdatedCurrentAJANState(State state) const;

        jobject getAjanAgentStateVectorFromStateVector(const std::vector<State *> &vector) const;

        std::vector<State *> getStatePointerVectorFromAJANStateVector(jobject transProb) const;

//        void convertAJANStateToState(jobject transProb, jmethodID getMethod, int i, State *cstate) const;
        ValuedAction getAJANBestAction() const;
    };

}

#endif //DESPOT_AJAN_AGENT_H
