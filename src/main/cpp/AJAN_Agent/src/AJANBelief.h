//
// Created by bhuvanesh on 22.06.23.
//
#include "ajan_agent.h"
#include <despot/core/particle_belief.h>
#ifndef POMDP_AJANBELIEF_H
#define POMDP_AJANBELIEF_H


//using namespace std;

namespace despot {
    class AJANBelief:public ParticleBelief{
    private:
        const AJANAgent* agent_model_;
    public:
        AJANBelief(std::vector<State*>particles, const AJANAgent* model,Belief* prior=nullptr);
        void Update(ACT_TYPE action, OBS_TYPE obs);
    };
}

#endif //POMDP_AJANBELIEF_H
