//
// Created by bhuvanesh on 19.06.23.
//
#include "ajan_agent.h"
#include <despot/solver/pomcp.h>
#ifndef POMDP_AJANPOMCPPRIOR_H
#define POMDP_AJANPOMCPPRIOR_H


namespace despot {
    class AJANPOMCPPrior:public POMCPPrior {
    private:
        const AJANAgent* agent_model_;
    public:
        AJANPOMCPPrior(const DSPOMDP* model);
        void ComputePreference(const State& state);
    };
}

#endif //POMDP_AJANPOMCPPRIOR_H
