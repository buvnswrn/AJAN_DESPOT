//
// Created by bhuvanesh on 19.06.23.
//

#ifndef POMDP_AJANPARTICLEUPPERBOUND_H
#define POMDP_AJANPARTICLEUPPERBOUND_H
#include <despot/core/builtin_upper_bounds.h>
#include "ajan_agent.h"

using namespace std;
namespace despot {
    class AJANParticleUpperBound : public ParticleUpperBound {
    protected:
        const AJANAgent* agent_model_;
        vector<double> value_;
    public:
        AJANParticleUpperBound(const AJANAgent* model);
        double Value(const State& s) const;
    };
}

#endif //POMDP_AJANPARTICLEUPPERBOUND_H
