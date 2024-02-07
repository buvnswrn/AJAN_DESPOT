//
// Created by bhuvanesh on 19.06.23.
//
#include <despot/util/floor.h>
#include "ajan_agent.h"
#ifndef POMDP_AJANPOLICY_H
#define POMDP_AJANPOLICY_H
namespace despot {
    class AjanPolicy: public DefaultPolicy{
    private:
        const AJANAgent *agent_model_;
        Floor floor_;
    public:
        AjanPolicy(const DSPOMDP* model, ParticleLowerBound* bound);

        ACT_TYPE Action(const std::vector<State *> &particles, RandomStreams &streams, History &history) const;
    };
}

#endif //POMDP_AJANPOLICY_H
