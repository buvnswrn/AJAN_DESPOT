//
// Created by bhuvanesh on 19.06.23.
//

#include "AJANParticleUpperBound.h"

namespace despot {

    AJANParticleUpperBound::AJANParticleUpperBound(const AJANAgent* model):
            agent_model_(model){
        Floor floor = agent_model_->floor_;
        value_.resize(agent_model_->NumStates());
        for(int s=0; s<agent_model_->NumStates(); s++){
            int rob = agent_model_->rob_[s], opp= agent_model_->opp_[s];
            int dist = (int) floor.Distance(rob, opp);
            value_[s] = -(1-Globals::Discount(dist)) /(1-Globals::Discount())
                        + agent_model_->TAG_REWARD * Globals::Discount(dist);
        }
    }

    double AJANParticleUpperBound::Value(const State& s) const {
        const AJANAgentState& state = static_cast<const AJANAgentState&>(s);
        return value_[state.state_id];
    }
}