//
// Created by bhuvanesh on 19.06.23.
//

#include "AJANPOMCPPrior.h"
#include <despot/solver/pomcp.h>

namespace despot {
    AJANPOMCPPrior::AJANPOMCPPrior(const DSPOMDP *model) :
            POMCPPrior(model),
            agent_model_(static_cast<const AJANAgent *>(model)) {
    }

    void AJANPOMCPPrior::ComputePreference(const State &state) {
//        Coord rob = agent_model_->GetRobPos(&state);
//
//        legal_actions_.clear();
//        preferred_actions_.clear();
//
//        for (int a = 0; a < 5; a++) {
//            legal_actions_.push_back(a);
//        }
//
//        if(history_.Size() !=0){
//            if(history_.LastObservation() == agent_model_->same_loc_obs_) {
//                preferred_actions_.push_back(agent_model_->TagAction());
//            } else {
//                if(agent_model_->robot_pos_unknown_){
//                    for (int a = 0; a < 4; a++) {
//                        if(agent_model_->floor_.Inside(
//                                rob + Compass::DIRECTIONS[a])) {
//                            if(!Compass::Opposite(a, history_.LastAction()))
//                                preferred_actions_.push_back(a);
//                        }
//                    }
//                }
//            }
//        }
    }
};
