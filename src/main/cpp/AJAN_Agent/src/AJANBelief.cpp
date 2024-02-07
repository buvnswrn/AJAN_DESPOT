//
// Created by bhuvanesh on 22.06.23.
//

#include "AJANBelief.h"

namespace despot {
    AJANBelief::AJANBelief(std::vector<State*>particles, const AJANAgent* model,Belief* prior):
            ParticleBelief(particles,model, prior, false),
            agent_model_(model){
    }

    void AJANBelief::Update(despot::ACT_TYPE action, despot::OBS_TYPE obs) {
        Belief* updated= agent_model_->Tau(this, action, obs);
        for (int i = 0; i < particles_.size(); i++)
            agent_model_->Free(particles_[i]);
        particles_.clear();

        const std::vector<State*>& new_particles = static_cast<ParticleBelief*>(updated)->particles();

        for (int i = 0; i < new_particles.size(); i++)
            particles_.push_back(agent_model_->Copy(new_particles[i]));

        delete updated;
    }
}