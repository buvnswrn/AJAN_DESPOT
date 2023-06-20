//
// Created by bhuvanesh on 19.06.23.
//

#include "AjanPolicy.h"
using namespace std;
namespace despot {
        AjanPolicy::AjanPolicy(const DSPOMDP *model, ParticleLowerBound *bound) :
        DefaultPolicy(model,bound),
        agent_model_(static_cast<const AJANAgent*>(model)){
                floor_ = agent_model_->floor();
        }

        ACT_TYPE AjanPolicy::Action(const vector<State*>& particles, RandomStreams& streams,
                        History& history) const {
            if(history.Size() == 0) {
                return Random::RANDOM.NextInt(agent_model_->NumActions() -1);
            }

            Coord rob;
            if(agent_model_->same_loc_obs_ != floor_.NumCells()){
                rob = agent_model_->MostLikelyRobPosition(particles);
            } else {
                rob = floor_.GetCell(history.LastObservation());
            }

            Coord opp;
            opp = agent_model_->MostLikelyOpponentPosition(particles);

            double distance = Coord::ManhattanDistance(rob, opp);

            if(distance<=1){
                return agent_model_->TagAction();
            }

            vector<ACT_TYPE> actions;

            for (int d = 0; d < 4; d++) {
                if(!Compass::Opposite(d, history.LastAction())
                    && floor_.Inside(rob+ Compass::DIRECTIONS[d])) {
                    actions.push_back(d);
                }
            }

            if(actions.size() == 0){
                for (int d = 0; d < 4; d++) {
                    if(floor_.Inside(rob+Compass::DIRECTIONS[d]))
                        actions.push_back(d);
                }
            }

            if (actions.size() == 0)
                return 0;

            ACT_TYPE action = actions[Random::RANDOM.NextInt(actions.size())];
            return action;
        }
};