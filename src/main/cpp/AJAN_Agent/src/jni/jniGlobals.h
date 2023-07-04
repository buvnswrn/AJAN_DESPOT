//
// Created by bhuvanesh on 24.06.23.
//
#include <string>

using namespace std;

#ifndef POMDP_JNIGLOBALS_H
#define POMDP_JNIGLOBALS_H

const string STRING = "java/lang/String";
extern const string VECTOR = "java/util/Vector";
const string OBJECT = "java/lang/Object";

const string PACKAGE_NAME = "de/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/";
const string IMPL_PKG = PACKAGE_NAME + "implementation/";
const string UTIL_PKG = PACKAGE_NAME + "util/";

const string STATE = PACKAGE_NAME + "State";
const string HISTORY = PACKAGE_NAME + "History";
const string BELIEF = PACKAGE_NAME + "Belief";
const string VALUED_ACTION = "de/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/VALUED_ACTION";

const string AJAN_AGENT = IMPL_PKG + "AJAN_Agent";
extern const string AJAN_AGENT_STATE = IMPL_PKG + "AJAN_Agent_State";
extern const string AJAN_PARTICLE_UPPER_BOUND = IMPL_PKG + "AJANParticleUpperBound";
extern const string AJAN_POLICY = IMPL_PKG + "AJANPolicy";

const string COORD = UTIL_PKG + "Coord";
const string FLOOR = UTIL_PKG + "Floor";


#endif //POMDP_JNIGLOBALS_H
